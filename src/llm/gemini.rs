//! Google Gemini `generateContent` backend.
//!
//! `POST https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent?key=API_KEY`.
//! Blocking HTTP via `ureq`. Prompt caching on the 2.5-series models is
//! implicit — identical prefixes are auto-cached for the same key with
//! no explicit `cachedContents` roundtrip needed. Tool calls come back
//! without an `id`, so we synthesize stable ids so the next turn's
//! `functionResponse` can reference the right call by name.

use std::collections::HashMap;
use std::env;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, Result};
use serde_json::{json, Value};

use crate::llm::backend::{
    write_transcript, CacheSnapshot, CacheStats, LlmBackend, LlmRequest, LlmResponse, LlmUsage,
    Message, Part, Role, ToolDef,
};

const API_BASE: &str = "https://generativelanguage.googleapis.com/v1beta/models";

pub struct GeminiBackend {
    pub model: String,
    pub api_key: Option<String>,
    cache_stats: Arc<CacheStats>,
    pub transcript_path: Option<PathBuf>,
    /// Monotonic counter for synthesized tool-call ids. Persists across
    /// calls within one session so each id is globally unique.
    call_counter: Arc<AtomicU64>,
    /// Synthesized-id → opaque `thoughtSignature` captured from model
    /// `functionCall` parts. Gemini 3 hard-rejects a request that
    /// replays a historical functionCall without its original signature.
    signatures: Arc<Mutex<HashMap<String, String>>>,
    /// Sanitized `functionDeclarations` JSON. Tool definitions are
    /// byte-stable for the lifetime of the backend, so we walk + clone
    /// each schema once instead of every turn.
    tools_cache: Arc<Mutex<Option<Vec<Value>>>>,
}

impl GeminiBackend {
    pub fn new(model: impl Into<String>) -> Self {
        Self {
            model: model.into(),
            api_key: None,
            cache_stats: Arc::new(CacheStats::default()),
            transcript_path: None,
            call_counter: Arc::new(AtomicU64::new(0)),
            signatures: Arc::new(Mutex::new(HashMap::new())),
            tools_cache: Arc::new(Mutex::new(None)),
        }
    }
}

impl LlmBackend for GeminiBackend {
    fn create_response(&self, req: &LlmRequest<'_>) -> Result<LlmResponse> {
        let api_key = self
            .api_key
            .clone()
            .or_else(|| env::var("GEMINI_API_KEY").ok())
            .ok_or_else(|| anyhow!("GEMINI_API_KEY is required for the Gemini backend"))?;

        let signatures_guard = self.signatures.lock().unwrap();
        let (system_instruction, contents) = encode_messages(req.messages, &signatures_guard);
        drop(signatures_guard);
        let tools_json = self
            .tools_cache
            .lock()
            .unwrap()
            .get_or_insert_with(|| encode_tools(req.tools))
            .clone();

        let mut payload = json!({
            "contents": contents,
            "tools": tools_json,
            "toolConfig": {"functionCallingConfig": {"mode": "AUTO"}},
        });
        if let Some(sys) = system_instruction {
            payload["systemInstruction"] = sys;
        }
        if let Some(effort) = req.reasoning_effort {
            // `includeThoughts` opts into the `thought: true` summary
            // parts; without it Gemini returns only the opaque
            // `thoughtSignature` blobs.
            payload["generationConfig"] = json!({
                "thinkingConfig": {
                    "thinkingBudget": effort.token_budget(),
                    "includeThoughts": true,
                }
            });
        }

        if let Some(path) = &self.transcript_path {
            write_transcript(path, &payload, None);
        }

        let url = format!("{}/{}:generateContent?key={}", API_BASE, self.model, api_key);
        let http = ureq::post(&url)
            .set("Content-Type", "application/json")
            .timeout(std::time::Duration::from_secs(req.timeout_secs));
        // Log URL w/o the API key
        let log_url = format!("{}:generateContent", self.model);
        let resp =
            crate::llm::backend::send_with_body_on_error(http, &payload.to_string(), &log_url)?;
        let raw: Value = resp.into_json()?;
        let usage = parse_usage(&raw);
        self.cache_stats.record(&usage);
        if let Some(path) = &self.transcript_path {
            write_transcript(path, &payload, Some(&raw));
        }
        let (output, thoughts) =
            parse_output(&raw, &self.call_counter, &self.signatures)?;
        Ok(LlmResponse {
            output,
            usage,
            raw,
            thoughts,
        })
    }

    fn cache_snapshot(&self) -> CacheSnapshot {
        self.cache_stats.snapshot()
    }

    fn cache_stats_arc(&self) -> Option<Arc<CacheStats>> {
        Some(self.cache_stats.clone())
    }
}

// ---------- encoders ----------

/// Split the neutral `Message` list into Gemini's `systemInstruction`
/// plus the `contents` array. Gemini does not allow tool_result and
/// tool_call blocks inside a single Content — they live as siblings in
/// the parts array, and the role alternates between "user" (for
/// tool_result and plain user text) and "model" (for assistant text and
/// functionCall). Adjacent same-role messages are collapsed so the
/// alternation stays well-formed.
fn encode_messages(
    messages: &[Message],
    signatures: &HashMap<String, String>,
) -> (Option<Value>, Vec<Value>) {
    // First pass: pull out all system text into one systemInstruction.
    let mut system_text: Vec<String> = Vec::new();
    // Remember the name for each tool_call id we've seen so that a
    // later tool_result can fill in the Gemini-required `name` field.
    let mut id_to_name: std::collections::HashMap<String, String> = std::collections::HashMap::new();
    for msg in messages {
        if msg.role == Role::System {
            for part in &msg.content {
                if let Part::Text(t) = part {
                    system_text.push(t.clone());
                }
            }
            continue;
        }
        for part in &msg.content {
            if let Part::ToolCall { id, name, .. } = part {
                id_to_name.insert(id.clone(), name.clone());
            }
        }
    }
    let system_instruction = if system_text.is_empty() {
        None
    } else {
        Some(json!({
            "parts": [{"text": system_text.join("\n\n")}]
        }))
    };

    // Second pass: build `contents`, collapsing adjacent same-role
    // messages.
    let mut contents: Vec<Value> = Vec::new();
    for msg in messages {
        if msg.role == Role::System {
            continue;
        }
        let target_role = match msg.role {
            Role::Assistant => "model",
            _ => "user",
        };
        let mut parts: Vec<Value> = Vec::new();
        for part in &msg.content {
            match part {
                Part::Text(t) => parts.push(json!({"text": t})),
                Part::ToolCall { id, name, arguments } => {
                    let args: Value =
                        serde_json::from_str(arguments).unwrap_or_else(|_| json!({}));
                    let mut block = json!({
                        "functionCall": {"name": name, "args": args}
                    });
                    if let Some(sig) = signatures.get(id) {
                        block["thoughtSignature"] = Value::String(sig.clone());
                    }
                    parts.push(block);
                }
                Part::ToolResult { id, content } => {
                    let name = id_to_name
                        .get(id)
                        .cloned()
                        .unwrap_or_else(|| "unknown".to_string());
                    parts.push(json!({
                        "functionResponse": {
                            "name": name,
                            "response": {"result": content}
                        }
                    }));
                }
            }
        }
        if let Some(last) = contents.last_mut() {
            if last.get("role").and_then(|v| v.as_str()) == Some(target_role) {
                if let Some(existing) = last.get_mut("parts").and_then(|v| v.as_array_mut()) {
                    existing.extend(parts);
                    continue;
                }
            }
        }
        contents.push(json!({"role": target_role, "parts": parts}));
    }

    (system_instruction, contents)
}

fn encode_tools(tools: &[ToolDef]) -> Vec<Value> {
    let declarations: Vec<Value> = tools
        .iter()
        .map(|t| {
            let mut params = t.parameters.clone();
            sanitize_schema_for_gemini(&mut params);
            json!({
                "name": t.name,
                "description": t.description,
                "parameters": params,
            })
        })
        .collect();
    vec![json!({"functionDeclarations": declarations})]
}

/// Strip JSON Schema fields Gemini's OpenAPI-subset validator rejects,
/// and rewrite the ones it spells differently. The rest of the codebase
/// authors schemas in OpenAI's strict-mode dialect (`additionalProperties:
/// false`, `type: ["string", "null"]`, `enum: [..., null]`); Gemini wants
/// `nullable: true` and a scalar `type`, with no `additionalProperties`
/// anywhere.
fn sanitize_schema_for_gemini(v: &mut Value) {
    match v {
        Value::Object(map) => {
            map.remove("additionalProperties");
            if let Some(arr) = map.get("type").and_then(Value::as_array).cloned() {
                // JSON Schema's nullable convention is `"type": ["X",
                // "null"]` — the string literal "null" is a type-name
                // token, not JSON null. Strip it to get the concrete type
                // and surface the nullability via Gemini's `nullable: true`.
                let original_len = arr.len();
                let non_null: Vec<Value> = arr
                    .into_iter()
                    .filter(|x| x.as_str() != Some("null"))
                    .collect();
                if non_null.len() == 1 {
                    let had_null = non_null.len() != original_len;
                    map.insert("type".to_string(), non_null.into_iter().next().unwrap());
                    if had_null {
                        map.insert("nullable".to_string(), Value::Bool(true));
                    }
                }
            }
            if let Some(Value::Array(items)) = map.get_mut("enum") {
                items.retain(|x| !x.is_null());
            }
            for (_, child) in map.iter_mut() {
                sanitize_schema_for_gemini(child);
            }
        }
        Value::Array(items) => {
            for item in items.iter_mut() {
                sanitize_schema_for_gemini(item);
            }
        }
        _ => {}
    }
}

// ---------- decoders ----------

fn parse_usage(response: &Value) -> LlmUsage {
    let Some(usage) = response.get("usageMetadata") else {
        return LlmUsage::default();
    };
    let input_tokens = usage
        .get("promptTokenCount")
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let output_tokens = usage
        .get("candidatesTokenCount")
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let cached_tokens = usage
        .get("cachedContentTokenCount")
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    LlmUsage {
        input_tokens,
        cached_tokens,
        cache_creation_tokens: 0,
        output_tokens,
    }
}

/// Detect Gemini's leading `. ` / `? ` aside marker. Accepts any
/// whitespace after the marker char (Gemini frequently emits the marker
/// on its own line, i.e. `".\n<body>"`, not `". <body>"`). Returns the
/// body with the marker + leading whitespace stripped, or `None` for
/// anything else. Gemini-only quirk — other providers don't emit this
/// convention, so it's checked here in the backend rather than in the
/// generic conversation loop.
fn strip_thought_marker(text: &str) -> Option<String> {
    let leading = text.trim_start_matches([' ', '\t', '\n', '\r']);
    let mut chars = leading.chars();
    let marker = chars.next()?;
    if marker != '.' && marker != '?' {
        return None;
    }
    let after = chars.as_str();
    // Marker must be followed by whitespace, otherwise this is regular
    // prose (sentence-ending period, ellipsis, "?why?" etc.).
    if !after.starts_with(|c: char| c.is_whitespace()) {
        return None;
    }
    let trimmed = after.trim();
    if trimmed.is_empty() {
        None
    } else {
        Some(trimmed.to_string())
    }
}

fn parse_output(
    response: &Value,
    call_counter: &AtomicU64,
    signatures: &Mutex<HashMap<String, String>>,
) -> Result<(Vec<Part>, Vec<String>)> {
    let Some(candidates) = response.get("candidates").and_then(|v| v.as_array()) else {
        return Err(anyhow!(
            "Gemini response missing `candidates` array: {}",
            response
        ));
    };
    let Some(first) = candidates.first() else {
        return Ok((Vec::new(), Vec::new()));
    };
    let Some(parts) = first
        .get("content")
        .and_then(|c| c.get("parts"))
        .and_then(|p| p.as_array())
    else {
        return Ok((Vec::new(), Vec::new()));
    };

    let mut out = Vec::new();
    let mut thoughts = Vec::new();
    for block in parts {
        let is_thought = block
            .get("thought")
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        if let Some(text) = block.get("text").and_then(|v| v.as_str()) {
            if is_thought {
                thoughts.push(text.to_string());
            } else if let Some(stripped) = strip_thought_marker(text) {
                // Gemini sometimes prefaces an internal aside with ". "
                // or "? " — reclassify those as thoughts so they land in
                // the cyan-italic LogKind::Thinking lane instead of the
                // user-facing assistant-text lane.
                thoughts.push(stripped);
            } else {
                out.push(Part::Text(text.to_string()));
            }
            continue;
        }
        if let Some(call) = block.get("functionCall") {
            let name = call
                .get("name")
                .and_then(|v| v.as_str())
                .unwrap_or("")
                .to_string();
            let args = call.get("args").cloned().unwrap_or_else(|| json!({}));
            let arguments = serde_json::to_string(&args).unwrap_or_else(|_| "{}".to_string());
            let n = call_counter.fetch_add(1, Ordering::Relaxed);
            let id = format!("{}-{}", name, n);
            if let Some(sig) = block.get("thoughtSignature").and_then(|v| v.as_str()) {
                signatures.lock().unwrap().insert(id.clone(), sig.to_string());
            }
            out.push(Part::ToolCall {
                id,
                name,
                arguments,
            });
        }
        // Ignore `executableCode` / etc. — preserved in raw.
    }
    Ok((out, thoughts))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::llm::backend::{Message, ToolDef};
    use serde_json::json;

    fn sample_tools() -> Vec<ToolDef> {
        vec![ToolDef {
            name: "a".to_string(),
            description: "first".to_string(),
            parameters: json!({"type":"object","properties":{}}),
        }]
    }

    #[test]
    fn encode_messages_pulls_system_out_and_alternates_roles() {
        let msgs = vec![
            Message::system("sys"),
            Message::user_text("hi"),
            Message::assistant_text("hello"),
        ];
        let (sys, contents) = encode_messages(&msgs, &HashMap::new());
        let sys = sys.expect("system instruction present");
        assert_eq!(sys["parts"][0]["text"], "sys");
        assert_eq!(contents.len(), 2);
        assert_eq!(contents[0]["role"], "user");
        assert_eq!(contents[1]["role"], "model");
    }

    #[test]
    fn encode_messages_maps_tool_call_then_result_with_name_lookup() {
        let msgs = vec![
            Message::assistant_tool_call("c1", "a", "{\"x\":1}"),
            Message::tool_result("c1", "ok"),
        ];
        let (_sys, contents) = encode_messages(&msgs, &HashMap::new());
        assert_eq!(contents.len(), 2);
        assert_eq!(contents[0]["role"], "model");
        let call = &contents[0]["parts"][0]["functionCall"];
        assert_eq!(call["name"], "a");
        assert_eq!(call["args"]["x"], 1);
        assert_eq!(contents[1]["role"], "user");
        let resp = &contents[1]["parts"][0]["functionResponse"];
        assert_eq!(resp["name"], "a");
        assert_eq!(resp["response"]["result"], "ok");
    }

    #[test]
    fn encode_messages_collapses_adjacent_user_text_and_tool_result() {
        let msgs = vec![
            Message::user_text("first"),
            Message::tool_result("c1", "ok"),
        ];
        let (_sys, contents) = encode_messages(&msgs, &HashMap::new());
        assert_eq!(contents.len(), 1, "adjacent user-role messages collapse");
        let parts = contents[0]["parts"].as_array().unwrap();
        assert_eq!(parts.len(), 2);
    }

    #[test]
    fn encode_tools_wraps_in_function_declarations() {
        let tools = sample_tools();
        let out = encode_tools(&tools);
        assert_eq!(out.len(), 1);
        let decls = out[0]["functionDeclarations"].as_array().unwrap();
        assert_eq!(decls.len(), 1);
        assert_eq!(decls[0]["name"], "a");
    }

    #[test]
    fn encode_tools_strips_additional_properties_and_normalizes_nullable() {
        let tools = vec![ToolDef {
            name: "t".to_string(),
            description: "d".to_string(),
            parameters: json!({
                "type": "object",
                "additionalProperties": false,
                "properties": {
                    "name": {"type": ["string", "null"]},
                    "mode": {"type": ["string", "null"], "enum": ["add", "set", null]},
                    "nested": {
                        "type": "object",
                        "additionalProperties": false,
                        "properties": {
                            "x": {"type": ["number", "null"]}
                        }
                    }
                }
            }),
        }];
        let out = encode_tools(&tools);
        let params = &out[0]["functionDeclarations"][0]["parameters"];
        assert!(params.get("additionalProperties").is_none());
        assert!(params["properties"]["nested"].get("additionalProperties").is_none());
        assert_eq!(params["properties"]["name"]["type"], "string");
        assert_eq!(params["properties"]["name"]["nullable"], true);
        assert_eq!(params["properties"]["mode"]["type"], "string");
        assert_eq!(params["properties"]["mode"]["nullable"], true);
        let mode_enum = params["properties"]["mode"]["enum"].as_array().unwrap();
        assert_eq!(mode_enum, &vec![json!("add"), json!("set")]);
        assert_eq!(params["properties"]["nested"]["properties"]["x"]["type"], "number");
        assert_eq!(params["properties"]["nested"]["properties"]["x"]["nullable"], true);
    }

    #[test]
    fn parse_output_synthesizes_stable_ids() {
        let counter = AtomicU64::new(0);
        let payload = json!({
            "candidates": [{
                "content": {
                    "parts": [
                        {"text": "hello"},
                        {"functionCall": {"name": "a", "args": {"x": 1}}},
                        {"functionCall": {"name": "a", "args": {"x": 2}}}
                    ]
                }
            }]
        });
        let signatures = Mutex::new(HashMap::new());
        let (parts, thoughts) = parse_output(&payload, &counter, &signatures).unwrap();
        assert!(thoughts.is_empty());
        assert_eq!(parts.len(), 3);
        match &parts[1] {
            Part::ToolCall { id, name, .. } => {
                assert_eq!(name, "a");
                assert_eq!(id, "a-0");
            }
            _ => panic!(),
        }
        match &parts[2] {
            Part::ToolCall { id, .. } => assert_eq!(id, "a-1"),
            _ => panic!(),
        }
    }

    #[test]
    fn thought_signature_round_trips_from_response_into_next_request() {
        // Gemini 3 returns an opaque `thoughtSignature` on functionCall
        // parts; replaying the historical call without it triggers a
        // 400. parse_output must capture it; encode_messages must
        // re-attach it on the way out.
        let counter = AtomicU64::new(0);
        let signatures = Mutex::new(HashMap::new());
        let payload = json!({
            "candidates": [{
                "content": {
                    "parts": [
                        {
                            "functionCall": {"name": "a", "args": {"x": 1}},
                            "thoughtSignature": "sig-xyz"
                        }
                    ]
                }
            }]
        });
        let (parts, _thoughts) = parse_output(&payload, &counter, &signatures).unwrap();
        let id = match &parts[0] {
            Part::ToolCall { id, .. } => id.clone(),
            _ => panic!("expected ToolCall"),
        };
        assert_eq!(
            signatures.lock().unwrap().get(&id).map(String::as_str),
            Some("sig-xyz")
        );

        let snapshot = signatures.lock().unwrap().clone();
        let msgs = vec![Message::assistant_tool_call(id, "a", "{\"x\":1}")];
        let (_sys, contents) = encode_messages(&msgs, &snapshot);
        assert_eq!(contents[0]["parts"][0]["thoughtSignature"], "sig-xyz");
        assert_eq!(contents[0]["parts"][0]["functionCall"]["name"], "a");
    }

    #[test]
    fn parse_output_partitions_thought_text_from_assistant_text() {
        // `thought: true` parts are reasoning summaries (Gemini 3 only,
        // when `includeThoughts` is set). They go to the side channel
        // for bus display; only the non-thought text becomes Part::Text.
        let counter = AtomicU64::new(0);
        let signatures = Mutex::new(HashMap::new());
        let payload = json!({
            "candidates": [{
                "content": {
                    "parts": [
                        {"text": "Let me think about the runway choice.", "thought": true},
                        {"text": "Cleared for takeoff runway 30."}
                    ]
                }
            }]
        });
        let (parts, thoughts) = parse_output(&payload, &counter, &signatures).unwrap();
        assert_eq!(parts.len(), 1);
        assert_eq!(thoughts.len(), 1);
        assert_eq!(thoughts[0], "Let me think about the runway choice.");
        match &parts[0] {
            Part::Text(t) => assert_eq!(t, "Cleared for takeoff runway 30."),
            _ => panic!("expected Text"),
        }
    }

    #[test]
    fn parse_output_reroutes_dot_prefix_to_thoughts() {
        let counter = AtomicU64::new(0);
        let signatures = Mutex::new(HashMap::new());
        let payload = json!({
            "candidates": [{
                "content": {
                    "parts": [
                        {"text": ". muttering to myself about the wind"},
                        {"text": "? wonder if tower will let us extend"},
                        {"text": "Cleared touch-and-go runway 19."}
                    ]
                }
            }]
        });
        let (parts, thoughts) = parse_output(&payload, &counter, &signatures).unwrap();
        assert_eq!(parts.len(), 1);
        match &parts[0] {
            Part::Text(t) => assert_eq!(t, "Cleared touch-and-go runway 19."),
            _ => panic!("expected Text"),
        }
        assert_eq!(thoughts.len(), 2);
        assert_eq!(thoughts[0], "muttering to myself about the wind");
        assert_eq!(thoughts[1], "wonder if tower will let us extend");
    }

    #[test]
    fn strip_thought_marker_handles_only_known_prefixes() {
        assert_eq!(
            strip_thought_marker(". hello").as_deref(),
            Some("hello"),
        );
        assert_eq!(
            strip_thought_marker("? hello").as_deref(),
            Some("hello"),
        );
        // Marker on its own line (Gemini's actual emission shape).
        assert_eq!(
            strip_thought_marker(".\nThe aircraft is on the ground.").as_deref(),
            Some("The aircraft is on the ground."),
        );
        assert_eq!(
            strip_thought_marker("?\nWhat is the runway?").as_deref(),
            Some("What is the runway?"),
        );
        // Leading whitespace before the marker is tolerated.
        assert_eq!(
            strip_thought_marker("  . hello").as_deref(),
            Some("hello"),
        );
        // Period without trailing whitespace (e.g. "..." or "...thinking")
        // is NOT a marker.
        assert_eq!(strip_thought_marker("...thinking..."), None);
        // Plain assertions stay user-facing.
        assert_eq!(
            strip_thought_marker("Cleared for takeoff."),
            None,
        );
        // Empty body after stripping → not a thought.
        assert_eq!(strip_thought_marker(". "), None);
        assert_eq!(strip_thought_marker(".\n"), None);
    }

    #[test]
    fn parse_usage_reads_cached_content_tokens() {
        let payload = json!({
            "usageMetadata": {
                "promptTokenCount": 100,
                "candidatesTokenCount": 20,
                "cachedContentTokenCount": 75
            }
        });
        let u = parse_usage(&payload);
        assert_eq!(u.input_tokens, 100);
        assert_eq!(u.cached_tokens, 75);
        assert_eq!(u.output_tokens, 20);
    }
}
