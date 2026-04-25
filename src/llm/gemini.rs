//! Google Gemini `generateContent` backend.
//!
//! `POST https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent?key=API_KEY`.
//! Blocking HTTP via `ureq`. Prompt caching on the 2.5-series models is
//! implicit — identical prefixes are auto-cached for the same key with
//! no explicit `cachedContents` roundtrip needed. Tool calls come back
//! without an `id`, so we synthesize stable ids so the next turn's
//! `functionResponse` can reference the right call by name.

use std::env;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

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
}

impl GeminiBackend {
    pub fn new(model: impl Into<String>) -> Self {
        Self {
            model: model.into(),
            api_key: None,
            cache_stats: Arc::new(CacheStats::default()),
            transcript_path: None,
            call_counter: Arc::new(AtomicU64::new(0)),
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

        let (system_instruction, contents) = encode_messages(req.messages);
        let tools_json = encode_tools(req.tools);

        let mut payload = json!({
            "contents": contents,
            "tools": tools_json,
            "toolConfig": {"functionCallingConfig": {"mode": "AUTO"}},
        });
        if let Some(sys) = system_instruction {
            payload["systemInstruction"] = sys;
        }
        if let Some(effort) = req.reasoning_effort {
            payload["generationConfig"] = json!({
                "thinkingConfig": {"thinkingBudget": effort.token_budget()}
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
        let output = parse_output(&raw, &self.call_counter)?;
        Ok(LlmResponse {
            output,
            usage,
            raw,
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
fn encode_messages(messages: &[Message]) -> (Option<Value>, Vec<Value>) {
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
                Part::ToolCall { name, arguments, .. } => {
                    let args: Value =
                        serde_json::from_str(arguments).unwrap_or_else(|_| json!({}));
                    parts.push(json!({
                        "functionCall": {"name": name, "args": args}
                    }));
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
            json!({
                "name": t.name,
                "description": t.description,
                "parameters": t.parameters,
            })
        })
        .collect();
    vec![json!({"functionDeclarations": declarations})]
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

fn parse_output(response: &Value, call_counter: &AtomicU64) -> Result<Vec<Part>> {
    let Some(candidates) = response.get("candidates").and_then(|v| v.as_array()) else {
        return Err(anyhow!(
            "Gemini response missing `candidates` array: {}",
            response
        ));
    };
    let Some(first) = candidates.first() else {
        return Ok(Vec::new());
    };
    let Some(parts) = first
        .get("content")
        .and_then(|c| c.get("parts"))
        .and_then(|p| p.as_array())
    else {
        return Ok(Vec::new());
    };

    let mut out = Vec::new();
    for block in parts {
        if let Some(text) = block.get("text").and_then(|v| v.as_str()) {
            out.push(Part::Text(text.to_string()));
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
            out.push(Part::ToolCall {
                id,
                name,
                arguments,
            });
        }
        // Ignore `thought` / `executableCode` / etc. — preserved in raw.
    }
    Ok(out)
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
        let (sys, contents) = encode_messages(&msgs);
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
        let (_sys, contents) = encode_messages(&msgs);
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
        let (_sys, contents) = encode_messages(&msgs);
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
        let parts = parse_output(&payload, &counter).unwrap();
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
