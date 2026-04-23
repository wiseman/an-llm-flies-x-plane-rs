//! Anthropic Messages API backend.
//!
//! `POST https://api.anthropic.com/v1/messages` (API version 2023-06-01).
//! Blocking HTTP via `ureq`. Explicit `cache_control: {type:"ephemeral"}`
//! breakpoints are placed on (a) the final system block and (b) the last
//! tool declaration so a turn that reuses the same system-prompt +
//! tool-schema prefix hits the cache.

use std::env;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{anyhow, Context, Result};
use serde_json::{json, Value};

use crate::llm::backend::{
    write_transcript, CacheSnapshot, CacheStats, LlmBackend, LlmRequest, LlmResponse, LlmUsage,
    Message, Part, ReasoningEffort, Role, ToolDef,
};

const API_URL: &str = "https://api.anthropic.com/v1/messages";
const API_VERSION: &str = "2023-06-01";
/// Cap on assistant output length. Bumped a bit above typical pilot
/// responses so an operator asking for a long explanation isn't cut off.
const DEFAULT_MAX_TOKENS: u32 = 4096;

pub struct AnthropicBackend {
    pub model: String,
    pub api_key: Option<String>,
    cache_stats: Arc<CacheStats>,
    pub transcript_path: Option<PathBuf>,
    pub max_tokens: u32,
}

impl AnthropicBackend {
    pub fn new(model: impl Into<String>) -> Self {
        Self {
            model: model.into(),
            api_key: None,
            cache_stats: Arc::new(CacheStats::default()),
            transcript_path: None,
            max_tokens: DEFAULT_MAX_TOKENS,
        }
    }
}

impl LlmBackend for AnthropicBackend {
    fn create_response(&self, req: &LlmRequest<'_>) -> Result<LlmResponse> {
        let api_key = self
            .api_key
            .clone()
            .or_else(|| env::var("ANTHROPIC_API_KEY").ok())
            .ok_or_else(|| anyhow!("ANTHROPIC_API_KEY is required for the Anthropic backend"))?;

        let (system_blocks, messages) = encode_messages(req.messages);
        let tools_json = encode_tools(req.tools);

        let mut payload = json!({
            "model": self.model,
            "max_tokens": self.max_tokens,
            "system": system_blocks,
            "messages": messages,
            "tools": tools_json,
            "tool_choice": {"type": "auto"},
        });
        if let Some(effort) = req.reasoning_effort {
            payload["thinking"] = thinking_block(effort);
        }

        if let Some(path) = &self.transcript_path {
            write_transcript(path, &payload, None);
        }

        let resp = ureq::post(API_URL)
            .set("x-api-key", &api_key)
            .set("anthropic-version", API_VERSION)
            .set("content-type", "application/json")
            .timeout(std::time::Duration::from_secs(req.timeout_secs))
            .send_string(&payload.to_string())
            .with_context(|| format!("posting to {}", API_URL))?;
        let raw: Value = resp.into_json()?;
        let usage = parse_usage(&raw);
        self.cache_stats
            .record(usage.input_tokens, usage.cached_tokens, usage.output_tokens);
        if let Some(path) = &self.transcript_path {
            write_transcript(path, &payload, Some(&raw));
        }
        let output = parse_output(&raw)?;
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

/// Split the neutral `Message` list into the Anthropic `system` blocks
/// plus the user/assistant `messages` array. Adjacent same-role
/// non-system messages are collapsed into one message carrying multiple
/// content blocks. The final system block gets an ephemeral
/// `cache_control` marker so prompt caching kicks in when the stable
/// prefix is unchanged between turns.
fn encode_messages(messages: &[Message]) -> (Vec<Value>, Vec<Value>) {
    let mut system_blocks: Vec<Value> = Vec::new();
    let mut out: Vec<Value> = Vec::new();

    for msg in messages {
        if msg.role == Role::System {
            for part in &msg.content {
                if let Part::Text(t) = part {
                    system_blocks.push(json!({"type": "text", "text": t}));
                }
            }
            continue;
        }

        let target_role = match msg.role {
            Role::Assistant => "assistant",
            _ => "user", // user + tool_result blocks both live on user-role messages
        };

        let mut blocks: Vec<Value> = Vec::new();
        for part in &msg.content {
            match part {
                Part::Text(t) => blocks.push(json!({"type": "text", "text": t})),
                Part::ToolCall {
                    id,
                    name,
                    arguments,
                } => {
                    // Anthropic expects `input` as a JSON object, not a
                    // string. Fall back to an empty object if the
                    // stored arguments don't parse.
                    let input: Value =
                        serde_json::from_str(arguments).unwrap_or_else(|_| json!({}));
                    blocks.push(json!({
                        "type": "tool_use",
                        "id": id,
                        "name": name,
                        "input": input,
                    }));
                }
                Part::ToolResult { id, content } => blocks.push(json!({
                    "type": "tool_result",
                    "tool_use_id": id,
                    "content": content,
                })),
            }
        }

        // Collapse into the previous message if the role matches — keeps
        // assistant tool_use+text and user tool_result+text pairs in a
        // single message, which Anthropic prefers.
        if let Some(last) = out.last_mut() {
            if last.get("role").and_then(|v| v.as_str()) == Some(target_role) {
                if let Some(existing) = last.get_mut("content").and_then(|v| v.as_array_mut()) {
                    existing.extend(blocks);
                    continue;
                }
            }
        }
        out.push(json!({
            "role": target_role,
            "content": blocks,
        }));
    }

    // Cache breakpoint on the final system block. The stable prefix is
    // system-prompt + earliest rotating history; the trailing profiles
    // summary is appended as its own system block at turn time and is
    // the one that changes per turn — so marking the block *before* it
    // would be ideal, but the loop appends summary last, so every block
    // up to and including the marker is stable within a turn.
    if let Some(last) = system_blocks.last_mut() {
        if let Some(obj) = last.as_object_mut() {
            obj.insert(
                "cache_control".to_string(),
                json!({"type": "ephemeral"}),
            );
        }
    }

    (system_blocks, out)
}

fn encode_tools(tools: &[ToolDef]) -> Vec<Value> {
    let mut out: Vec<Value> = tools
        .iter()
        .map(|t| {
            json!({
                "name": t.name,
                "description": t.description,
                "input_schema": t.parameters,
            })
        })
        .collect();
    // Cache breakpoint on the last tool definition — the tools array is
    // identical across turns, so this makes the whole tools blob
    // cacheable.
    if let Some(last) = out.last_mut() {
        if let Some(obj) = last.as_object_mut() {
            obj.insert(
                "cache_control".to_string(),
                json!({"type": "ephemeral"}),
            );
        }
    }
    out
}

fn thinking_block(effort: ReasoningEffort) -> Value {
    json!({
        "type": "enabled",
        "budget_tokens": effort.token_budget(),
    })
}

// ---------- decoders ----------

fn parse_usage(response: &Value) -> LlmUsage {
    let Some(usage) = response.get("usage") else {
        return LlmUsage::default();
    };
    let input_tokens = usage.get("input_tokens").and_then(|v| v.as_u64()).unwrap_or(0);
    let output_tokens = usage.get("output_tokens").and_then(|v| v.as_u64()).unwrap_or(0);
    // Anthropic reports cache reads and cache writes as separate
    // counters. For parity with OpenAI's `cached_tokens` we meter the
    // read side — that's the portion that costs less this turn because
    // it was already populated.
    let cached_tokens = usage
        .get("cache_read_input_tokens")
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    LlmUsage {
        input_tokens,
        cached_tokens,
        output_tokens,
    }
}

fn parse_output(response: &Value) -> Result<Vec<Part>> {
    let Some(content) = response.get("content").and_then(|v| v.as_array()) else {
        return Err(anyhow!(
            "Anthropic response missing `content` array: {}",
            response
        ));
    };
    let mut parts = Vec::new();
    for block in content {
        let ty = block.get("type").and_then(|v| v.as_str()).unwrap_or("");
        match ty {
            "text" => {
                if let Some(text) = block.get("text").and_then(|v| v.as_str()) {
                    parts.push(Part::Text(text.to_string()));
                }
            }
            "tool_use" => {
                let id = block.get("id").and_then(|v| v.as_str()).unwrap_or("").to_string();
                let name = block
                    .get("name")
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_string();
                let input = block.get("input").cloned().unwrap_or_else(|| json!({}));
                let arguments = serde_json::to_string(&input).unwrap_or_else(|_| "{}".to_string());
                parts.push(Part::ToolCall {
                    id,
                    name,
                    arguments,
                });
            }
            _ => {
                // thinking / redacted_thinking / etc. — ignore in the
                // upstream stream; `LlmResponse.raw` preserves them.
            }
        }
    }
    Ok(parts)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::llm::backend::{Message, ToolDef};
    use serde_json::json;

    fn sample_tools() -> Vec<ToolDef> {
        vec![
            ToolDef {
                name: "a".to_string(),
                description: "first".to_string(),
                parameters: json!({"type":"object","properties":{},"required":[]}),
            },
            ToolDef {
                name: "b".to_string(),
                description: "second".to_string(),
                parameters: json!({"type":"object","properties":{},"required":[]}),
            },
        ]
    }

    #[test]
    fn encode_messages_puts_system_in_its_own_slot_with_cache_control() {
        let msgs = vec![
            Message::system("sys base"),
            Message::user_text("hi"),
        ];
        let (system, msgs_out) = encode_messages(&msgs);
        assert_eq!(system.len(), 1);
        assert_eq!(system[0]["type"], "text");
        assert_eq!(system[0]["text"], "sys base");
        assert_eq!(system[0]["cache_control"]["type"], "ephemeral");
        assert_eq!(msgs_out.len(), 1);
        assert_eq!(msgs_out[0]["role"], "user");
    }

    #[test]
    fn encode_messages_collapses_adjacent_same_role_messages() {
        let msgs = vec![
            Message::user_text("a"),
            Message::user_text("b"),
        ];
        let (_sys, out) = encode_messages(&msgs);
        assert_eq!(out.len(), 1, "two user messages should collapse into one");
        let content = out[0]["content"].as_array().unwrap();
        assert_eq!(content.len(), 2);
    }

    #[test]
    fn encode_messages_maps_tool_call_and_tool_result() {
        let msgs = vec![
            Message::assistant_tool_call("c1", "a", "{\"x\":1}"),
            Message::tool_result("c1", "ok"),
        ];
        let (_sys, out) = encode_messages(&msgs);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0]["role"], "assistant");
        assert_eq!(out[0]["content"][0]["type"], "tool_use");
        assert_eq!(out[0]["content"][0]["id"], "c1");
        assert_eq!(out[0]["content"][0]["input"]["x"], 1);
        assert_eq!(out[1]["role"], "user");
        assert_eq!(out[1]["content"][0]["type"], "tool_result");
        assert_eq!(out[1]["content"][0]["tool_use_id"], "c1");
        assert_eq!(out[1]["content"][0]["content"], "ok");
    }

    #[test]
    fn encode_tools_puts_cache_control_on_last_tool() {
        let tools = sample_tools();
        let out = encode_tools(&tools);
        assert_eq!(out.len(), 2);
        assert!(out[0].get("cache_control").is_none());
        assert_eq!(out[1]["cache_control"]["type"], "ephemeral");
    }

    #[test]
    fn parse_output_pulls_text_and_tool_use() {
        let payload = json!({
            "content": [
                {"type":"text","text":"hello"},
                {"type":"tool_use","id":"c1","name":"a","input":{"x":1}}
            ]
        });
        let parts = parse_output(&payload).unwrap();
        assert_eq!(parts.len(), 2);
        match &parts[0] {
            Part::Text(t) => assert_eq!(t, "hello"),
            _ => panic!(),
        }
        match &parts[1] {
            Part::ToolCall { id, name, arguments } => {
                assert_eq!(id, "c1");
                assert_eq!(name, "a");
                let v: Value = serde_json::from_str(arguments).unwrap();
                assert_eq!(v["x"], 1);
            }
            _ => panic!(),
        }
    }

    #[test]
    fn parse_usage_reads_cache_read_as_cached_tokens() {
        let payload = json!({
            "usage": {
                "input_tokens": 100,
                "output_tokens": 20,
                "cache_read_input_tokens": 75,
                "cache_creation_input_tokens": 10
            }
        });
        let u = parse_usage(&payload);
        assert_eq!(u.input_tokens, 100);
        assert_eq!(u.cached_tokens, 75);
        assert_eq!(u.output_tokens, 20);
    }

    #[test]
    fn thinking_block_uses_budget_tokens() {
        let v = thinking_block(ReasoningEffort::Medium);
        assert_eq!(v["type"], "enabled");
        assert_eq!(v["budget_tokens"], 8192);
    }
}
