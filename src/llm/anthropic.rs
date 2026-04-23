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

        let (system_blocks, messages) = encode_messages(req.messages, req.cache_anchor_idx);
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
/// content blocks.
///
/// Two cache breakpoints are placed per request:
/// 1. On the *first* system block (the pinned system prompt). Anthropic
///    caches everything up to and including the marked block in
///    `tools → system → messages` order, so this covers all tools + the
///    stable system prompt.
/// 2. On the last content block of the message at `cache_anchor_idx`
///    when provided — the rotating conversation history. This extends
///    the cached prefix through the end of the prior turn so only the
///    new user message plus the trailing (volatile) profile summary are
///    reprocessed each turn.
fn encode_messages(
    messages: &[Message],
    cache_anchor_idx: Option<usize>,
) -> (Vec<Value>, Vec<Value>) {
    let mut system_blocks: Vec<Value> = Vec::new();
    let mut out: Vec<Value> = Vec::new();
    // (out_message_index, content_block_index) of the last block that
    // came from the source message at cache_anchor_idx.
    let mut anchor_location: Option<(usize, usize)> = None;

    for (src_idx, msg) in messages.iter().enumerate() {
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
            _ => "user",
        };

        let mut blocks: Vec<Value> = Vec::new();
        for part in &msg.content {
            match part {
                Part::Text(t) => blocks.push(json!({"type": "text", "text": t})),
                Part::ToolCall { id, name, arguments } => {
                    // Anthropic expects `input` as a JSON object, not a
                    // string. Fall back to an empty object if the stored
                    // arguments don't parse.
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

        let block_count = blocks.len();
        let collapse = out
            .last()
            .and_then(|v| v.get("role").and_then(|r| r.as_str()))
            == Some(target_role);
        let (out_idx, first_new_block_idx) = if collapse {
            let last_idx = out.len() - 1;
            let existing = out[last_idx]
                .get_mut("content")
                .and_then(|v| v.as_array_mut())
                .expect("collapsed message always has a content array");
            let offset = existing.len();
            existing.extend(blocks);
            (last_idx, offset)
        } else {
            out.push(json!({"role": target_role, "content": blocks}));
            (out.len() - 1, 0)
        };

        if cache_anchor_idx == Some(src_idx) && block_count > 0 {
            anchor_location = Some((out_idx, first_new_block_idx + block_count - 1));
        }
    }

    // Breakpoint 1: first (pinned, stable) system block → caches tools +
    // system. Later system blocks (if any) inherit the cache coverage
    // because they're part of the prefix from that point forward.
    if let Some(first) = system_blocks.first_mut() {
        mark_ephemeral(first);
    }

    // Breakpoint 2: the content block corresponding to cache_anchor_idx
    // → caches everything through the last stable rotating message. The
    // trailing profile-summary user message is appended *after* the
    // anchor in `messages`, so it stays outside the cached prefix and
    // can change per turn without invalidating the cache.
    if let Some((msg_i, blk_i)) = anchor_location {
        if let Some(block) = out
            .get_mut(msg_i)
            .and_then(|m| m.get_mut("content"))
            .and_then(|c| c.as_array_mut())
            .and_then(|a| a.get_mut(blk_i))
        {
            mark_ephemeral(block);
        }
    }

    (system_blocks, out)
}

/// Stamp a JSON object block with an ephemeral `cache_control` marker.
/// No-op on non-object Values.
fn mark_ephemeral(block: &mut Value) {
    if let Some(obj) = block.as_object_mut() {
        obj.insert("cache_control".to_string(), json!({"type": "ephemeral"}));
    }
}

fn encode_tools(tools: &[ToolDef]) -> Vec<Value> {
    // No tool-array cache_control: the system-block breakpoint in
    // `encode_messages` already caches `tools + system` because
    // Anthropic's prefix-caching order is tools → system → messages.
    tools
        .iter()
        .map(|t| {
            json!({
                "name": t.name,
                "description": t.description,
                "input_schema": t.parameters,
            })
        })
        .collect()
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
    fn encode_messages_puts_first_system_block_cache_control_even_with_later_volatile_block() {
        // The stable pinned prompt is the first system block; later
        // system text (were it ever present) would be uncached. The
        // profile-summary-as-user-message path in production avoids a
        // second system block altogether, but we still want the
        // breakpoint on the first block specifically.
        let msgs = vec![
            Message::system("pinned stable prompt"),
            Message::system("later volatile text"),
            Message::user_text("hi"),
        ];
        let (system, _msgs_out) = encode_messages(&msgs, None);
        assert_eq!(system.len(), 2);
        assert_eq!(system[0]["cache_control"]["type"], "ephemeral");
        assert!(system[1].get("cache_control").is_none());
    }

    #[test]
    fn encode_messages_collapses_adjacent_same_role_messages() {
        let msgs = vec![
            Message::user_text("a"),
            Message::user_text("b"),
        ];
        let (_sys, out) = encode_messages(&msgs, None);
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
        let (_sys, out) = encode_messages(&msgs, None);
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
    fn cache_anchor_marks_the_last_block_from_that_source_message() {
        // src indices: 0=system (ignored for anchor), 1=user "hi",
        // 2=assistant tool_call, 3=user tool_result, 4=user "profile".
        // Anchor = 3 → the tool_result block should carry cache_control,
        // and the trailing profile summary (index 4) should not.
        let msgs = vec![
            Message::system("sys"),
            Message::user_text("hi"),
            Message::assistant_tool_call("c1", "a", "{\"x\":1}"),
            Message::tool_result("c1", "ok"),
            Message::user_text("Active profiles: heading_hold"),
        ];
        let (_sys, out) = encode_messages(&msgs, Some(3));
        // user("hi") + assistant(tool_use) + user(tool_result, profile) — tool_result
        // and profile collapse into one user message.
        assert_eq!(out.len(), 3);
        let user_msg = &out[2];
        assert_eq!(user_msg["role"], "user");
        let content = user_msg["content"].as_array().unwrap();
        assert_eq!(content.len(), 2);
        // The tool_result block has cache_control; the profile text does not.
        assert_eq!(content[0]["type"], "tool_result");
        assert_eq!(content[0]["cache_control"]["type"], "ephemeral");
        assert_eq!(content[1]["type"], "text");
        assert!(content[1].get("cache_control").is_none());
    }

    #[test]
    fn encode_tools_has_no_cache_control() {
        // Tools are cached transitively via the system-block breakpoint;
        // placing another marker on the tools array would waste one of
        // the four available breakpoints.
        let tools = sample_tools();
        let out = encode_tools(&tools);
        assert_eq!(out.len(), 2);
        assert!(out[0].get("cache_control").is_none());
        assert!(out[1].get("cache_control").is_none());
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
