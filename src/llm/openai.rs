//! OpenAI Responses API backend.
//!
//! Uses `ureq` so we stay on blocking HTTP — the LLM worker runs on its
//! own thread, there is no async runtime.

use std::env;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{anyhow, Result};
use serde_json::{json, Value};

use crate::llm::backend::{
    send_with_body_on_error, write_transcript, CacheSnapshot, CacheStats, LlmBackend, LlmRequest,
    LlmResponse, LlmUsage, Message, Part, Role, ToolDef,
};

pub struct OpenAiBackend {
    pub model: String,
    pub api_key: Option<String>,
    pub api_base: String,
    cache_stats: Arc<CacheStats>,
    /// When set, each call dumps the request + response (or a `(pending)`
    /// placeholder) to this path, overwriting it. Lets an operator tail
    /// the current round without sifting the scrolling log. `None` = no
    /// dump.
    pub transcript_path: Option<PathBuf>,
}

impl OpenAiBackend {
    pub fn new(model: impl Into<String>) -> Self {
        Self {
            model: model.into(),
            api_key: None,
            api_base: "https://api.openai.com/v1".to_string(),
            cache_stats: Arc::new(CacheStats::default()),
            transcript_path: None,
        }
    }
}

impl LlmBackend for OpenAiBackend {
    fn create_response(&self, req: &LlmRequest<'_>) -> Result<LlmResponse> {
        let api_key = self
            .api_key
            .clone()
            .or_else(|| env::var("OPENAI_API_KEY").ok())
            .ok_or_else(|| anyhow!("OPENAI_API_KEY is required for the OpenAI backend"))?;
        let input_items = encode_input(req.messages);
        let tools_json = encode_tools(req.tools);
        let mut payload = json!({
            "model": self.model,
            "input": input_items,
            "tools": tools_json,
        });
        if let Some(effort) = req.reasoning_effort {
            payload["reasoning"] = json!({ "effort": effort.as_str() });
        }
        if let Some(path) = &self.transcript_path {
            write_transcript(path, &payload, None);
        }

        let url = format!("{}/responses", self.api_base.trim_end_matches('/'));
        let http = ureq::post(&url)
            .set("Authorization", &format!("Bearer {}", api_key))
            .set("Content-Type", "application/json")
            .timeout(std::time::Duration::from_secs(req.timeout_secs));
        let resp = send_with_body_on_error(http, &payload.to_string(), &url)?;
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

fn encode_input(messages: &[Message]) -> Vec<Value> {
    let mut out = Vec::with_capacity(messages.len());
    for msg in messages {
        // Responses API represents tool_calls and tool_results as
        // top-level items (siblings of `message`), not as content blocks
        // inside a message. Split them out here.
        let mut text_parts: Vec<&str> = Vec::new();
        for part in &msg.content {
            match part {
                Part::Text(t) => text_parts.push(t.as_str()),
                Part::ToolCall {
                    id,
                    name,
                    arguments,
                } => {
                    out.push(json!({
                        "type": "function_call",
                        "call_id": id,
                        "name": name,
                        "arguments": arguments,
                    }));
                }
                Part::ToolResult { id, content } => {
                    out.push(json!({
                        "type": "function_call_output",
                        "call_id": id,
                        "output": content,
                    }));
                }
            }
        }
        if !text_parts.is_empty() {
            let role = match msg.role {
                Role::System => "system",
                Role::User => "user",
                Role::Assistant => "assistant",
            };
            let content_type = match msg.role {
                Role::Assistant => "output_text",
                _ => "input_text",
            };
            let content: Vec<Value> = text_parts
                .into_iter()
                .map(|t| json!({"type": content_type, "text": t}))
                .collect();
            out.push(json!({
                "role": role,
                "content": content,
            }));
        }
    }
    out
}

fn encode_tools(tools: &[ToolDef]) -> Vec<Value> {
    tools
        .iter()
        .map(|t| {
            json!({
                "type": "function",
                "name": t.name,
                "description": t.description,
                "strict": true,
                "parameters": t.parameters,
            })
        })
        .collect()
}

// ---------- decoders ----------

fn parse_usage(response: &Value) -> LlmUsage {
    let Some(usage) = response.get("usage") else {
        return LlmUsage::default();
    };
    let input_tokens = usage.get("input_tokens").and_then(|v| v.as_u64()).unwrap_or(0);
    let cached_tokens = usage
        .get("input_tokens_details")
        .and_then(|d| d.get("cached_tokens"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let output_tokens = usage.get("output_tokens").and_then(|v| v.as_u64()).unwrap_or(0);
    LlmUsage {
        input_tokens,
        cached_tokens,
        output_tokens,
    }
}

fn parse_output(response: &Value) -> Result<Vec<Part>> {
    let Some(output) = response.get("output").and_then(|v| v.as_array()) else {
        return Err(anyhow!("response missing `output` array: {}", response));
    };
    let mut parts = Vec::new();
    for item in output {
        let ty = item.get("type").and_then(|v| v.as_str()).unwrap_or("");
        match ty {
            "message" => {
                if let Some(content) = item.get("content").and_then(|v| v.as_array()) {
                    for block in content {
                        if block.get("type").and_then(|v| v.as_str()) == Some("output_text") {
                            if let Some(text) = block.get("text").and_then(|v| v.as_str()) {
                                parts.push(Part::Text(text.to_string()));
                            }
                        }
                    }
                }
            }
            "function_call" => {
                let id = item
                    .get("call_id")
                    .or_else(|| item.get("id"))
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_string();
                let name = item
                    .get("name")
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_string();
                let arguments = item
                    .get("arguments")
                    .and_then(|v| v.as_str())
                    .unwrap_or("{}")
                    .to_string();
                parts.push(Part::ToolCall {
                    id,
                    name,
                    arguments,
                });
            }
            _ => {
                // Reasoning items, refusals, etc. — ignore for now; the
                // raw blob is preserved on `LlmResponse.raw` for anyone
                // who needs it.
            }
        }
    }
    Ok(parts)
}

/// Helper for the `extract_output_text` path used by a couple of
/// non-loop call sites. Preserves the same shape as the old
/// `responses_client::extract_output_text`.
pub fn extract_output_text(response_payload: &Value) -> String {
    let parts = parse_output(response_payload).unwrap_or_default();
    let mut out = String::new();
    for p in parts {
        if let Part::Text(t) = p {
            out.push_str(&t);
        }
    }
    out.trim().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::llm::backend::{Message, ReasoningEffort, ToolDef};
    use serde_json::json;

    fn sample_tools() -> Vec<ToolDef> {
        vec![ToolDef {
            name: "t".to_string(),
            description: "test tool".to_string(),
            parameters: json!({"type":"object","properties":{},"required":[],"additionalProperties":false}),
        }]
    }

    #[test]
    fn encode_input_splits_tool_calls_and_text() {
        let msgs = vec![
            Message::system("sys"),
            Message::user_text("hello"),
            Message::assistant_tool_call("c1", "t", "{\"a\":1}"),
            Message::tool_result("c1", "ok"),
        ];
        let input = encode_input(&msgs);
        // system message, user message, function_call, function_call_output
        assert_eq!(input.len(), 4);
        assert_eq!(input[0]["role"], "system");
        assert_eq!(input[0]["content"][0]["type"], "input_text");
        assert_eq!(input[1]["role"], "user");
        assert_eq!(input[2]["type"], "function_call");
        assert_eq!(input[2]["call_id"], "c1");
        assert_eq!(input[2]["arguments"], "{\"a\":1}");
        assert_eq!(input[3]["type"], "function_call_output");
        assert_eq!(input[3]["output"], "ok");
    }

    #[test]
    fn encode_tools_wraps_each_tooldef() {
        let tools = sample_tools();
        let out = encode_tools(&tools);
        assert_eq!(out[0]["type"], "function");
        assert_eq!(out[0]["name"], "t");
        assert_eq!(out[0]["strict"], true);
        assert!(out[0]["parameters"].is_object());
    }

    #[test]
    fn parse_output_pulls_text_and_function_call() {
        let payload = json!({
            "output": [
                {"type":"message","role":"assistant","content":[{"type":"output_text","text":"hi"}]},
                {"type":"function_call","call_id":"c1","name":"t","arguments":"{\"a\":1}"}
            ]
        });
        let parts = parse_output(&payload).unwrap();
        assert_eq!(parts.len(), 2);
        match &parts[0] {
            Part::Text(t) => assert_eq!(t, "hi"),
            _ => panic!(),
        }
        match &parts[1] {
            Part::ToolCall { id, name, arguments } => {
                assert_eq!(id, "c1");
                assert_eq!(name, "t");
                assert_eq!(arguments, "{\"a\":1}");
            }
            _ => panic!(),
        }
    }

    #[test]
    fn parse_usage_reads_cached_tokens() {
        let payload = json!({
            "usage": {
                "input_tokens": 100,
                "output_tokens": 20,
                "input_tokens_details": {"cached_tokens": 75}
            }
        });
        let u = parse_usage(&payload);
        assert_eq!(u.input_tokens, 100);
        assert_eq!(u.cached_tokens, 75);
        assert_eq!(u.output_tokens, 20);
    }
}
