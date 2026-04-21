//! Thin OpenAI Responses API client. Mirrors llm/responses_client.py.
//!
//! Uses `ureq` so we stay on blocking HTTP — the LLM worker runs on its own
//! thread; there is no async runtime needed.

use std::env;
use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, Context, Result};
use serde_json::{json, Value};

#[derive(Debug, Clone, Copy, Default)]
pub struct CacheSnapshot {
    pub total_input_tokens: u64,
    pub total_cached_tokens: u64,
    pub total_output_tokens: u64,
    pub total_requests: u64,
}

#[derive(Default)]
pub struct CacheStats {
    inner: Mutex<CacheSnapshot>,
}

impl CacheStats {
    pub fn record(&self, input_tokens: u64, cached_tokens: u64, output_tokens: u64) {
        let mut g = self.inner.lock().unwrap();
        g.total_input_tokens += input_tokens;
        g.total_cached_tokens += cached_tokens;
        g.total_output_tokens += output_tokens;
        g.total_requests += 1;
    }
    pub fn snapshot(&self) -> CacheSnapshot {
        *self.inner.lock().unwrap()
    }
    pub fn hit_rate(&self) -> Option<f64> {
        let s = self.snapshot();
        if s.total_input_tokens == 0 {
            None
        } else {
            Some(s.total_cached_tokens as f64 / s.total_input_tokens as f64)
        }
    }
}

pub trait ResponsesBackend: Send + Sync {
    /// Call `/responses` with the given input + tool schemas, return the raw payload.
    fn create_response(
        &self,
        input_items: &[Value],
        tools: &[Value],
        timeout_secs: u64,
    ) -> Result<Value>;

    /// Cumulative token counters across every call on this backend. Used by
    /// the conversation loop to log session totals after each API call.
    /// Default: empty (for test stubs that don't care).
    fn cache_snapshot(&self) -> CacheSnapshot {
        CacheSnapshot::default()
    }
}

pub struct ResponsesClient {
    pub model: String,
    pub api_key: Option<String>,
    pub api_base: String,
    pub reasoning_effort: Option<String>,
    pub cache_stats: Arc<CacheStats>,
    /// When set, each call dumps the request + response (or "(pending)"
    /// placeholder) to this path, overwriting it. Useful for
    /// live-tailing the pilot's current round without sifting through
    /// the scrolling log. `None` = no dump.
    pub transcript_path: Option<PathBuf>,
}

impl ResponsesClient {
    pub fn new(model: impl Into<String>) -> Self {
        Self {
            model: model.into(),
            api_key: None,
            api_base: "https://api.openai.com/v1".to_string(),
            reasoning_effort: None,
            cache_stats: Arc::new(CacheStats::default()),
            transcript_path: None,
        }
    }
}

impl ResponsesBackend for ResponsesClient {
    fn create_response(
        &self,
        input_items: &[Value],
        tools: &[Value],
        timeout_secs: u64,
    ) -> Result<Value> {
        let api_key = self
            .api_key
            .clone()
            .or_else(|| env::var("OPENAI_API_KEY").ok())
            .ok_or_else(|| anyhow!("OPENAI_API_KEY is required for the LLM worker"))?;
        let mut payload = json!({
            "model": self.model,
            "input": input_items,
            "tools": tools,
        });
        if let Some(effort) = &self.reasoning_effort {
            payload["reasoning"] = json!({ "effort": effort });
        }
        // Dump the outgoing request with a placeholder response so a
        // live tail shows "request in flight" while the HTTP call is
        // open. The post-response write below overwrites this with the
        // full round once the API returns.
        if let Some(path) = &self.transcript_path {
            write_transcript(path, &payload, None);
        }

        let url = format!("{}/responses", self.api_base.trim_end_matches('/'));
        let resp = ureq::post(&url)
            .set("Authorization", &format!("Bearer {}", api_key))
            .set("Content-Type", "application/json")
            .timeout(std::time::Duration::from_secs(timeout_secs))
            .send_string(&payload.to_string())
            .with_context(|| format!("posting to {}", url))?;
        let result: Value = resp.into_json()?;
        if let Some(usage) = result.get("usage") {
            let input_tokens = usage.get("input_tokens").and_then(|v| v.as_u64()).unwrap_or(0);
            let cached_tokens = usage
                .get("input_tokens_details")
                .and_then(|d| d.get("cached_tokens"))
                .and_then(|v| v.as_u64())
                .unwrap_or(0);
            let output_tokens = usage.get("output_tokens").and_then(|v| v.as_u64()).unwrap_or(0);
            self.cache_stats.record(input_tokens, cached_tokens, output_tokens);
        }
        if let Some(path) = &self.transcript_path {
            write_transcript(path, &payload, Some(&result));
        }
        Ok(result)
    }

    fn cache_snapshot(&self) -> CacheSnapshot {
        self.cache_stats.snapshot()
    }
}

/// Best-effort dump of the most recent Responses API round. Writes
/// the request body pretty-printed, then the response (or `(pending)`
/// when called before the HTTP call completes). Silently swallows
/// errors — a broken transcript dump shouldn't take down the pilot.
fn write_transcript(path: &PathBuf, request: &Value, response: Option<&Value>) {
    if let Some(parent) = path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    let request_str = serde_json::to_string_pretty(request)
        .unwrap_or_else(|e| format!("(request serialize failed: {e})"));
    let response_str = match response {
        Some(r) => serde_json::to_string_pretty(r)
            .unwrap_or_else(|e| format!("(response serialize failed: {e})")),
        None => "(pending — HTTP request in flight)".to_string(),
    };
    let body = format!(
        "=== REQUEST ===\n{request_str}\n\n=== RESPONSE ===\n{response_str}\n"
    );
    let _ = fs::write(path, body);
}

pub fn extract_output_text(response_payload: &Value) -> String {
    let Some(output) = response_payload.get("output").and_then(|v| v.as_array()) else {
        return String::new();
    };
    let mut out = String::new();
    for item in output {
        if item.get("type").and_then(|v| v.as_str()) == Some("message") {
            if let Some(content) = item.get("content").and_then(|v| v.as_array()) {
                for part in content {
                    if part.get("type").and_then(|v| v.as_str()) == Some("output_text") {
                        if let Some(t) = part.get("text").and_then(|v| v.as_str()) {
                            out.push_str(t);
                        }
                    }
                }
            }
        }
    }
    out.trim().to_string()
}
