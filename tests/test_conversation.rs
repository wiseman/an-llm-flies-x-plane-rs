//! Port of tests/test_conversation.py.
//!
//! The conversation loop uses the OpenAI Responses API via a
//! `ResponsesBackend` trait so we can swap in a scripted stub.

use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};

use anyhow::Result;
use crossbeam_channel::unbounded;
use parking_lot::Mutex as PLMutex;
use serde_json::{json, Value};

use xplane_pilot::bus::SimBus;
use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::llm::conversation::{
    run_conversation_loop, Conversation, IncomingMessage, MAX_INPUT_CHARS, SYSTEM_PROMPT,
};
use xplane_pilot::llm::responses_client::ResponsesBackend;
use xplane_pilot::llm::tools::ToolContext;

// ---------- stub ----------

struct StubClient {
    scripted: Mutex<Vec<Value>>,
    calls: Mutex<Vec<Value>>,
}

impl StubClient {
    fn new(scripted: Vec<Value>) -> Self {
        Self {
            scripted: Mutex::new(scripted),
            calls: Mutex::new(Vec::new()),
        }
    }
    fn n_calls(&self) -> usize {
        self.calls.lock().unwrap().len()
    }
    fn first_call(&self) -> Value {
        self.calls.lock().unwrap()[0].clone()
    }
}

impl ResponsesBackend for StubClient {
    fn create_response(
        &self,
        input_items: &[Value],
        tools: &[Value],
        timeout_secs: u64,
    ) -> Result<Value> {
        self.calls.lock().unwrap().push(json!({
            "input_items": input_items,
            "tools": tools,
            "timeout_secs": timeout_secs,
        }));
        let mut scripted = self.scripted.lock().unwrap();
        if scripted.is_empty() {
            return Ok(json!({"output": []}));
        }
        Ok(scripted.remove(0))
    }
}

fn make_ctx() -> ToolContext {
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(PLMutex::new(PilotCore::new(cfg.clone())));
    ToolContext::new(pilot, cfg)
}

fn fn_call(name: &str, args: Value, id: &str) -> Value {
    json!({
        "type": "function_call",
        "call_id": id,
        "name": name,
        "arguments": args.to_string(),
    })
}

fn text_msg(text: &str) -> Value {
    json!({
        "type": "message",
        "role": "assistant",
        "content": [{"type": "output_text", "text": text}],
    })
}

// ---------- conversation state ----------

#[test]
fn append_operator_message_adds_tagged_user_item() {
    let mut conv = Conversation::new("sys");
    conv.append_operator_message("take off");
    assert_eq!(conv.rotating_items.len(), 1);
    let item = &conv.rotating_items[0];
    assert_eq!(item["role"], "user");
    let text = item["content"][0]["text"].as_str().unwrap();
    assert!(text.contains("[OPERATOR]"));
}

#[test]
fn append_atc_message_adds_tagged_user_item() {
    let mut conv = Conversation::new("sys");
    conv.append_atc_message("squawk 1234");
    let t = conv.rotating_items[0]["content"][0]["text"].as_str().unwrap();
    assert!(t.contains("[ATC]"));
}

#[test]
fn append_heartbeat_message_adds_tagged_user_item() {
    let mut conv = Conversation::new("sys");
    conv.append_heartbeat_message("periodic check-in");
    let item = &conv.rotating_items[0];
    assert_eq!(item["role"], "user");
    let t = item["content"][0]["text"].as_str().unwrap();
    assert!(t.contains("[HEARTBEAT]"));
    assert!(t.contains("periodic check-in"));
}

#[test]
fn build_input_includes_pinned_profiles_summary_and_rotating() {
    let mut conv = Conversation::new("sys");
    conv.append_operator_message("hi");
    let items = conv.build_input("heading_hold, idle_vertical, idle_speed");
    assert_eq!(items[0]["role"], "system");
    assert_eq!(items[1]["role"], "user");
    assert_eq!(items[2]["role"], "system");
    let summary = items[2]["content"][0]["text"].as_str().unwrap();
    assert!(summary.contains("Active profiles"));
    assert!(summary.contains("heading_hold"));
}

#[test]
fn compaction_drops_oldest_full_turn() {
    let mut conv = Conversation::new("sys");
    for turn in 0..2 {
        conv.append_operator_message(&format!("msg{turn}"));
        conv.rotating_items.push(fn_call("noop", json!({}), &format!("c{turn}")));
        conv.append_function_call_output(&format!("c{turn}"), "ok").unwrap();
    }
    let user_count = |c: &Conversation| -> usize {
        c.rotating_items
            .iter()
            .filter(|i| i.get("role").and_then(|v| v.as_str()) == Some("user"))
            .count()
    };
    assert_eq!(user_count(&conv), 2);
    let dropped = conv.compact_if_needed(50);
    assert!(dropped > 0);
    assert_eq!(user_count(&conv), 1);
}

#[test]
fn compaction_leaves_partial_turn_alone() {
    let mut conv = Conversation::new("sys");
    conv.append_operator_message("only");
    conv.rotating_items.push(fn_call("noop", json!({}), "c0"));
    conv.append_function_call_output("c0", "ok").unwrap();
    assert_eq!(conv.compact_if_needed(10), 0);
    let _ = MAX_INPUT_CHARS;
}

// ---------- run loop ----------

#[test]
fn single_tool_call_then_text_ends_turn() {
    let ctx = make_ctx();
    let client = StubClient::new(vec![
        json!({"output": [fn_call("engage_heading_hold", json!({"heading_deg": 270.0}), "c1")]}),
        json!({"output": [text_msg("heading 270 engaged")]}),
    ]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("fly heading 270")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    let _ = std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || {
                run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, None);
            }
        });
        // wait for the stub to drain
        for _ in 0..50 {
            if client.n_calls() >= 2 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        stop.store(true, Ordering::Release);
        handle.join().unwrap();
    });

    assert_eq!(client.n_calls(), 2);
    assert!(ctx.pilot.lock().list_profile_names().contains(&"heading_hold".to_string()));
}

#[test]
fn sleep_tool_ends_turn_without_second_post() {
    let ctx = make_ctx();
    let client = StubClient::new(vec![json!({"output": [fn_call("sleep", json!({}), "c1")]})]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("standby")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, None)
        });
        for _ in 0..50 {
            if client.n_calls() >= 1 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        std::thread::sleep(std::time::Duration::from_millis(200));
        stop.store(true, Ordering::Release);
        handle.join().unwrap();
    });
    assert_eq!(client.n_calls(), 1);
}

#[test]
fn text_only_first_response_ends_turn_immediately() {
    let ctx = make_ctx();
    let client = StubClient::new(vec![json!({"output": [text_msg("acknowledged")]})]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::atc("maintain altitude")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, None)
        });
        for _ in 0..50 {
            if client.n_calls() >= 1 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        stop.store(true, Ordering::Release);
        handle.join().unwrap();
    });
    assert_eq!(client.n_calls(), 1);
}

#[test]
fn system_prompt_is_pinned_at_head_of_input() {
    let ctx = make_ctx();
    let client = StubClient::new(vec![json!({"output": [text_msg("ok")]})]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("status?")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, None)
        });
        for _ in 0..50 {
            if client.n_calls() >= 1 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        stop.store(true, Ordering::Release);
        handle.join().unwrap();
    });

    let first = client.first_call();
    let items = first["input_items"].as_array().unwrap();
    assert_eq!(items[0]["role"], "system");
    let text = items[0]["content"][0]["text"].as_str().unwrap();
    assert!(text.starts_with(&SYSTEM_PROMPT[..40]));
}

#[test]
fn tool_call_log_lands_on_bus_when_provided() {
    let ctx = make_ctx();
    let bus = SimBus::new(false);
    let client = StubClient::new(vec![
        json!({"output": [fn_call("engage_heading_hold", json!({"heading_deg": 90.0}), "c1")]}),
        json!({"output": [text_msg("heading 090 engaged")]}),
    ]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("fly heading 090")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, Some(&bus))
        });
        for _ in 0..50 {
            if client.n_calls() >= 2 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        stop.store(true, Ordering::Release);
        handle.join().unwrap();
    });

    let (_, logs, _) = bus.snapshot();
    let tool_log: Vec<&String> = logs.iter().filter(|l| l.contains("engage_heading_hold")).collect();
    let assistant: Vec<&String> = logs.iter().filter(|l| l.starts_with("[llm]")).collect();
    assert!(!tool_log.is_empty(), "expected tool log lines, got {:?}", logs);
    assert!(!assistant.is_empty(), "expected [llm] text, got {:?}", logs);
}

#[test]
fn token_usage_is_logged_after_each_call_when_usage_present() {
    let ctx = make_ctx();
    let bus = SimBus::new(false);
    // Two scripted responses; only one carries a `usage` block.
    let client = StubClient::new(vec![
        json!({
            "output": [fn_call("engage_heading_hold", json!({"heading_deg": 90.0}), "c1")],
            "usage": {
                "input_tokens": 1234,
                "output_tokens": 56,
                "input_tokens_details": {"cached_tokens": 900}
            }
        }),
        json!({"output": [text_msg("heading 090 engaged")]}),
    ]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("fly heading 090")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, Some(&bus))
        });
        for _ in 0..50 {
            if client.n_calls() >= 2 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        stop.store(true, Ordering::Release);
        handle.join().unwrap();
    });

    let (_, logs, _) = bus.snapshot();
    let token_lines: Vec<&String> = logs.iter().filter(|l| l.contains("tokens in=")).collect();
    // One `usage`-bearing response → exactly one token log line.
    assert_eq!(token_lines.len(), 1, "logs = {:?}", logs);
    let line = token_lines[0];
    assert!(line.contains("in=1234"), "{}", line);
    assert!(line.contains("cached=900"), "{}", line);
    assert!(line.contains("out=56"), "{}", line);
    // StubClient's default cache_snapshot() is zeros — session totals reflect that.
    assert!(line.contains("session in=0 out=0"), "{}", line);
}
