//! The conversation loop uses a provider-neutral `LlmBackend` trait so we
//! can swap in a scripted stub.

use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};

use anyhow::Result;
use crossbeam_channel::unbounded;
use parking_lot::Mutex as PLMutex;
use serde_json::{json, Value};

use xplane_pilot::bus::{LogKind, SimBus};
use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::llm::backend::{LlmBackend, LlmRequest, LlmResponse, LlmUsage, Message, Part};
use xplane_pilot::llm::conversation::{
    run_conversation_loop, Conversation, IncomingMessage, PilotMode, SYSTEM_PROMPT,
};
use xplane_pilot::llm::tools::ToolContext;

// ---------- stub ----------

/// Scripted backend: returns pre-built `LlmResponse` values in order
/// and records each captured request for inspection.
struct StubClient {
    scripted: Mutex<Vec<LlmResponse>>,
    calls: Mutex<Vec<CapturedCall>>,
}

struct CapturedCall {
    messages: Vec<Message>,
    tool_count: usize,
    timeout_secs: u64,
}

impl StubClient {
    fn new(scripted: Vec<LlmResponse>) -> Self {
        Self {
            scripted: Mutex::new(scripted),
            calls: Mutex::new(Vec::new()),
        }
    }
    fn n_calls(&self) -> usize {
        self.calls.lock().unwrap().len()
    }
    fn first_call_messages(&self) -> Vec<Message> {
        self.calls.lock().unwrap()[0].messages.clone()
    }
}

impl LlmBackend for StubClient {
    fn create_response(&self, req: &LlmRequest<'_>) -> Result<LlmResponse> {
        self.calls.lock().unwrap().push(CapturedCall {
            messages: req.messages.to_vec(),
            tool_count: req.tools.len(),
            timeout_secs: req.timeout_secs,
        });
        let mut scripted = self.scripted.lock().unwrap();
        if scripted.is_empty() {
            return Ok(LlmResponse {
                output: Vec::new(),
                usage: LlmUsage::default(),
                raw: Value::Null,
            });
        }
        Ok(scripted.remove(0))
    }
}

fn make_ctx() -> ToolContext {
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(PLMutex::new(PilotCore::new(cfg.clone())));
    ToolContext::new(pilot, cfg)
}

fn text_response(text: &str) -> LlmResponse {
    LlmResponse {
        output: vec![Part::Text(text.to_string())],
        usage: LlmUsage::default(),
        raw: Value::Null,
    }
}

fn tool_call_response(name: &str, args: Value, id: &str) -> LlmResponse {
    LlmResponse {
        output: vec![Part::ToolCall {
            id: id.to_string(),
            name: name.to_string(),
            arguments: args.to_string(),
        }],
        usage: LlmUsage::default(),
        raw: Value::Null,
    }
}

fn user_text(msg: &Message) -> Option<&str> {
    if let Part::Text(t) = msg.content.first()? {
        Some(t.as_str())
    } else {
        None
    }
}

// ---------- conversation state ----------

#[test]
fn append_operator_message_adds_tagged_user_item() {
    let mut conv = Conversation::new("sys");
    conv.append_operator_message("take off");
    assert_eq!(conv.rotating.len(), 1);
    let text = conv.rotating_text(0).unwrap();
    assert!(text.contains("[OPERATOR]"));
}

#[test]
fn append_atc_message_adds_tagged_user_item() {
    let mut conv = Conversation::new("sys");
    conv.append_atc_message("squawk 1234");
    let t = conv.rotating_text(0).unwrap();
    assert!(t.contains("[ATC]"));
}

#[test]
fn append_heartbeat_message_adds_tagged_user_item() {
    let mut conv = Conversation::new("sys");
    conv.append_heartbeat_message("periodic check-in");
    let t = conv.rotating_text(0).unwrap();
    assert!(t.contains("[HEARTBEAT]"));
    assert!(t.contains("periodic check-in"));
}

#[test]
fn build_input_includes_pinned_operator_and_trailing_profiles_user() {
    let mut conv = Conversation::new("sys");
    conv.append_operator_message("hi");
    let items = conv.build_input("heading_hold, idle_vertical, idle_speed");
    // pinned system + operator user + trailing user profile summary
    assert_eq!(items.len(), 3);
    assert_eq!(items[0].role, xplane_pilot::llm::backend::Role::System);
    assert_eq!(items[1].role, xplane_pilot::llm::backend::Role::User);
    assert_eq!(items[2].role, xplane_pilot::llm::backend::Role::User);
    let summary = user_text(&items[2]).unwrap();
    assert!(summary.contains("Active profiles"));
    assert!(summary.contains("heading_hold"));
}

#[test]
fn cache_anchor_idx_points_at_last_rotating_message() {
    let mut conv = Conversation::new("sys");
    assert_eq!(conv.cache_anchor_idx(), None, "no anchor when rotating is empty");
    conv.append_operator_message("hi");
    assert_eq!(conv.cache_anchor_idx(), Some(1));
    conv.append_heartbeat_message("hb");
    assert_eq!(conv.cache_anchor_idx(), Some(2));
}

// ---------- pilot mode ----------

#[test]
fn pilot_mode_parse_accepts_known_modes_case_insensitive() {
    assert_eq!(PilotMode::parse("normal"), Some(PilotMode::Normal));
    assert_eq!(PilotMode::parse("Realistic"), Some(PilotMode::Realistic));
    assert_eq!(PilotMode::parse("  REALISTIC  "), Some(PilotMode::Realistic));
    assert_eq!(PilotMode::parse("strict"), None);
    assert_eq!(PilotMode::parse(""), None);
}

#[test]
fn pilot_mode_selects_distinct_prompts() {
    let normal = PilotMode::Normal.system_prompt();
    let realistic = PilotMode::Realistic.system_prompt();
    assert!(realistic.len() > normal.len());
    assert!(realistic.starts_with(normal));
    assert!(realistic.contains("Realistic-pilot mode"));
}

#[test]
fn set_system_prompt_rewrites_pinned_head() {
    let mut conv = Conversation::new("first prompt");
    conv.append_operator_message("msg before swap");
    conv.set_system_prompt("second prompt");
    assert_eq!(conv.system_prompt(), "second prompt");
    assert_eq!(conv.pinned_text(0), Some("second prompt"));
    assert_eq!(conv.rotating.len(), 1, "history preserved across swap");
}

#[test]
fn runtime_mode_switch_injects_overlay_without_touching_system_prompt() {
    // A runtime `/mode realistic` must leave the pinned system message
    // alone (so the cached prefix stays stable) and push the overlay
    // into rotating user history tagged `[MODE_SWITCH]`.
    let mut conv = Conversation::new(SYSTEM_PROMPT);
    assert!(conv.rotating.is_empty());

    conv.append_mode_switch_message(PilotMode::Realistic.runtime_overlay());
    assert_eq!(
        conv.pinned_text(0),
        Some(SYSTEM_PROMPT),
        "system prompt must remain the base prompt at runtime"
    );
    assert_eq!(conv.rotating.len(), 1);
    let injected = conv.rotating_text(0).unwrap();
    assert!(injected.starts_with("[MODE_SWITCH] "));
    assert!(injected.contains("Realistic-pilot mode"));
}

#[test]
fn runtime_mode_switch_to_normal_injects_cancel_notice() {
    let mut conv = Conversation::new(SYSTEM_PROMPT);
    conv.append_mode_switch_message(PilotMode::Normal.runtime_overlay());
    let injected = conv.rotating_text(0).unwrap();
    assert!(injected.starts_with("[MODE_SWITCH] "));
    assert!(injected.contains("Return to normal pilot mode"));
}

#[test]
fn mode_switch_message_does_not_trigger_api_call() {
    let ctx = make_ctx();
    let client = StubClient::new(vec![text_response("unused")]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::mode_switch(PilotMode::Realistic)).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, PilotMode::Normal, None, None)
        });
        std::thread::sleep(std::time::Duration::from_millis(200));
        stop.store(true, Ordering::Release);
        handle.join().unwrap();
    });

    assert_eq!(client.n_calls(), 0, "mode switch must not trigger an API call");
}

// ---------- run loop ----------

#[test]
fn single_tool_call_then_text_ends_turn() {
    let ctx = make_ctx();
    let client = StubClient::new(vec![
        tool_call_response("engage_heading_hold", json!({"heading_deg": 270.0}), "c1"),
        text_response("heading 270 engaged"),
    ]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("fly heading 270")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || {
                run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, PilotMode::Normal, None, None);
            }
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

    assert_eq!(client.n_calls(), 2);
    assert!(ctx.pilot.lock().list_profile_names().contains(&"heading_hold".to_string()));
}

#[test]
fn assistant_text_emitted_even_when_response_also_has_tool_call() {
    let ctx = make_ctx();
    let client = StubClient::new(vec![LlmResponse {
        output: vec![
            Part::Text("I'm good and ready.".to_string()),
            Part::ToolCall {
                id: "c1".to_string(),
                name: "sleep".to_string(),
                arguments: "{}".to_string(),
            },
        ],
        usage: LlmUsage::default(),
        raw: Value::Null,
    }]);
    let bus = SimBus::new(false);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("hi how are you")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || {
                run_conversation_loop(
                    &client,
                    &ctx,
                    &rx,
                    stop,
                    60,
                    120,
                    PilotMode::Normal,
                    None,
                    Some(&bus),
                );
            }
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
    let entries = bus.log_entries();
    let saw_text = entries
        .iter()
        .any(|e| e.kind == LogKind::Llm && e.text == "I'm good and ready.");
    assert!(
        saw_text,
        "expected assistant text to reach the bus; got entries: {:?}",
        entries.iter().map(|e| (e.kind, e.text.clone())).collect::<Vec<_>>()
    );
}

#[test]
fn sleep_tool_ends_turn_without_second_post() {
    let ctx = make_ctx();
    let client = StubClient::new(vec![tool_call_response("sleep", json!({}), "c1")]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("standby")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, PilotMode::Normal, None, None)
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
    let client = StubClient::new(vec![text_response("acknowledged")]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::atc("maintain altitude")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, PilotMode::Normal, None, None)
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
    let client = StubClient::new(vec![text_response("ok")]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("status?")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, PilotMode::Normal, None, None)
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

    let messages = client.first_call_messages();
    assert_eq!(messages[0].role, xplane_pilot::llm::backend::Role::System);
    let text = match &messages[0].content[0] {
        Part::Text(t) => t.clone(),
        _ => panic!(),
    };
    assert!(text.starts_with(&SYSTEM_PROMPT[..40]));
}

#[test]
fn tool_call_log_lands_on_bus_when_provided() {
    let ctx = make_ctx();
    let bus = SimBus::new(false);
    let client = StubClient::new(vec![
        tool_call_response("engage_heading_hold", json!({"heading_deg": 90.0}), "c1"),
        text_response("heading 090 engaged"),
    ]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("fly heading 090")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, PilotMode::Normal, None, Some(&bus))
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
    // Two scripted responses; only one carries non-zero `usage`.
    let client = StubClient::new(vec![
        LlmResponse {
            output: vec![Part::ToolCall {
                id: "c1".to_string(),
                name: "engage_heading_hold".to_string(),
                arguments: json!({"heading_deg": 90.0}).to_string(),
            }],
            usage: LlmUsage {
                input_tokens: 1234,
                output_tokens: 56,
                cached_tokens: 900,
            },
            raw: Value::Null,
        },
        text_response("heading 090 engaged"),
    ]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("fly heading 090")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));

    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, PilotMode::Normal, None, Some(&bus))
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
    // One response with non-zero usage → exactly one token log line.
    assert_eq!(token_lines.len(), 1, "logs = {:?}", logs);
    let line = token_lines[0];
    assert!(line.contains("in=1234"), "{}", line);
    assert!(line.contains("cached=900"), "{}", line);
    assert!(line.contains("out=56"), "{}", line);
    // StubClient's default cache_snapshot() is zeros — session totals reflect that.
    assert!(line.contains("session in=0 out=0"), "{}", line);
}

// Keep `CapturedCall.tool_count` and `timeout_secs` addressable so the
// stub can be extended without field-drift warnings.
#[test]
fn stub_captures_tools_and_timeout() {
    let ctx = make_ctx();
    let client = StubClient::new(vec![text_response("ok")]);
    let (tx, rx) = unbounded::<IncomingMessage>();
    tx.send(IncomingMessage::operator("hi")).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    std::thread::scope(|s| {
        let handle = s.spawn({
            let stop = stop.clone();
            || run_conversation_loop(&client, &ctx, &rx, stop, 60, 120, PilotMode::Normal, None, None)
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
    let captured = client.calls.lock().unwrap();
    assert!(captured[0].tool_count > 0);
    assert!(captured[0].timeout_secs > 0);
}
