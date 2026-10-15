//! Claude Code pilot mode without the real CLI: the MCP server over real
//! HTTP, stream-json parsing, and an end-to-end eval run driven by a fake
//! `claude` script that calls our tools through the MCP server.

use std::os::unix::fs::PermissionsExt;
use std::sync::Arc;

use parking_lot::Mutex as PLMutex;
use serde_json::{json, Value};
use tempfile::TempDir;

mod common;

use common::{kwhp_resolved, make_eval_config};
use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::eval_runner::{run_eval_with_brain, OutcomeSource};
use xplane_pilot::llm::backend::LlmProvider;
use xplane_pilot::llm::brain::PilotBrain;
use xplane_pilot::llm::claude_code::{parse_stream_line, ClaudeCodeConfig, ClaudeEvent};
use xplane_pilot::llm::conversation::IncomingMessage;
use xplane_pilot::llm::mcp_server::{McpServer, END_TURN_NOTE};
use xplane_pilot::llm::tools::{tool_schemas, ToolContext, ToolCounters};

fn make_ctx() -> Arc<ToolContext> {
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(PLMutex::new(PilotCore::new(cfg.clone())));
    let mut ctx = ToolContext::new(pilot, cfg);
    ctx.tool_counters = Some(Arc::new(ToolCounters::default()));
    Arc::new(ctx)
}

fn post(url: &str, body: Value) -> (u16, Option<Value>) {
    match ureq::post(url).send_json(body) {
        Ok(resp) => {
            let status = resp.status();
            let text = resp.into_string().unwrap_or_default();
            (status, serde_json::from_str(&text).ok())
        }
        Err(ureq::Error::Status(code, _)) => (code, None),
        Err(e) => panic!("request failed: {e}"),
    }
}

#[test]
fn mcp_server_lists_and_calls_pilot_tools() {
    let ctx = make_ctx();
    let server = McpServer::start(ctx.clone(), None).unwrap();

    let (status, init) = post(
        &server.url,
        json!({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18"}}),
    );
    assert_eq!(status, 200);
    let init = init.unwrap();
    assert_eq!(init["result"]["protocolVersion"], "2025-06-18");
    assert!(init["result"]["capabilities"]["tools"].is_object());

    let (status, _) = post(&server.url, json!({"jsonrpc":"2.0","method":"notifications/initialized"}));
    assert_eq!(status, 202, "notifications get no body");

    let (_, list) = post(&server.url, json!({"jsonrpc":"2.0","id":2,"method":"tools/list"}));
    let tools = list.unwrap()["result"]["tools"].as_array().unwrap().clone();
    assert_eq!(tools.len(), tool_schemas(false).len());
    let status_tool = tools.iter().find(|t| t["name"] == "get_status").unwrap();
    assert_eq!(status_tool["inputSchema"]["type"], "object");

    let (_, call) = post(
        &server.url,
        json!({"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"get_status","arguments":{}}}),
    );
    let result = &call.unwrap()["result"];
    assert_eq!(result["isError"], false);
    assert!(result["content"][0]["text"].as_str().unwrap().contains("status"));

    let (_, call) = post(
        &server.url,
        json!({"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"no_such_tool","arguments":{}}}),
    );
    assert_eq!(call.unwrap()["result"]["isError"], true);

    // Turn-ending tools tell Claude Code to stop.
    let (_, call) = post(
        &server.url,
        json!({"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"sleep","arguments":{"seconds":null}}}),
    );
    let text = call.unwrap()["result"]["content"][0]["text"].as_str().unwrap().to_string();
    assert!(text.ends_with(END_TURN_NOTE), "{text}");

    let (_, unknown) = post(&server.url, json!({"jsonrpc":"2.0","id":6,"method":"server/discover"}));
    assert_eq!(unknown.unwrap()["error"]["code"], -32601);

    let counts = ctx.tool_counters.as_ref().unwrap().snapshot();
    assert_eq!(counts.total, 3, "every tools/call is counted: {:?}", counts.per_tool);
    assert_eq!(counts.failed, 1);

    let (status, _) = match ureq::get(&server.url).call() {
        Ok(r) => (r.status(), ()),
        Err(ureq::Error::Status(code, _)) => (code, ()),
        Err(e) => panic!("{e}"),
    };
    assert_eq!(status, 405, "no server-to-client stream");
}

#[test]
fn stream_json_lines_parse_into_events() {
    let init = parse_stream_line(
        r#"{"type":"system","subtype":"init","session_id":"abc","tools":[],"apiKeySource":"none"}"#,
    );
    assert!(matches!(&init[..], [ClaudeEvent::Init { session_id, api_key_source }]
        if session_id == "abc" && api_key_source.as_deref() == Some("none")));

    let assistant = parse_stream_line(
        r#"{"type":"assistant","message":{"content":[
            {"type":"thinking","thinking":""},
            {"type":"thinking","thinking":"plan the exit"},
            {"type":"text","text":"Roger."},
            {"type":"tool_use","id":"t1","name":"mcp__sim_pilot__sleep","input":{}}]}}"#,
    );
    assert_eq!(assistant.len(), 2, "empty thinking and tool_use dropped: {assistant:?}");
    assert!(matches!(&assistant[1], ClaudeEvent::Text(t) if t == "Roger."));

    let result = parse_stream_line(
        r#"{"type":"result","subtype":"success","is_error":false,"num_turns":3,"total_cost_usd":0.02,
            "usage":{"input_tokens":4,"cache_read_input_tokens":900,"cache_creation_input_tokens":50,"output_tokens":120}}"#,
    );
    match &result[..] {
        [ClaudeEvent::Result { usage, num_turns, cost_usd, is_error, .. }] => {
            assert_eq!(usage.input_tokens, 954);
            assert_eq!(usage.cached_tokens, 900);
            assert_eq!(usage.cache_creation_tokens, 50);
            assert_eq!(usage.output_tokens, 120);
            assert_eq!(*num_turns, 3);
            assert_eq!(*cost_usd, Some(0.02));
            assert!(!is_error);
        }
        other => panic!("unexpected {other:?}"),
    }

    let failed = parse_stream_line(r#"{"type":"result","subtype":"error_max_turns","is_error":true}"#);
    assert!(matches!(&failed[..], [ClaudeEvent::Result { is_error: true, .. }]));
    assert!(matches!(&parse_stream_line("not json")[..], [ClaudeEvent::Other]));
}

/// A stand-in for `claude -p`: reads stream-json user messages, calls
/// tools through the MCP server named in `--mcp-config`, and answers
/// with canned assistant/result lines. Records argv and messages.
/// Stands in for `claude -p` in stream-json mode. Like a real model that
/// won't end its turn, it keeps polling after `sleep` until the worker
/// sends an interrupt control request.
const FAKE_CLAUDE: &str = r#"#!/usr/bin/env python3
import json, os, select, sys, urllib.request
args = sys.argv[1:]
log = open(sys.argv[0] + ".log", "a")
log.write(json.dumps(args) + "\n"); log.flush()
log.write("APIKEY " + str("ANTHROPIC_API_KEY" in os.environ) + "\n"); log.flush()
url = json.loads(args[args.index("--mcp-config") + 1])["mcpServers"]["sim_pilot"]["url"]
def call(name, arguments, rid):
    body = json.dumps({"jsonrpc": "2.0", "id": rid, "method": "tools/call",
                       "params": {"name": name, "arguments": arguments}}).encode()
    req = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req).read())
def emit(v):
    print(json.dumps(v), flush=True)
buf = b""
def read_msg(timeout):
    global buf
    while b"\n" not in buf:
        if not select.select([0], [], [], timeout)[0]:
            return None
        chunk = os.read(0, 65536)
        if not chunk:
            sys.exit(0)
        buf += chunk
    line, buf = buf.split(b"\n", 1)
    return json.loads(line)
def result(subtype, turns, usage):
    emit({"type": "result", "subtype": subtype, "is_error": subtype != "success", "num_turns": turns,
          "total_cost_usd": 0.01, "usage": usage})
emit({"type": "system", "subtype": "init", "session_id": "fake-session"})
while True:
    msg = read_msg(None)
    if msg.get("type") == "control_request":
        emit({"type": "control_response", "response": {"subtype": "success", "request_id": msg["request_id"]}})
        continue
    content = msg["message"]["content"]
    log.write("MSG " + json.dumps(content) + "\n"); log.flush()
    call("get_status", {}, 1)
    emit({"type": "assistant", "message": {"content": [{"type": "text", "text": "Fake pilot, wilco."}]}})
    if "[OPERATOR]" in content:
        call("mission_complete", {"success": True, "summary": "fake claude ok"}, 2)
        result("success", 2, {"input_tokens": 5, "cache_read_input_tokens": 100,
                              "cache_creation_input_tokens": 10, "output_tokens": 7})
        continue
    call("sleep", {}, 3)
    while True:
        m = read_msg(0.1)
        if m and m.get("type") == "control_request":
            log.write("INTERRUPT\n"); log.flush()
            emit({"type": "control_response", "response": {"subtype": "success", "request_id": m["request_id"]}})
            result("error_during_execution", 1, {})
            break
        call("get_status", {}, 4)
"#;

#[test]
fn eval_run_through_claude_code_brain_completes_via_mcp_tools() {
    let tmp = TempDir::new().unwrap();
    let fake = tmp.path().join("fake-claude");
    std::fs::write(&fake, FAKE_CLAUDE).unwrap();
    std::fs::set_permissions(&fake, std::fs::Permissions::from_mode(0o755)).unwrap();

    // An API key in the environment must not reach the child: it would
    // take precedence over the subscription login.
    std::env::set_var("ANTHROPIC_API_KEY", "sk-test-should-not-leak");
    let mut brain_cfg = ClaudeCodeConfig::new("opus");
    brain_cfg.binary = fake.clone();
    let mut cfg = make_eval_config(&tmp, "eval-claude-code");
    cfg.llm_provider = LlmProvider::ClaudeCode;
    cfg.llm_model = "opus".to_string();
    // The ATC call's turn only ends when the worker interrupts it after
    // `sleep`; the operator message must still get through.
    cfg.startup_messages = vec![
        IncomingMessage::atc("Skyhawk 2SP, Whiteman tower, runway 12."),
        IncomingMessage::operator("Fly a lap and park."),
    ];

    let result = run_eval_with_brain(
        load_default_config_bundle(),
        cfg,
        kwhp_resolved(),
        PilotBrain::ClaudeCode(brain_cfg),
    )
    .expect("eval runs");

    assert_eq!(result.outcome.source, OutcomeSource::LlmReport);
    assert!(result.outcome.success);
    assert_eq!(result.outcome.summary, "fake claude ok");
    assert_eq!(result.turn_count, 3, "num_turns from the result lines");
    assert_eq!(result.total_cached_tokens, 100);
    assert_eq!(result.total_input_tokens, 115);
    let tools: Vec<&str> = result.tool_stats.per_tool.iter().map(|(n, _)| n.as_str()).collect();
    assert!(tools.contains(&"get_status") && tools.contains(&"mission_complete"), "{tools:?}");

    // The child was isolated from the user's Claude Code setup and given
    // only our tools.
    let fake_log = std::fs::read_to_string(fake.with_extension("log")).unwrap();
    let argv: Vec<String> = serde_json::from_str(fake_log.lines().next().unwrap()).unwrap();
    for flag in ["--strict-mcp-config", "--setting-sources", "--tools", "--system-prompt"] {
        assert!(argv.iter().any(|a| a == flag), "missing {flag}: {argv:?}");
    }
    let tools_idx = argv.iter().position(|a| a == "--tools").unwrap();
    assert_eq!(argv[tools_idx + 1], "", "built-in tools disabled");
    assert!(fake_log.contains("APIKEY False"), "API key leaked to the child: {fake_log}");
    let interrupt = fake_log.find("INTERRUPT").expect("worker interrupted the turn after sleep");
    let operator = fake_log
        .find(r#"MSG "[OPERATOR] Fly a lap and park.\n\nActive profiles:"#)
        .expect("operator message delivered");
    assert!(interrupt < operator, "{fake_log}");

    let log = std::fs::read_to_string(result.log_file.unwrap()).unwrap();
    assert!(log.contains("tool get_status ->"), "tool calls logged for the transcript");
    assert!(log.contains("Fake pilot, wilco."));
}

/// A controller that answers takeoff requests and stays silent otherwise.
const FAKE_ATC: &str = r#"#!/usr/bin/env python3
import json, sys
print(json.dumps({"type": "system", "subtype": "init", "session_id": "fake-atc"}), flush=True)
for line in sys.stdin:
    content = json.loads(line)["message"]["content"]
    reply = "Cessna 2SP, runway 12, cleared for takeoff." if "ready for departure" in content else "NONE"
    print(json.dumps({"type": "assistant", "message": {"content": [{"type": "text", "text": reply}]}}), flush=True)
    print(json.dumps({"type": "result", "subtype": "success", "is_error": False, "num_turns": 1,
                      "usage": {"input_tokens": 1, "output_tokens": 1}}), flush=True)
"#;

#[test]
fn sim_atc_answers_radio_calls_and_stays_silent_on_none() {
    use xplane_pilot::llm::conversation::IncomingSource;
    use xplane_pilot::llm::sim_atc::{RadioCall, SimAtc};
    let tmp = TempDir::new().unwrap();
    let fake = tmp.path().join("fake-atc");
    std::fs::write(&fake, FAKE_ATC).unwrap();
    std::fs::set_permissions(&fake, std::fs::Permissions::from_mode(0o755)).unwrap();
    std::env::set_var("CLAUDE_CODE_BIN", &fake);

    let (tx, rx) = crossbeam_channel::unbounded();
    let atc = SimAtc::start("haiku", "Airport: KWHP.", tx, None).unwrap();
    let call = |message: &str| RadioCall {
        freq_mhz: Some(135.0),
        message: message.to_string(),
        status_json: "{}".to_string(),
    };
    atc.hear(call("Whiteman Tower, Cessna 2SP, runway 12, ready for departure."));
    assert!(atc.composing());
    let reply = rx.recv_timeout(std::time::Duration::from_secs(10)).expect("controller answered");
    assert_eq!(reply.source, IncomingSource::Atc);
    assert_eq!(reply.text, "Cessna 2SP, runway 12, cleared for takeoff.");

    atc.hear(call("Runway 12, cleared for takeoff, Cessna 2SP."));
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    while atc.composing() && std::time::Instant::now() < deadline {
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    assert!(!atc.composing());
    assert!(rx.try_recv().is_err(), "a readback gets no answer");
    atc.shutdown();
}
