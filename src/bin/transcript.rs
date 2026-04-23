//! Renders a sim_pilot-*.log file as a styled HTML transcript.

use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;

#[derive(Parser)]
#[command(about = "Render a sim_pilot log as a styled HTML transcript")]
struct Cli {
    /// Path to the .log file.
    input: PathBuf,

    /// Output HTML path. Defaults to input with `.html` extension.
    #[arg(short, long)]
    output: Option<PathBuf>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let text = fs::read_to_string(&cli.input)
        .with_context(|| format!("reading {}", cli.input.display()))?;
    let raw = parse_log(&text);
    let (meta, events) = categorize(&raw, &cli.input);
    let html = render_html(&meta, &events);

    let out = cli.output.unwrap_or_else(|| cli.input.with_extension("html"));
    fs::write(&out, html).with_context(|| format!("writing {}", out.display()))?;
    eprintln!("wrote {}", out.display());
    Ok(())
}

#[derive(Debug)]
struct RawEntry {
    ts: Option<String>,
    kind: String,
    source: Option<String>,
    body: String,
}

fn parse_log(text: &str) -> Vec<RawEntry> {
    let mut entries: Vec<RawEntry> = Vec::new();
    for line in text.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let has_ts = line.len() >= 20 && line.as_bytes().get(10) == Some(&b'T');
        let (ts, rest) = if has_ts {
            (Some(line[..19].to_string()), line[19..].trim_start())
        } else {
            (None, line.trim_start())
        };
        let (kind, after_kind) = if let Some(stripped) = rest.strip_prefix('[') {
            if let Some(end) = stripped.find(']') {
                (stripped[..end].to_string(), stripped[end + 1..].trim_start())
            } else {
                (String::new(), rest)
            }
        } else {
            (String::new(), rest)
        };
        // `[log]` and `[radio]` both carry a nested `[source]` tag that
        // names the subsystem (llm-worker, heartbeat, atc, COM1 …).
        let nests_source = kind == "log" || kind == "radio";
        let (source, body) = if nests_source {
            if let Some(stripped) = after_kind.strip_prefix('[') {
                if let Some(end) = stripped.find(']') {
                    (
                        Some(stripped[..end].to_string()),
                        stripped[end + 1..].trim_start().to_string(),
                    )
                } else {
                    (None, after_kind.to_string())
                }
            } else {
                (None, after_kind.to_string())
            }
        } else {
            (None, after_kind.to_string())
        };
        // Lines without a timestamp are continuations of the prior entry —
        // multi-row sql_query output, multi-line checklists, route plans.
        if !has_ts {
            if let Some(prev) = entries.last_mut() {
                if !prev.body.is_empty() {
                    prev.body.push('\n');
                }
                prev.body.push_str(&body);
                continue;
            }
        }
        entries.push(RawEntry { ts, kind, source, body });
    }
    entries
}

#[derive(Debug, Default)]
struct Meta {
    filename: String,
    date: Option<String>,       // YYYY-MM-DD
    start_time: Option<String>, // HH:MM:SS
    end_time: Option<String>,
    duration: Option<String>,   // "04m 33s"
    pilot_mode: Option<String>,
    llm_provider: Option<String>,
    llm_model: Option<String>,
}

#[derive(Debug)]
struct TimedEvent {
    ts: String,
    kind: EventKind,
}

#[derive(Debug)]
enum EventKind {
    LlmText { text: String },
    ToolCall { summary: String, result: Option<String> },
    PhaseChange { from: String, to: String, extras: Option<String> },
    Profiles { text: String },
    Safety { text: String },
    Input { channel: String, text: String },
    ModeSwitch { text: String },
}

fn categorize(raw: &[RawEntry], input: &std::path::Path) -> (Meta, Vec<TimedEvent>) {
    let mut meta = Meta::default();
    meta.filename = input
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("transcript")
        .to_string();

    let mut events: Vec<TimedEvent> = Vec::new();
    let mut last_ts = String::new();
    let mut first_ts: Option<String> = None;

    let push = |events: &mut Vec<TimedEvent>, ts: &str, kind: EventKind| {
        events.push(TimedEvent { ts: ts.to_string(), kind });
    };

    for r in raw {
        if let Some(ts) = &r.ts {
            last_ts.clear();
            last_ts.push_str(ts);
            if first_ts.is_none() {
                first_ts = Some(ts.clone());
            }
        }
        if r.kind == "status" {
            continue;
        }
        let src = r.source.as_deref().unwrap_or("");
        let body = &r.body;

        if r.kind == "radio" {
            // Radio entries land in the file as `[radio] [<src>] <body>`.
            // Route ATC side to the existing atc channel; everything else
            // (pilot broadcasts, tune confirmations) goes to a dedicated
            // radio channel so the two sides of the conversation are
            // visually distinct.
            if src == "atc" {
                push(
                    &mut events,
                    &last_ts,
                    EventKind::Input { channel: "atc".into(), text: body.clone() },
                );
            } else {
                let text = if body.is_empty() {
                    src.to_string()
                } else if src.is_empty() {
                    body.clone()
                } else {
                    format!("[{}] {}", src, body)
                };
                if !text.is_empty() {
                    push(
                        &mut events,
                        &last_ts,
                        EventKind::Input { channel: "radio".into(), text },
                    );
                }
            }
            continue;
        }
        if r.kind != "log" || r.body.is_empty() {
            continue;
        }

        match src {
            "" => {
                // Un-sourced [log] lines are startup environment noise
                // (log_file=..., bridge connected, track_csv=..., etc.).
                // Mine them for metadata, then drop.
                if let Some(rest) = body.strip_prefix("pilot llm: ") {
                    for part in rest.split_whitespace() {
                        if let Some(v) = part.strip_prefix("provider=") {
                            meta.llm_provider = Some(v.to_string());
                        } else if let Some(v) = part.strip_prefix("model=") {
                            meta.llm_model = Some(v.to_string());
                        }
                    }
                }
            }
            "llm" => push(&mut events, &last_ts, EventKind::LlmText { text: body.clone() }),
            "llm-worker" => {
                if let Some(rest) = body.strip_prefix("tool ") {
                    if let Some(arrow) = rest.find(" -> ") {
                        let name = rest[..arrow].trim().to_string();
                        let result = rest[arrow + 4..].to_string();
                        let summary = summarize_tool(&name, &result);
                        let res = if result_is_interesting(&name, &result)
                            && !summary.contains(&result)
                        {
                            Some(result)
                        } else {
                            None
                        };
                        push(
                            &mut events,
                            &last_ts,
                            EventKind::ToolCall { summary, result: res },
                        );
                    }
                }
                // tokens in=... out=... accounting — drop.
            }
            "heartbeat" => {
                let summary = body.split(" | status=").next().unwrap_or(body).trim();
                if summary == "periodic check-in" {
                    continue;
                }
                let kind = parse_phase_change(summary).unwrap_or(EventKind::Profiles {
                    text: summary.to_string(),
                });
                push(&mut events, &last_ts, kind);
            }
            "safety" => push(&mut events, &last_ts, EventKind::Safety { text: body.clone() }),
            "operator" => push(
                &mut events,
                &last_ts,
                EventKind::Input { channel: "operator".into(), text: body.clone() },
            ),
            "atc" => push(
                &mut events,
                &last_ts,
                EventKind::Input { channel: "atc".into(), text: body.clone() },
            ),
            "mode" => {
                if let Some(m) = body.strip_prefix("pilot mode: ") {
                    if meta.pilot_mode.is_none() {
                        meta.pilot_mode = Some(m.to_string());
                        continue;
                    }
                }
                push(&mut events, &last_ts, EventKind::ModeSwitch { text: body.clone() });
            }
            _ => {}
        }
    }

    // Fill timing meta.
    if let Some(ref ts) = first_ts {
        let (date, time) = split_ts(ts);
        meta.date = Some(date);
        meta.start_time = Some(time);
    }
    if !last_ts.is_empty() {
        let (_, time) = split_ts(&last_ts);
        meta.end_time = Some(time);
    }
    if let (Some(f), Some(l)) = (first_ts.as_ref(), (!last_ts.is_empty()).then_some(&last_ts)) {
        meta.duration = duration_between(f, l);
    }

    (meta, events)
}

fn split_ts(ts: &str) -> (String, String) {
    if let Some((d, t)) = ts.split_once('T') {
        (d.to_string(), t.to_string())
    } else {
        (String::new(), ts.to_string())
    }
}

fn duration_between(a: &str, b: &str) -> Option<String> {
    let ta = chrono::NaiveDateTime::parse_from_str(a, "%Y-%m-%dT%H:%M:%S").ok()?;
    let tb = chrono::NaiveDateTime::parse_from_str(b, "%Y-%m-%dT%H:%M:%S").ok()?;
    let total = tb.signed_duration_since(ta).num_seconds().max(0);
    let m = total / 60;
    let s = total % 60;
    if m > 0 {
        Some(format!("{}m {:02}s", m, s))
    } else {
        Some(format!("{}s", s))
    }
}

fn parse_phase_change(summary: &str) -> Option<EventKind> {
    let rest = summary.strip_prefix("phase changed: ")?;
    let (from_to, extras) = match rest.split_once("; ") {
        Some((ft, ex)) => (ft, Some(ex.to_string())),
        None => (rest, None),
    };
    let (from, to) = from_to.split_once(" -> ")?;
    Some(EventKind::PhaseChange {
        from: from.trim().to_string(),
        to: to.trim().to_string(),
        extras,
    })
}

fn summarize_tool(name: &str, result: &str) -> String {
    let first_line = result.lines().next().unwrap_or(result).trim_end();
    if first_line.starts_with("error") {
        return format!("{} → {}", name, truncate_chars(first_line, 160));
    }
    if name == "get_status" {
        return name.to_string();
    }
    if name == "sleep" {
        return format!("{} ({})", name, truncate_chars(first_line, 80));
    }
    if name.starts_with("engage_") || name.starts_with("disengage_") || name.starts_with("set_") {
        return format!("{} — {}", name, truncate_chars(first_line, 140));
    }
    if name == "sql_query" {
        let rows = result.lines().count().saturating_sub(1);
        return format!("{} ({} row{})", name, rows, if rows == 1 { "" } else { "s" });
    }
    name.to_string()
}

fn result_is_interesting(name: &str, result: &str) -> bool {
    let trimmed = result.trim_start();
    if trimmed.starts_with("error") {
        return true;
    }
    if name == "get_status" {
        return false;
    }
    if name.starts_with("engage_")
        || name.starts_with("disengage_")
        || name.starts_with("set_")
        || name == "sleep"
    {
        return false;
    }
    result.lines().count() > 1
}

fn truncate_chars(s: &str, n: usize) -> String {
    let mut out: String = s.chars().take(n).collect();
    if out.chars().count() < s.chars().count() {
        out.push('…');
    }
    out
}

fn short_ts(ts: &str) -> String {
    ts.split_once('T').map(|(_, t)| t.to_string()).unwrap_or_else(|| ts.to_string())
}

fn escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

fn render_html(meta: &Meta, events: &[TimedEvent]) -> String {
    let mut html = String::new();
    html.push_str("<!doctype html>\n<html lang=\"en\"><head>\n");
    html.push_str("<meta charset=\"utf-8\">\n");
    html.push_str("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n");
    html.push_str(&format!("<title>{} — cockpit transcript</title>\n", escape(&meta.filename)));
    html.push_str(FONT_LINKS);
    html.push_str(STYLE);
    html.push_str("</head><body>\n");

    // ───────── compact header ─────────
    html.push_str("<header class=\"masthead\">\n");
    html.push_str("<div class=\"masthead-inner\">\n");
    html.push_str(&format!(
        "<h1 class=\"title\">Cockpit <em>transcript</em></h1>\n\
         <div class=\"file mono\">{}</div>\n",
        escape(&meta.filename),
    ));

    html.push_str("<dl class=\"meta\">\n");
    if let Some(d) = &meta.date {
        html.push_str(&meta_item("Date", d));
    }
    if let (Some(s), Some(e)) = (&meta.start_time, &meta.end_time) {
        html.push_str(&meta_item("Window", &format!("{} → {}", s, e)));
    }
    if let Some(dur) = &meta.duration {
        html.push_str(&meta_item("Duration", dur));
    }
    html.push_str(&meta_item("Events", &events.len().to_string()));
    if let Some(m) = &meta.pilot_mode {
        html.push_str(&meta_item("Mode", m));
    }
    if meta.llm_provider.is_some() || meta.llm_model.is_some() {
        let provider = meta.llm_provider.as_deref().unwrap_or("");
        let model = meta.llm_model.as_deref().unwrap_or("");
        let value = match (provider.is_empty(), model.is_empty()) {
            (false, false) => format!("{} · {}", provider, model),
            (true, false) => model.to_string(),
            (false, true) => provider.to_string(),
            (true, true) => String::new(),
        };
        html.push_str(&meta_item("Pilot LLM", &value));
    }
    html.push_str("</dl>\n");
    html.push_str("</div></header>\n");

    // ───────── timeline ─────────
    html.push_str("<main class=\"transcript\">\n");
    for e in events {
        html.push_str(&render_event(e));
    }
    html.push_str("</main>\n");

    html.push_str("</body></html>\n");
    html
}

fn meta_item(key: &str, val: &str) -> String {
    format!(
        "<div class=\"meta-item\"><dt>{}</dt><dd class=\"mono\">{}</dd></div>\n",
        escape(key),
        escape(val),
    )
}

fn render_event(e: &TimedEvent) -> String {
    let ts = &e.ts;
    match &e.kind {
        EventKind::LlmText { text } => row("pilot", ts, "PILOT", &escape(text)),
        EventKind::ToolCall { summary, result } => {
            let mut body = format!("<span class=\"tool-sum\">{}</span>", escape(summary));
            if let Some(res) = result {
                body.push_str(&format!(
                    "<details class=\"tool-detail\"><summary>result</summary><pre>{}</pre></details>",
                    escape(res)
                ));
            }
            row("tool", ts, "TOOL", &body)
        }
        EventKind::PhaseChange { from, to, extras } => {
            let mut body = format!(
                "<span class=\"phase-change\"><em>{}</em> → <em>{}</em></span>",
                escape(from),
                escape(to),
            );
            if let Some(ex) = extras {
                body.push_str(&format!("<span class=\"phase-extras\">{}</span>", escape(ex)));
            }
            row("phase", ts, "PHASE", &body)
        }
        EventKind::Profiles { text } => row("profiles", ts, "STATE", &escape(text)),
        EventKind::Safety { text } => row("safety", ts, "⚠", &escape(text)),
        EventKind::Input { channel, text } => {
            let label = match channel.as_str() {
                "atc" => "ATC",
                "operator" => "OPS",
                "radio" => "RADIO",
                _ => channel.as_str(),
            };
            row(&format!("input input-{}", channel), ts, label, &escape(text))
        }
        EventKind::ModeSwitch { text } => row("mode", ts, "MODE", &escape(text)),
    }
}

fn row(classes: &str, ts: &str, label: &str, body_html: &str) -> String {
    format!(
        "<div class=\"evt {cls}\"><time class=\"ts mono\">{t}</time><span class=\"tag mono\">{lbl}</span><div class=\"body\">{body}</div></div>\n",
        cls = classes,
        t = escape(&short_ts(ts)),
        lbl = escape(label),
        body = body_html,
    )
}

const FONT_LINKS: &str = r#"<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Instrument+Serif:ital@0;1&family=Source+Serif+4:ital,opsz,wght@0,8..60,400..700;1,8..60,400..700&family=JetBrains+Mono:wght@400;500;600&display=swap" rel="stylesheet">
"#;

const STYLE: &str = r#"<style>
:root {
    --paper: #fbfaf5;
    --paper-deep: #f1ede1;
    --ink: #17243d;
    --ink-soft: #3a4764;
    --muted: #7a7364;
    --tool-faint: #9a9280;
    --rule: #d9d2be;
    --rule-soft: #e5dfcc;
    --signal: #b0311d;
    --signal-bg: #f7e4dd;
    --amber: #9e6c12;
    --royal: #1e4079;
    --mauve: #6b3560;
    --sage: #4a6b3e;
    --phase-bg: #f4ead0;
    --phase-rule: #d8c98f;
    --profile-bg: #e9efd6;
    --profile-rule: #c1ce9c;
}

* { box-sizing: border-box; }
html, body { margin: 0; padding: 0; }
em, i { font-style: normal; }
body {
    background: var(--paper);
    color: var(--ink);
    font-family: 'Source Serif 4', 'Iowan Old Style', Georgia, serif;
    font-size: 16px;
    line-height: 1.4;
    -webkit-font-smoothing: antialiased;
    text-rendering: optimizeLegibility;
}
.mono {
    font-family: 'JetBrains Mono', ui-monospace, 'SF Mono', Menlo, Consolas, monospace;
    font-variant-numeric: tabular-nums;
}

/* ───────── compact header ───────── */
.masthead {
    border-bottom: 1px solid var(--rule);
    background: var(--paper);
}
.masthead-inner {
    max-width: 960px;
    margin: 0 auto;
    padding: 18px 24px 16px;
}
.title {
    margin: 0;
    font-family: 'Instrument Serif', 'Source Serif 4', serif;
    font-weight: 400;
    font-size: 26px;
    line-height: 1.1;
    letter-spacing: -0.01em;
    color: var(--ink);
    display: inline-block;
    margin-right: 10px;
}
.title em { color: var(--royal); }
.file {
    display: inline-block;
    font-size: 12px;
    color: var(--muted);
    vertical-align: middle;
}

.meta {
    display: flex;
    flex-wrap: wrap;
    gap: 0 20px;
    margin: 10px 0 0;
    padding: 0;
}
.meta-item {
    display: flex;
    align-items: baseline;
    gap: 6px;
    font-size: 12px;
}
.meta-item dt {
    margin: 0;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 0.08em;
    font-family: 'JetBrains Mono', monospace;
    font-size: 10px;
}
.meta-item dd {
    margin: 0;
    color: var(--ink);
    font-size: 12px;
}

/* ───────── timeline ───────── */
main.transcript {
    max-width: 960px;
    margin: 0 auto;
    padding: 8px 24px 40px;
}
.evt {
    display: grid;
    grid-template-columns: 66px 46px 1fr;
    gap: 10px;
    padding: 3px 0;
    align-items: baseline;
    border-top: 1px solid transparent;
}
.evt .ts {
    font-size: 12px;
    color: var(--muted);
    padding-top: 2px;
}
.evt .tag {
    font-size: 10px;
    letter-spacing: 0.1em;
    color: var(--muted);
    text-transform: uppercase;
    padding-top: 4px;
    text-align: left;
}
.evt .body { min-width: 0; word-wrap: break-word; }

/* technical events (tool / phase / profile / safety / mode) render in mono;
   conversational events (pilot / input) inherit the body serif. */
.evt.tool .body,
.evt.phase .body,
.evt.profiles .body,
.evt.safety .body,
.evt.mode .body {
    font-family: 'JetBrains Mono', ui-monospace, 'SF Mono', Menlo, Consolas, monospace;
    font-variant-numeric: tabular-nums;
}

/* pilot LLM — the primary voice */
.evt.pilot .tag { color: var(--royal); font-weight: 500; }
.evt.pilot .body {
    font-size: 18px;
    line-height: 1.4;
    color: var(--ink);
}

/* tool calls — faded, recede into secondary status */
.evt.tool { opacity: 0.78; }
.evt.tool .ts { color: var(--tool-faint); }
.evt.tool .tag { color: var(--tool-faint); }
.evt.tool .tool-sum {
    font-size: 12px;
    color: var(--tool-faint);
    line-height: 1.35;
}
.evt.tool .tool-detail {
    display: block;
    margin-top: 2px;
}
.evt.tool .tool-detail summary {
    cursor: pointer;
    font-family: 'JetBrains Mono', monospace;
    font-size: 10px;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    color: var(--tool-faint);
    user-select: none;
    list-style: none;
    display: inline-flex;
    align-items: center;
    gap: 4px;
}
.evt.tool .tool-detail summary::-webkit-details-marker { display: none; }
.evt.tool .tool-detail summary::before {
    content: '▸';
    display: inline-block;
    transition: transform 0.12s ease;
    font-size: 10px;
}
.evt.tool .tool-detail[open] summary::before { transform: rotate(90deg); }
.evt.tool .tool-detail pre {
    margin: 4px 0 6px;
    padding: 8px 10px;
    background: var(--paper-deep);
    border-left: 2px solid var(--rule);
    font-size: 11.5px;
    line-height: 1.5;
    white-space: pre-wrap;
    word-break: break-word;
    overflow-x: auto;
    max-height: 280px;
    overflow-y: auto;
    color: var(--ink-soft);
}

/* phase changes — the primary state beats; tinted band, heavier type */
.evt.phase {
    background: var(--phase-bg);
    border-top: 1px solid var(--phase-rule);
    border-bottom: 1px solid var(--phase-rule);
    padding: 5px 10px 4px;
    margin: 4px -10px;
}
.evt.phase .ts { color: var(--amber); }
.evt.phase .tag {
    color: var(--amber);
    font-weight: 700;
}
.evt.phase .phase-change {
    font-size: 15px;
    color: var(--ink);
    font-weight: 500;
}
.evt.phase .phase-change em {
    font-family: 'Instrument Serif', serif;
    font-size: 22px;
    color: var(--royal);
    font-weight: 500;
}
.evt.phase .phase-extras {
    display: block;
    margin-top: 1px;
    font-size: 11px;
    color: var(--muted);
    letter-spacing: 0.02em;
}

/* profile-change heartbeats — sage-tinted band, same treatment as phase */
.evt.profiles {
    background: var(--profile-bg);
    border-top: 1px solid var(--profile-rule);
    border-bottom: 1px solid var(--profile-rule);
    padding: 5px 10px 4px;
    margin: 4px -10px;
}
.evt.profiles .ts { color: var(--sage); }
.evt.profiles .tag {
    color: var(--sage);
    font-weight: 700;
}
.evt.profiles .body {
    font-size: 14px;
    color: var(--ink);
    line-height: 1.4;
}

/* safety — red, compact but unmissable */
.evt.safety {
    background: var(--signal-bg);
    border-left: 2px solid var(--signal);
    padding: 4px 8px;
    margin: 3px -10px;
}
.evt.safety .ts { color: var(--signal); }
.evt.safety .tag { color: var(--signal); font-weight: 600; }
.evt.safety .body {
    font-size: 13px;
    color: var(--signal);
    letter-spacing: 0.03em;
}

/* operator / ATC input — display serif at pilot size, amber accent */
.evt.input {
    border-left: 2px solid var(--amber);
    padding-left: 6px;
    margin-left: -8px;
}
.evt.input .tag { color: var(--amber); font-weight: 600; }
.evt.input .body {
    font-size: 18px;
    line-height: 1.4;
    color: var(--ink);
}
.evt.input.input-atc { border-left-color: #8a3c12; }
.evt.input.input-atc .tag { color: #8a3c12; }
/* Pilot talking on the radio — royal accent so the transcript reads like
   a back-and-forth between ATC (orange-amber) and the pilot (navy). */
.evt.input.input-radio { border-left-color: var(--royal); }
.evt.input.input-radio .tag { color: var(--royal); }

/* mode switches */
.evt.mode .tag { color: var(--mauve); font-weight: 600; }
.evt.mode .body { font-size: 12px; color: var(--mauve); }

@media (max-width: 640px) {
    .masthead-inner { padding: 14px 16px 12px; }
    .evt { grid-template-columns: 56px 40px 1fr; gap: 8px; }
    .evt.pilot .body, .evt.input .body { font-size: 17px; }
    main.transcript { padding: 6px 16px 32px; }
}
</style>"#;
