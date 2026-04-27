//! "AI PILOT CONSOLE" multi-pane TUI — opt-in via `--tui-v2`.
//!
//! Layout (top → bottom):
//! ```text
//! ┌── header ────────────────────────────────────────────────────┐
//! ├── status strip ──────────────────────────────────────────────┤
//! ├── flight plan │ aircraft state + attitude │ tasks / tools ──┤
//! ├── radio                       │ action log ─────────────────┤
//! ├── comm strip ────────────────────────────────────────────────┤
//! ├── input ─────────────────────────────────────────────────────┤
//! ├── system status footer ──────────────────────────────────────┤
//! └──────────────────────────────────────────────────────────────┘
//! ```
//!
//! Shares input/log/radio/PTT plumbing with `tui::run_tui` via
//! `pub(crate)` helpers. Only the visual layout is new.

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use crossbeam_channel::Sender;
use crossterm::cursor::SetCursorStyle;
use crossterm::event::{
    self, Event, KeyCode, KeyEventKind, KeyModifiers, KeyboardEnhancementFlags,
    PopKeyboardEnhancementFlags, PushKeyboardEnhancementFlags,
};
use crossterm::execute;
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use parking_lot::Mutex as PLMutex;
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph, Wrap};
use ratatui::Terminal;

use crate::bus::{LogKind, SimBus};
use crate::core::mission_manager::{PilotCore, StatusSnapshot};
use crate::live_runner::HeartbeatPump;
use crate::llm::backend::{CacheSnapshot, CacheStats};
use crate::llm::conversation::{IncomingMessage, PilotMode};
use crate::transcribe::{PttController, PttMode, PttSnapshot};
use crate::tui::{
    amp_glyph, amp_level_from_smoothed, break_outer, build_log_lines, char_byte_index,
    handle_ptt_key, history_go_back, history_go_forward, history_record,
    insert_transcript_at_cursor, kill_to_row_end, parse_input_source, pin_to_bottom,
    render_radio_line, render_thinking_line, smart_end, smart_home, smooth_amp,
    wrap_input_spans, PttKeyTracker, INPUT_HISTORY_MAX,
};
use crate::types::FlightPhase;

const FRAME_INTERVAL: Duration = Duration::from_millis(100);

/// Run the v2 TUI on the calling thread. Same contract as `tui::run_tui`:
/// blocks until the user exits, then sets `stop_event` for graceful
/// shutdown of the control + LLM threads.
#[allow(clippy::too_many_arguments)]
pub fn run_tui_v2(
    bus: SimBus,
    input_queue: Sender<IncomingMessage>,
    stop_event: Arc<AtomicBool>,
    pilot: Arc<PLMutex<PilotCore>>,
    heartbeat_pump: Option<Arc<HeartbeatPump>>,
    cache_stats: Option<Arc<CacheStats>>,
    ptt: Option<Arc<PttController>>,
    aircraft_label: String,
    model_label: String,
) -> Result<()> {
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let _ = execute!(stdout, SetCursorStyle::SteadyBlock);
    let kitty_push_ok = execute!(
        stdout,
        PushKeyboardEnhancementFlags(KeyboardEnhancementFlags::REPORT_EVENT_TYPES)
    )
    .is_ok();
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut input_buffer = String::new();
    let mut input_cursor: usize = 0;
    let mut history: Vec<String> = Vec::new();
    let mut history_index: Option<usize> = None;
    let mut pending_draft: String = String::new();
    let mut log_detail = true;
    let mut show_thoughts = true;
    let mut log_scroll: Option<u16> = None;
    let mut last_log_pin: u16 = 0;
    let mut last_log_page: u16 = 10;
    let mut last_input_width: u16 = 0;
    let mut ptt_tracker = PttKeyTracker::default();
    let mut ptt_amp_smoothed: f32 = 0.0;
    let tui_epoch = Instant::now();

    let outcome = loop {
        let next_tick = Instant::now() + FRAME_INTERVAL;

        if let Some(p) = &ptt {
            while let Some(final_text) = p.try_recv_final() {
                insert_transcript_at_cursor(
                    &mut input_buffer,
                    &mut input_cursor,
                    &final_text,
                );
            }
        }

        // ---- key event drain ----
        loop {
            let timeout = next_tick.saturating_duration_since(Instant::now());
            if timeout.is_zero() {
                break;
            }
            if !event::poll(timeout)? {
                break;
            }
            if let Event::Key(k) = event::read()? {
                if matches!(k.kind, KeyEventKind::Repeat | KeyEventKind::Release) {
                    ptt_tracker.kitty_observed = true;
                }
                let is_ptt_key = matches!(k.code, KeyCode::Char(' ') | KeyCode::Tab)
                    && k.modifiers.is_empty();

                if k.kind == KeyEventKind::Release && !is_ptt_key {
                    continue;
                }

                if k.modifiers.contains(KeyModifiers::CONTROL)
                    && k.kind != KeyEventKind::Release
                {
                    match k.code {
                        KeyCode::Char('c') | KeyCode::Char('d') => {
                            break_outer(&stop_event);
                            break;
                        }
                        KeyCode::Char('t') => {
                            log_detail = !log_detail;
                            continue;
                        }
                        KeyCode::Char('o') => {
                            show_thoughts = !show_thoughts;
                            continue;
                        }
                        KeyCode::Char('a') => {
                            input_cursor = smart_home(input_cursor, last_input_width);
                            continue;
                        }
                        KeyCode::Char('e') => {
                            input_cursor =
                                smart_end(input_cursor, &input_buffer, last_input_width);
                            continue;
                        }
                        KeyCode::Char('k') => {
                            kill_to_row_end(
                                &mut input_buffer,
                                input_cursor,
                                last_input_width,
                            );
                            continue;
                        }
                        _ => {}
                    }
                }

                if is_ptt_key {
                    let was_recording = ptt_tracker
                        .pending
                        .as_ref()
                        .map(|p| p.recording)
                        .unwrap_or(false);
                    handle_ptt_key(
                        &k.kind,
                        &k.code,
                        &mut ptt_tracker,
                        &mut input_buffer,
                        &mut input_cursor,
                        ptt.as_deref(),
                    );
                    let now_recording = ptt_tracker
                        .pending
                        .as_ref()
                        .map(|p| p.recording)
                        .unwrap_or(false);
                    if now_recording && !was_recording {
                        bus.push_log_kind(
                            LogKind::Voice,
                            format!(
                                "voice: listening ({})",
                                if ptt_tracker.kitty_observed {
                                    "kitty"
                                } else {
                                    "fallback"
                                }
                            ),
                        );
                    } else if !now_recording && was_recording {
                        bus.push_log_kind(LogKind::Voice, "voice: finalizing");
                    }
                    continue;
                }

                ptt_tracker.pending = None;

                if k.kind == KeyEventKind::Release {
                    continue;
                }
                match k.code {
                    KeyCode::PageUp => {
                        let current = log_scroll.unwrap_or(last_log_pin);
                        log_scroll = Some(current.saturating_sub(last_log_page));
                        continue;
                    }
                    KeyCode::PageDown => {
                        if let Some(v) = log_scroll {
                            let next = v.saturating_add(last_log_page);
                            log_scroll = if next >= last_log_pin { None } else { Some(next) };
                        }
                        continue;
                    }
                    KeyCode::End => {
                        log_scroll = None;
                        continue;
                    }
                    KeyCode::Enter => {
                        if let Some(p) = &ptt {
                            if p.snapshot().mode != PttMode::Idle {
                                p.stop();
                                let deadline =
                                    Instant::now() + Duration::from_millis(300);
                                while Instant::now() < deadline {
                                    if let Some(t) = p.try_recv_final() {
                                        insert_transcript_at_cursor(
                                            &mut input_buffer,
                                            &mut input_cursor,
                                            &t,
                                        );
                                        break;
                                    }
                                    std::thread::sleep(Duration::from_millis(15));
                                }
                            }
                        }
                        let raw = input_buffer.trim().to_string();
                        if !raw.is_empty() {
                            history_record(&mut history, &raw, INPUT_HISTORY_MAX);
                            if let Some(rest) = raw.strip_prefix("/mode") {
                                let arg = rest.trim();
                                match PilotMode::parse(arg) {
                                    Some(mode) => {
                                        let _ = input_queue
                                            .send(IncomingMessage::mode_switch(mode));
                                        bus.push_log_kind(
                                            LogKind::Mode,
                                            format!("switched to {}", mode.label()),
                                        );
                                    }
                                    None => {
                                        bus.push_log_kind(
                                            LogKind::Error,
                                            format!(
                                                "unknown mode {:?} — expected normal | realistic",
                                                arg
                                            ),
                                        );
                                    }
                                }
                            } else {
                                let (source, payload) = parse_input_source(&raw);
                                let msg = match source.as_str() {
                                    "atc" => IncomingMessage::atc(payload.clone()),
                                    _ => IncomingMessage::operator(payload.clone()),
                                };
                                let _ = input_queue.send(msg);
                                if let Some(pump) = &heartbeat_pump {
                                    pump.record_user_input();
                                }
                                if source != "atc" {
                                    bus.push_log_kind(LogKind::Operator, payload);
                                }
                            }
                        }
                        input_buffer.clear();
                        input_cursor = 0;
                        history_index = None;
                        pending_draft.clear();
                    }
                    KeyCode::Esc => {
                        break_outer(&stop_event);
                        break;
                    }
                    KeyCode::Up => {
                        history_go_back(
                            &history,
                            &mut input_buffer,
                            &mut input_cursor,
                            &mut history_index,
                            &mut pending_draft,
                        );
                    }
                    KeyCode::Down => {
                        history_go_forward(
                            &history,
                            &mut input_buffer,
                            &mut input_cursor,
                            &mut history_index,
                            &mut pending_draft,
                        );
                    }
                    KeyCode::Left => {
                        input_cursor = input_cursor.saturating_sub(1);
                    }
                    KeyCode::Right => {
                        let total = input_buffer.chars().count();
                        if input_cursor < total {
                            input_cursor += 1;
                        }
                    }
                    KeyCode::Backspace => {
                        if input_cursor > 0 {
                            let byte_idx =
                                char_byte_index(&input_buffer, input_cursor - 1);
                            input_buffer.remove(byte_idx);
                            input_cursor -= 1;
                        }
                    }
                    KeyCode::Char(c) => {
                        let byte_idx = char_byte_index(&input_buffer, input_cursor);
                        input_buffer.insert(byte_idx, c);
                        input_cursor += 1;
                    }
                    _ => {}
                }
            }
        }

        if let Some(pending) = &ptt_tracker.pending {
            if pending.recording
                && pending.last_seen.elapsed() > Duration::from_millis(250)
            {
                if let Some(p) = &ptt {
                    p.stop();
                }
                ptt_tracker.pending = None;
            }
        }
        if stop_event.load(Ordering::Acquire) {
            break Ok(());
        }

        // ---- snapshot data for this frame ----
        let snapshot = pilot.lock().latest_snapshot.clone();
        let log_entries = bus.log_entries();
        let (_, _, radio) = bus.snapshot();
        let cache_snap = cache_stats.as_ref().map(|c| c.snapshot());
        let alerts_count = log_entries
            .iter()
            .filter(|e| matches!(e.kind, LogKind::Error | LogKind::Safety))
            .count() as u32;

        terminal.draw(|f| {
            let area = f.area();
            // Top-level vertical split. Footer + comm + input pinned to
            // bottom; body flexes; header + status pinned to top.
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Length(1), // header
                    Constraint::Length(1), // status strip
                    Constraint::Min(8),    // body (3-col)
                    Constraint::Length(8), // bottom pair (radio + log)
                    Constraint::Length(1), // comm strip
                    Constraint::Length(5), // input
                    Constraint::Length(1), // footer
                ])
                .split(area);

            // Header.
            f.render_widget(
                Paragraph::new(render_header(
                    chunks[0].width,
                    snapshot.as_ref(),
                    &aircraft_label,
                )),
                chunks[0],
            );

            // Status strip.
            f.render_widget(
                Paragraph::new(render_status_strip(
                    chunks[1].width,
                    &model_label,
                    cache_snap.as_ref(),
                    snapshot.as_ref(),
                )),
                chunks[1],
            );

            // Body: 3 columns.
            let body_cols = Layout::default()
                .direction(Direction::Horizontal)
                .constraints([
                    Constraint::Length(36), // flight plan
                    Constraint::Min(40),    // state + attitude
                    Constraint::Length(38), // tasks / tools
                ])
                .split(chunks[2]);

            // Col 1: flight plan.
            let fp_para = Paragraph::new(render_flight_plan_pane(snapshot.as_ref()))
                .block(Block::default().borders(Borders::ALL).title(" FLIGHT PLAN "));
            f.render_widget(fp_para, body_cols[0]);

            // Col 2: split top half = state, bottom half = attitude.
            let mid_split = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Length(9), // aircraft state (8 rows + borders)
                    Constraint::Min(5),    // attitude
                ])
                .split(body_cols[1]);
            let state_para = Paragraph::new(render_aircraft_state(snapshot.as_ref()))
                .block(Block::default().borders(Borders::ALL).title(" AIRCRAFT STATE "));
            f.render_widget(state_para, mid_split[0]);
            let adi_block = Block::default()
                .borders(Borders::ALL)
                .title(" ATTITUDE / FLIGHT DIRECTOR ");
            let adi_inner = adi_block.inner(mid_split[1]);
            f.render_widget(adi_block, mid_split[1]);
            let adi_lines = render_attitude(adi_inner, snapshot.as_ref());
            f.render_widget(Paragraph::new(adi_lines), adi_inner);

            // Col 3: tasks + tools.
            let agent_para = Paragraph::new(render_agent_pane(snapshot.as_ref(), alerts_count))
                .block(Block::default().borders(Borders::ALL).title(" AGENT / TASKS / TOOLS "));
            f.render_widget(agent_para, body_cols[2]);

            // Bottom pair: radio (left) + action log (right).
            let bottom_cols = Layout::default()
                .direction(Direction::Horizontal)
                .constraints([Constraint::Percentage(45), Constraint::Percentage(55)])
                .split(chunks[3]);

            // Radio.
            let radio_lines: Vec<Line> = radio.iter().flat_map(|l| render_radio_line(l)).collect();
            let radio_para = Paragraph::new(radio_lines.clone())
                .block(Block::default().borders(Borders::ALL).title(" ATC / RADIO TRANSCRIPT "))
                .wrap(Wrap { trim: false });
            f.render_widget(
                radio_para.scroll((pin_to_bottom(&radio_lines, bottom_cols[0]), 0)),
                bottom_cols[0],
            );

            // Action log.
            let log_inner_width = bottom_cols[1].width.saturating_sub(2);
            let mut log_lines = build_log_lines(
                &log_entries,
                log_detail,
                show_thoughts,
                log_inner_width,
            );
            if bus.is_llm_busy() {
                log_lines.push(render_thinking_line(tui_epoch.elapsed()));
            }
            let log_pin = pin_to_bottom(&log_lines, bottom_cols[1]);
            let inner_height = bottom_cols[1].height.saturating_sub(2);
            last_log_pin = log_pin;
            last_log_page = inner_height.max(3).saturating_sub(1);
            let (log_offset, scroll_indicator) = match log_scroll {
                Some(v) => {
                    let clamped = v.min(log_pin);
                    let up = log_pin.saturating_sub(clamped);
                    (clamped, if up > 0 { format!(" [↑{}]", up) } else { String::new() })
                }
                None => (log_pin, String::new()),
            };
            let base_title = if log_detail { " LLM / ACTION LOG " } else { " LLM / ACTION LOG (compact) " };
            let log_title = if scroll_indicator.is_empty() {
                base_title.to_string()
            } else {
                format!("{}{} ", base_title.trim_end(), scroll_indicator)
            };
            let log_para = Paragraph::new(log_lines.clone())
                .block(Block::default().borders(Borders::ALL).title(log_title))
                .wrap(Wrap { trim: false });
            f.render_widget(log_para.scroll((log_offset, 0)), bottom_cols[1]);

            // Comm strip.
            f.render_widget(
                Paragraph::new(render_comm_strip(
                    chunks[4].width,
                    snapshot.as_ref(),
                    &model_label,
                    cache_snap.as_ref(),
                )),
                chunks[4],
            );

            // Input pane.
            let input_title =
                " INPUT — space/tab hold to talk · enter send · ↑/↓ history · c-a/e/k edit · ctrl-t log detail · ctrl-o thoughts · pgup/pgdn/end scroll ";
            let ptt_snap: Option<PttSnapshot> = ptt.as_ref().map(|p| p.snapshot());
            let mut spans: Vec<Span> = vec![
                Span::styled("> ", Style::default().fg(Color::Cyan)),
                Span::raw(input_buffer.clone()),
            ];
            let recording = matches!(
                ptt_snap.as_ref().map(|s| &s.mode),
                Some(PttMode::Recording)
            );
            let finalizing = matches!(
                ptt_snap.as_ref().map(|s| &s.mode),
                Some(PttMode::Finalizing)
            );
            let warmup_hint = ptt_tracker
                .pending
                .as_ref()
                .filter(|p| !p.recording)
                .filter(|p| p.last_seen.elapsed() >= Duration::from_millis(120))
                .is_some();
            if recording {
                if let Some(snap) = &ptt_snap {
                    ptt_amp_smoothed = smooth_amp(ptt_amp_smoothed, snap.amp);
                }
            } else {
                ptt_amp_smoothed *= 0.3;
            }
            if let Some(snap) = &ptt_snap {
                if snap.mode != PttMode::Idle && !snap.partial.is_empty() {
                    let mut partial = String::new();
                    if !snap.prefix.is_empty() && input_buffer.is_empty() {
                        partial.push_str(&snap.prefix);
                    }
                    partial.push_str(&snap.partial);
                    spans.push(Span::styled(
                        partial,
                        Style::default().add_modifier(Modifier::DIM),
                    ));
                }
                if recording {
                    let level = amp_level_from_smoothed(ptt_amp_smoothed);
                    spans.push(Span::styled(
                        amp_glyph(level).to_string(),
                        Style::default().fg(Color::LightGreen),
                    ));
                } else if finalizing {
                    let t = tui_epoch.elapsed().as_secs_f64();
                    let phase = ((t / 0.8) * std::f64::consts::PI).sin();
                    let mut style = Style::default().fg(Color::Yellow);
                    if phase < 0.0 {
                        style = style.add_modifier(Modifier::DIM);
                    }
                    spans.push(Span::styled("…", style));
                }
            }
            if warmup_hint {
                spans.push(Span::styled(
                    " keep holding…".to_string(),
                    Style::default().add_modifier(Modifier::DIM),
                ));
            }
            let pane = chunks[5];
            let content_width = pane.width.saturating_sub(2).max(1);
            let content_height = pane.height.saturating_sub(2).max(1);
            last_input_width = content_width;
            let input_lines = wrap_input_spans(spans, content_width);
            let input_para = Paragraph::new(input_lines)
                .block(Block::default().borders(Borders::ALL).title(input_title));
            f.render_widget(input_para, pane);
            if !recording {
                let consumed = 2u16.saturating_add(input_cursor as u16);
                let row = (consumed / content_width).min(content_height.saturating_sub(1));
                let col = consumed % content_width;
                f.set_cursor_position((pane.x + 1 + col, pane.y + 1 + row));
            }

            // Footer.
            f.render_widget(
                Paragraph::new(render_footer(
                    chunks[6].width,
                    snapshot.as_ref(),
                    alerts_count,
                )),
                chunks[6],
            );
        })?;
    };

    if kitty_push_ok {
        let _ = execute!(terminal.backend_mut(), PopKeyboardEnhancementFlags);
    }
    let _ = execute!(terminal.backend_mut(), SetCursorStyle::DefaultUserShape);
    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;
    outcome
}

// ---------- header ----------

fn render_header(width: u16, snapshot: Option<&StatusSnapshot>, aircraft_label: &str) -> Line<'static> {
    let phase = snapshot
        .and_then(|s| s.phase)
        .map(|p| p.value().to_uppercase())
        .unwrap_or_else(|| "—".to_string());
    let t_sim = snapshot.map(|s| s.t_sim).unwrap_or(0.0);
    let h = (t_sim / 3600.0).floor() as u32;
    let m = ((t_sim % 3600.0) / 60.0).floor() as u32;
    let s = (t_sim % 60.0).floor() as u32;
    let title = " AI PILOT CONSOLE ";
    let sep = "  │  ";
    let left = format!(
        "{title}AIRCRAFT: {ac}{sep}PHASE: {phase}{sep}T+{h:02}:{m:02}:{s:02}",
        title = title,
        ac = aircraft_label,
        sep = sep,
        phase = phase,
        h = h,
        m = m,
        s = s,
    );
    let mut spans: Vec<Span<'static>> = vec![
        Span::styled(
            title.to_string(),
            Style::default().fg(Color::Black).bg(Color::Cyan).add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("AIRCRAFT: ")),
        Span::styled(aircraft_label.to_string(), Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
        Span::styled(sep.to_string(), Style::default().fg(Color::DarkGray)),
        Span::raw("PHASE: "),
        Span::styled(phase.clone(), phase_color_style(snapshot.and_then(|s| s.phase))),
        Span::styled(sep.to_string(), Style::default().fg(Color::DarkGray)),
        Span::raw(format!("T+{h:02}:{m:02}:{s:02}")),
    ];
    let used = visible_len(&left);
    let pad = (width as usize).saturating_sub(used);
    if pad > 0 {
        spans.push(Span::raw(" ".repeat(pad)));
    }
    Line::from(spans)
}

fn phase_color_style(phase: Option<FlightPhase>) -> Style {
    match phase {
        Some(FlightPhase::GoAround) => Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
        Some(FlightPhase::Final | FlightPhase::Flare | FlightPhase::Roundout | FlightPhase::Rollout) => {
            Style::default().fg(Color::LightGreen).add_modifier(Modifier::BOLD)
        }
        Some(_) => Style::default().fg(Color::Green).add_modifier(Modifier::BOLD),
        None => Style::default().fg(Color::DarkGray),
    }
}

// ---------- status strip ----------

fn render_status_strip(
    width: u16,
    model_label: &str,
    cache: Option<&CacheSnapshot>,
    snapshot: Option<&StatusSnapshot>,
) -> Line<'static> {
    let profiles = snapshot
        .map(|s| {
            if s.active_profiles.is_empty() {
                "(none)".to_string()
            } else {
                s.active_profiles.join("+")
            }
        })
        .unwrap_or_else(|| "—".to_string());
    let (input_tok, output_tok, hit) = match cache {
        Some(c) => {
            let hit = if c.total_input_tokens == 0 {
                0.0
            } else {
                (c.total_cached_tokens as f64 / c.total_input_tokens as f64) * 100.0
            };
            (c.total_input_tokens, c.total_output_tokens, hit)
        }
        None => (0, 0, 0.0),
    };
    let reqs = cache.map(|c| c.total_requests).unwrap_or(0);
    let safety = snapshot
        .and_then(|s| s.go_around_reason.as_ref())
        .map(|_| "GO-AROUND")
        .unwrap_or("NOMINAL");
    let safety_style = if safety == "NOMINAL" {
        Style::default().fg(Color::LightGreen).add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
    };

    let sep = "  │  ";
    let mut spans: Vec<Span<'static>> = vec![
        Span::styled(" LLM: ", Style::default().fg(Color::DarkGray)),
        Span::styled(
            model_label.to_string(),
            Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
        ),
        Span::styled(sep, Style::default().fg(Color::DarkGray)),
        Span::raw("PROFILES: "),
        Span::styled(profiles.clone(), Style::default().fg(Color::White)),
        Span::styled(sep, Style::default().fg(Color::DarkGray)),
        Span::raw("TOKENS: "),
        Span::styled(
            format!("↓{} ↑{}", fmt_int(input_tok), fmt_int(output_tok)),
            Style::default().fg(Color::White),
        ),
        Span::styled(sep, Style::default().fg(Color::DarkGray)),
        Span::raw("CACHE: "),
        Span::styled(
            format!("{:.0}%", hit),
            Style::default().fg(Color::Green),
        ),
        Span::styled(sep, Style::default().fg(Color::DarkGray)),
        Span::raw("REQS: "),
        Span::styled(format!("{}", reqs), Style::default().fg(Color::White)),
        Span::styled(sep, Style::default().fg(Color::DarkGray)),
        Span::raw("SAFETY: "),
        Span::styled(safety.to_string(), safety_style),
    ];
    let used: usize = spans.iter().map(|s| s.content.chars().count()).sum();
    let pad = (width as usize).saturating_sub(used);
    if pad > 0 {
        spans.push(Span::raw(" ".repeat(pad)));
    }
    Line::from(spans)
}

fn fmt_int(n: u64) -> String {
    let s = n.to_string();
    let bytes = s.as_bytes();
    let mut out = String::with_capacity(s.len() + s.len() / 3);
    for (i, b) in bytes.iter().enumerate() {
        if i > 0 && (bytes.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(*b as char);
    }
    out
}

// ---------- flight plan pane ----------

fn render_flight_plan_pane(snapshot: Option<&StatusSnapshot>) -> Vec<Line<'static>> {
    let mut lines: Vec<Line<'static>> = Vec::new();
    let Some(s) = snapshot else {
        lines.push(Line::from(Span::styled(
            " (waiting for first tick…) ",
            Style::default().fg(Color::DarkGray),
        )));
        return lines;
    };

    let airport = s.airport_ident.clone().unwrap_or_else(|| "—".into());
    let runway = s.runway_id.clone().unwrap_or_else(|| "—".into());
    let elev = s
        .field_elevation_ft
        .map(|e| format!("{:.0} ft", e))
        .unwrap_or_else(|| "—".into());

    lines.push(kv_line("AIRPORT", &airport, Color::White));
    lines.push(kv_line("RUNWAY", &runway, Color::White));
    lines.push(kv_line("FIELD ELEV", &elev, Color::White));
    lines.push(blank());

    // Active waypoint from guidance targets.
    let wp = s.last_guidance.as_ref().and_then(|g| g.target_waypoint.as_ref());
    lines.push(Line::from(Span::styled(
        "ACTIVE WAYPOINT",
        Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
    )));
    if let Some(wp) = wp {
        let dx = wp.position_ft.x - s.state.position_ft.x;
        let dy = wp.position_ft.y - s.state.position_ft.y;
        let dist_ft = (dx * dx + dy * dy).sqrt();
        let dist_nm = dist_ft / 6076.1155;
        let bearing = (dx.atan2(dy).to_degrees() + 360.0) % 360.0;
        let alt = wp
            .altitude_ft
            .map(|a| format!("{:.0} ft", a))
            .unwrap_or_else(|| "—".into());
        lines.push(kv_line("  NAME", &wp.name, Color::White));
        lines.push(kv_line("  DIST", &format!("{:.1} NM", dist_nm), Color::White));
        lines.push(kv_line("  BEARING", &format!("{:03.0}°", bearing), Color::White));
        lines.push(kv_line("  TGT ALT", &alt, Color::White));
    } else {
        lines.push(Line::from(Span::styled(
            "  (no active route)",
            Style::default().fg(Color::DarkGray),
        )));
    }
    lines.push(blank());

    // Targets snapshot — what the guidance is actually steering toward.
    if let Some(g) = s.last_guidance.as_ref() {
        lines.push(Line::from(Span::styled(
            "TARGETS",
            Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
        )));
        let hdg = g
            .target_heading_deg
            .or(g.target_track_deg)
            .map(|v| format!("{:03.0}°", (v + 360.0) % 360.0))
            .unwrap_or_else(|| "—".into());
        let alt = g
            .target_altitude_ft
            .map(|v| format!("{:.0} ft", v))
            .unwrap_or_else(|| "—".into());
        let spd = g
            .target_speed_kt
            .map(|v| format!("{:.0} kt", v))
            .unwrap_or_else(|| "—".into());
        lines.push(kv_line("  HEADING", &hdg, Color::White));
        lines.push(kv_line("  ALTITUDE", &alt, Color::White));
        lines.push(kv_line("  SPEED", &spd, Color::White));
    }

    lines
}

fn kv_line(label: &str, value: &str, value_fg: Color) -> Line<'static> {
    let label_w: usize = 12;
    let pad = label_w.saturating_sub(label.chars().count());
    Line::from(vec![
        Span::raw(format!("{}{}", label, " ".repeat(pad))),
        Span::raw(" "),
        Span::styled(value.to_string(), Style::default().fg(value_fg)),
    ])
}

fn blank() -> Line<'static> {
    Line::from(Span::raw(""))
}

// ---------- aircraft state pane ----------

fn render_aircraft_state(snapshot: Option<&StatusSnapshot>) -> Vec<Line<'static>> {
    let Some(s) = snapshot else {
        return vec![Line::from(Span::styled(
            " (waiting…)",
            Style::default().fg(Color::DarkGray),
        ))];
    };
    let st = &s.state;

    // Two-column rows: (label-left, value-left, label-right, value-right).
    let rows: Vec<(&'static str, String, &'static str, String)> = vec![
        (
            "ALTITUDE",
            format!("{:>6.0} ft  ({:>5.0} AGL)", st.alt_msl_ft, st.alt_agl_ft),
            "VS",
            format!("{:+5.0} fpm", st.vs_fpm),
        ),
        (
            "AIRSPEED",
            format!("{:>3.0} kt IAS / {:>3.0} kt GS", st.ias_kt, st.gs_kt),
            "TAS",
            format!("{:>3.0} kt", st.tas_kt),
        ),
        (
            "HDG (TRK)",
            format!(
                "{:03.0}°  ({:03.0}°)",
                (st.heading_deg + 360.0) % 360.0,
                (st.track_deg + 360.0) % 360.0,
            ),
            "BANK",
            format!(
                "{:>4.1}°{}",
                st.roll_deg.abs(),
                if st.roll_deg.abs() < 0.5 {
                    " "
                } else if st.roll_deg < 0.0 {
                    " L"
                } else {
                    " R"
                }
            ),
        ),
        (
            "PITCH",
            format!("{:+5.1}°", st.pitch_deg),
            "GEAR",
            (if st.gear_down { "DOWN" } else { "UP" }).to_string(),
        ),
        (
            "THROTTLE",
            format!(
                "{}  {:>3.0}%",
                throttle_bar(st.throttle_pos, 8),
                st.throttle_pos * 100.0,
            ),
            "FLAPS",
            format!("{}°", st.flap_index),
        ),
        (
            "STALL MARGIN",
            format!("{:.2} kt", st.stall_margin),
            "GROUND",
            (if st.on_ground { "ON" } else { "AIRBORNE" }).to_string(),
        ),
    ];

    let mut out: Vec<Line<'static>> = Vec::with_capacity(rows.len());
    for (l1, v1, l2, v2) in rows {
        let left_w = 34;
        let mut left = format!("{:<10}{}", l1, v1);
        if left.chars().count() < left_w {
            left.push_str(&" ".repeat(left_w - left.chars().count()));
        }
        out.push(Line::from(vec![
            Span::styled(
                format!("{:<10}", l1),
                Style::default().fg(Color::DarkGray),
            ),
            Span::styled(format!("{:<22}", v1), Style::default().fg(Color::White)),
            Span::styled(
                format!("{:<8}", l2),
                Style::default().fg(Color::DarkGray),
            ),
            Span::styled(v2, Style::default().fg(Color::White)),
        ]));
    }
    out
}

fn throttle_bar(value: f64, width: usize) -> String {
    let v = value.clamp(0.0, 1.0);
    let filled = (v * width as f64).round() as usize;
    let filled = filled.min(width);
    let mut out = String::with_capacity(width);
    for _ in 0..filled {
        out.push('█');
    }
    for _ in filled..width {
        out.push('░');
    }
    out
}

// ---------- attitude indicator ----------

/// Render a synthetic ADI: horizon line tilted by roll, shifted vertically
/// by pitch. Heading tape across the bottom; pitch ladder on the sides.
pub(crate) fn render_attitude(area: Rect, snapshot: Option<&StatusSnapshot>) -> Vec<Line<'static>> {
    let w = area.width as i32;
    let h = area.height as i32;
    if w < 20 || h < 5 {
        return vec![Line::from(Span::styled(
            "(too small)",
            Style::default().fg(Color::DarkGray),
        ))];
    }
    let (pitch, roll, hdg) = match snapshot {
        Some(s) => (s.state.pitch_deg, s.state.roll_deg, s.state.heading_deg),
        None => (0.0, 0.0, 0.0),
    };

    let cx = w / 2;
    let cy = h / 2;
    // 1° pitch == ~1 row; clamp so the horizon stays visible.
    let pitch_offset_rows = (pitch / 4.0).round() as i32;

    // Build a 2D char grid.
    let mut grid: Vec<Vec<(char, Color)>> = vec![vec![(' ', Color::Reset); w as usize]; h as usize];

    // Horizon: for each column x, compute y on the tilted line.
    // y = cy - pitch_offset + (x - cx) * tan(roll)
    let tan_roll = (roll.to_radians()).tan();
    for x in 0..w {
        let y_f = (cy as f64) - pitch_offset_rows as f64 + (x as f64 - cx as f64) * tan_roll;
        let y = y_f.round() as i32;
        // Sky / ground fill above and below.
        for yy in 0..h {
            if yy < y {
                grid[yy as usize][x as usize] = ('·', Color::Blue);
            } else if yy > y {
                grid[yy as usize][x as usize] = ('▒', Color::Rgb(120, 70, 30));
            }
        }
        if (0..h).contains(&y) {
            grid[y as usize][x as usize] = ('━', Color::White);
        }
    }

    // Center reticle: little aircraft symbol.
    if (0..h).contains(&cy) && (0..w).contains(&(cx - 3)) && (0..w).contains(&(cx + 3)) {
        for &dx in &[-3i32, -2, -1] {
            grid[cy as usize][(cx + dx) as usize] = ('─', Color::Yellow);
        }
        for &dx in &[1i32, 2, 3] {
            grid[cy as usize][(cx + dx) as usize] = ('─', Color::Yellow);
        }
        grid[cy as usize][cx as usize] = ('◆', Color::Yellow);
    }

    // Pitch ladder: small ticks at ±10°.
    for &deg in &[10.0_f64, -10.0] {
        let rows_off = (deg / 4.0).round() as i32;
        let y = cy - pitch_offset_rows + rows_off;
        if (0..h).contains(&y) {
            for off in [-6i32, -5, 5, 6] {
                let xx = cx + off;
                if (0..w).contains(&xx) {
                    grid[y as usize][xx as usize] = ('─', Color::White);
                }
            }
        }
    }

    // Heading tape on the bottom row.
    if h >= 4 {
        let bottom = (h - 1) as usize;
        let hdg_norm = ((hdg % 360.0) + 360.0) % 360.0;
        for x in 0..w {
            grid[bottom][x as usize] = (' ', Color::Reset);
        }
        let center_str = format!("{:03.0}", hdg_norm);
        let start = (cx as usize).saturating_sub(center_str.len() / 2);
        for (i, ch) in center_str.chars().enumerate() {
            if start + i < w as usize {
                grid[bottom][start + i] = (ch, Color::LightCyan);
            }
        }
        // Brackets.
        if start >= 1 {
            grid[bottom][start - 1] = ('[', Color::Cyan);
        }
        if start + center_str.len() < w as usize {
            grid[bottom][start + center_str.len()] = (']', Color::Cyan);
        }
    }

    // Roll arc indicator at top: tick at the current bank.
    if h >= 3 {
        let top = 0usize;
        let bank_col = cx + (roll / 60.0 * (w as f64 / 2.0 - 2.0)) as i32;
        if (0..w).contains(&bank_col) {
            grid[top][bank_col as usize] = ('▼', Color::Yellow);
        }
        // Reference notches at 0, ±30.
        for &deg in &[-30.0_f64, 0.0, 30.0] {
            let col = cx + (deg / 60.0 * (w as f64 / 2.0 - 2.0)) as i32;
            if (0..w).contains(&col) {
                let g = if deg.abs() < 0.1 { '┬' } else { '·' };
                if grid[top][col as usize].0 == ' ' {
                    grid[top][col as usize] = (g, Color::DarkGray);
                }
            }
        }
    }

    // Convert grid to lines.
    grid
        .into_iter()
        .map(|row| {
            let mut spans: Vec<Span<'static>> = Vec::with_capacity(row.len());
            // Coalesce same-color runs to keep span count down.
            let mut cur_color = row[0].1;
            let mut buf = String::new();
            for (ch, color) in row {
                if color == cur_color {
                    buf.push(ch);
                } else {
                    spans.push(Span::styled(
                        std::mem::take(&mut buf),
                        Style::default().fg(cur_color),
                    ));
                    buf.push(ch);
                    cur_color = color;
                }
            }
            if !buf.is_empty() {
                spans.push(Span::styled(buf, Style::default().fg(cur_color)));
            }
            Line::from(spans)
        })
        .collect()
}

// ---------- agent / tasks / tools pane ----------

fn render_agent_pane(snapshot: Option<&StatusSnapshot>, alerts_count: u32) -> Vec<Line<'static>> {
    let mut lines: Vec<Line<'static>> = Vec::new();
    let Some(s) = snapshot else {
        return vec![Line::from(Span::styled(
            "(waiting…)",
            Style::default().fg(Color::DarkGray),
        ))];
    };

    // Active goal line — synthesized from the active profile + targets.
    let goal = synthesize_goal(s);
    lines.push(Line::from(Span::styled(
        "ACTIVE GOAL",
        Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
    )));
    lines.push(Line::from(Span::styled(
        goal,
        Style::default().fg(Color::LightGreen),
    )));
    lines.push(blank());

    // Task queue derived from the canonical phase progression.
    lines.push(Line::from(Span::styled(
        "TASK QUEUE",
        Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
    )));
    for entry in derive_task_queue(s.phase) {
        let (sigil, sigil_style, label_style) = match entry.status {
            TaskStatus::Done => (
                "✓",
                Style::default().fg(Color::Green).add_modifier(Modifier::BOLD),
                Style::default().fg(Color::DarkGray),
            ),
            TaskStatus::Active => (
                "▶",
                Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
                Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
            ),
            TaskStatus::Pending => (
                "·",
                Style::default().fg(Color::DarkGray),
                Style::default().fg(Color::DarkGray),
            ),
        };
        let status_text = match entry.status {
            TaskStatus::Done => "DONE",
            TaskStatus::Active => "ACTIVE",
            TaskStatus::Pending => "PENDING",
        };
        let row_w = 36;
        let label_w = row_w - 2 - 1 - status_text.len() - 1;
        let mut label = entry.label.to_string();
        if label.chars().count() > label_w {
            label.truncate(label_w);
        }
        let pad = label_w.saturating_sub(label.chars().count());
        lines.push(Line::from(vec![
            Span::styled(format!("{} ", sigil), sigil_style),
            Span::styled(format!("{}{}", label, " ".repeat(pad)), label_style),
            Span::styled(status_text.to_string(), sigil_style),
        ]));
    }
    lines.push(blank());

    // Tools / systems status.
    lines.push(Line::from(Span::styled(
        "TOOLS / SYSTEMS",
        Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
    )));
    let on_ground = s.state.on_ground;
    let nav_ok = !on_ground || s.state.runway_x_ft.is_some();
    let autopilot_engaged = !s.active_profiles.is_empty()
        && !s
            .active_profiles
            .iter()
            .all(|n| n.starts_with("idle_"));
    let alerts_state = if alerts_count == 0 { "OK" } else { "ALERT" };
    let alerts_color = if alerts_count == 0 { Color::Green } else { Color::Red };

    for (name, ok, detail, color) in [
        ("NAV (GPS)", nav_ok, "RX", Color::Green),
        (
            "AUTOPILOT",
            autopilot_engaged,
            if autopilot_engaged { "engaged" } else { "stby" },
            if autopilot_engaged { Color::Green } else { Color::DarkGray },
        ),
        ("COMMS", true, "RX/TX", Color::Green),
        ("ENGINE", !on_ground || s.state.throttle_pos > 0.0, "OK", Color::Green),
        (
            "GEAR",
            true,
            if s.state.gear_down { "DOWN" } else { "UP" },
            if s.state.gear_down { Color::Green } else { Color::Yellow },
        ),
        ("ALERTS", true, alerts_state, alerts_color),
    ] {
        let badge = if ok { "OK" } else { "—" };
        let badge_style = if ok {
            Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)
        };
        let label_w: usize = 14;
        let pad = label_w.saturating_sub(name.chars().count());
        lines.push(Line::from(vec![
            Span::styled(
                format!("{}{}", name, " ".repeat(pad)),
                Style::default().fg(Color::DarkGray),
            ),
            Span::styled(format!("{:<4}", badge), badge_style),
            Span::raw(" "),
            Span::styled(detail.to_string(), Style::default().fg(color)),
        ]));
    }

    lines
}

fn synthesize_goal(s: &StatusSnapshot) -> String {
    if let Some(reason) = s.go_around_reason.as_ref() {
        return format!("Go-around: {}", reason);
    }
    let phase_word = s
        .phase
        .map(|p| match p {
            FlightPhase::Preflight => "Hold for clearance",
            FlightPhase::TakeoffRoll => "Departure roll",
            FlightPhase::Rotate => "Rotate",
            FlightPhase::InitialClimb | FlightPhase::EnrouteClimb => "Climb to altitude",
            FlightPhase::Cruise => "Cruise to destination",
            FlightPhase::Descent => "Descend for approach",
            FlightPhase::PatternEntry => "Enter pattern",
            FlightPhase::Crosswind => "Crosswind leg",
            FlightPhase::Downwind => "Fly downwind",
            FlightPhase::Base => "Turn base",
            FlightPhase::Final => "Final approach",
            FlightPhase::Roundout => "Roundout",
            FlightPhase::Flare => "Flare",
            FlightPhase::Rollout => "Rollout",
            FlightPhase::RunwayExit => "Exit runway",
            FlightPhase::TaxiClear => "Taxi clear of runway",
            FlightPhase::GoAround => "Go-around climb",
        })
        .unwrap_or("Standby");
    let runway = s
        .runway_id
        .as_ref()
        .map(|r| format!(" RWY {}", r))
        .unwrap_or_default();
    let airport = s
        .airport_ident
        .as_ref()
        .map(|a| format!(" @ {}", a))
        .unwrap_or_default();
    format!("{}{}{}", phase_word, runway, airport)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TaskStatus {
    Done,
    Active,
    Pending,
}

#[derive(Debug, Clone)]
struct TaskEntry {
    label: &'static str,
    status: TaskStatus,
}

/// Synthesize a fixed task queue from the canonical phase progression.
/// The active phase becomes ACTIVE, prior phases DONE, later PENDING.
/// GoAround is overlaid as ACTIVE alongside the main mission progression.
fn derive_task_queue(phase: Option<FlightPhase>) -> Vec<TaskEntry> {
    let canon: &[(FlightPhase, &'static str)] = &[
        (FlightPhase::Preflight, "Preflight"),
        (FlightPhase::TakeoffRoll, "Takeoff roll"),
        (FlightPhase::InitialClimb, "Climb out"),
        (FlightPhase::Cruise, "Cruise"),
        (FlightPhase::Descent, "Descend for approach"),
        (FlightPhase::PatternEntry, "Enter pattern"),
        (FlightPhase::Downwind, "Downwind"),
        (FlightPhase::Base, "Base"),
        (FlightPhase::Final, "Final approach"),
        (FlightPhase::Flare, "Flare & touchdown"),
        (FlightPhase::TaxiClear, "Exit & taxi clear"),
    ];

    // Pick the canonical index that best matches the live phase.
    let live_idx = phase.and_then(|p| match p {
        FlightPhase::Preflight => Some(0),
        FlightPhase::TakeoffRoll | FlightPhase::Rotate => Some(1),
        FlightPhase::InitialClimb | FlightPhase::Crosswind | FlightPhase::EnrouteClimb => Some(2),
        FlightPhase::Cruise => Some(3),
        FlightPhase::Descent => Some(4),
        FlightPhase::PatternEntry => Some(5),
        FlightPhase::Downwind => Some(6),
        FlightPhase::Base => Some(7),
        FlightPhase::Final => Some(8),
        FlightPhase::Roundout | FlightPhase::Flare | FlightPhase::Rollout | FlightPhase::RunwayExit => {
            Some(9)
        }
        FlightPhase::TaxiClear => Some(10),
        FlightPhase::GoAround => Some(8), // back to final
    });

    canon
        .iter()
        .enumerate()
        .map(|(i, (_, label))| {
            let status = match live_idx {
                Some(idx) if i < idx => TaskStatus::Done,
                Some(idx) if i == idx => TaskStatus::Active,
                _ => TaskStatus::Pending,
            };
            TaskEntry { label, status }
        })
        .collect()
}

// ---------- comm strip ----------

fn render_comm_strip(
    width: u16,
    snapshot: Option<&StatusSnapshot>,
    model_label: &str,
    cache: Option<&CacheSnapshot>,
) -> Line<'static> {
    let runway = snapshot
        .and_then(|s| s.runway_id.clone())
        .unwrap_or_else(|| "—".into());
    let airport = snapshot
        .and_then(|s| s.airport_ident.clone())
        .unwrap_or_else(|| "—".into());
    let context_tok = cache.map(|c| c.total_input_tokens).unwrap_or(0);
    let reqs = cache.map(|c| c.total_requests).unwrap_or(0);

    let sep = "  │  ";
    let mut spans: Vec<Span<'static>> = vec![
        Span::styled(" RWY ", Style::default().fg(Color::DarkGray)),
        Span::styled(runway.clone(), Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
        Span::styled(sep, Style::default().fg(Color::DarkGray)),
        Span::raw("FIELD: "),
        Span::styled(airport.clone(), Style::default().fg(Color::White)),
        Span::styled(sep, Style::default().fg(Color::DarkGray)),
        Span::raw("DATA LINK: "),
        Span::styled("OK", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
        Span::styled(sep, Style::default().fg(Color::DarkGray)),
        Span::raw("MODEL: "),
        Span::styled(model_label.to_string(), Style::default().fg(Color::Cyan)),
        Span::styled(sep, Style::default().fg(Color::DarkGray)),
        Span::raw("CONTEXT: "),
        Span::styled(format!("{}", fmt_int(context_tok)), Style::default().fg(Color::White)),
        Span::styled(sep, Style::default().fg(Color::DarkGray)),
        Span::raw("REQS: "),
        Span::styled(format!("{}", reqs), Style::default().fg(Color::White)),
    ];
    let used: usize = spans.iter().map(|s| s.content.chars().count()).sum();
    let pad = (width as usize).saturating_sub(used);
    if pad > 0 {
        spans.push(Span::raw(" ".repeat(pad)));
    }
    Line::from(spans)
}

// ---------- footer ----------

fn render_footer(width: u16, snapshot: Option<&StatusSnapshot>, alerts: u32) -> Line<'static> {
    let s = snapshot;
    let on_ground = s.map(|s| s.state.on_ground).unwrap_or(true);
    let gear_down = s.map(|s| s.state.gear_down).unwrap_or(true);
    let flap_idx = s.map(|s| s.state.flap_index).unwrap_or(0);
    let stall = s.map(|s| s.state.stall_margin).unwrap_or(99.0);
    let active = s.map(|s| !s.active_profiles.iter().all(|n| n.starts_with("idle_"))).unwrap_or(false);
    let go_around = s.and_then(|s| s.go_around_reason.as_ref()).is_some();

    let nav_status = if !on_ground || s.and_then(|s| s.state.runway_x_ft).is_some() {
        ("NAV ", "GOOD", Color::Green)
    } else {
        ("NAV ", "—", Color::DarkGray)
    };
    let ap_status = if active {
        ("A/P ", "ENGAGED", Color::Green)
    } else {
        ("A/P ", "STANDBY", Color::DarkGray)
    };
    let comms_status = ("COMMS ", "OK", Color::Green);
    let engine_status = if !on_ground || s.map(|s| s.state.throttle_pos > 0.0).unwrap_or(false) {
        ("ENG ", "RUN", Color::Green)
    } else {
        ("ENG ", "IDLE", Color::DarkGray)
    };
    let gear_status = (
        "GEAR ",
        if gear_down { "DOWN" } else { "UP" },
        if gear_down { Color::Green } else { Color::Yellow },
    );
    let flaps_status = (
        "FLAPS ",
        Box::leak(format!("{}°", flap_idx).into_boxed_str()) as &str,
        Color::White,
    );
    let stall_status = if stall < 5.0 {
        ("STALL ", "WARN", Color::Yellow)
    } else {
        ("STALL ", "OK", Color::Green)
    };
    let safety_status = if go_around {
        ("SAFETY ", "GO-AROUND", Color::Yellow)
    } else if alerts > 0 {
        ("SAFETY ", "ALERT", Color::Red)
    } else {
        ("SAFETY ", "NOMINAL", Color::Green)
    };
    let alerts_label = Box::leak(format!("{}", alerts).into_boxed_str()) as &str;
    let alerts_color = if alerts == 0 { Color::Green } else { Color::Red };
    let alerts_status = ("ALERTS ", alerts_label, alerts_color);

    let groups = [
        nav_status,
        ap_status,
        comms_status,
        engine_status,
        gear_status,
        flaps_status,
        stall_status,
        safety_status,
        alerts_status,
    ];

    let mut spans: Vec<Span<'static>> = Vec::with_capacity(groups.len() * 4);
    spans.push(Span::styled(
        " STATUS ",
        Style::default().fg(Color::Black).bg(Color::White).add_modifier(Modifier::BOLD),
    ));
    for (i, (label, value, color)) in groups.iter().enumerate() {
        if i > 0 {
            spans.push(Span::styled(" │ ", Style::default().fg(Color::DarkGray)));
        } else {
            spans.push(Span::raw(" "));
        }
        spans.push(Span::styled(
            label.to_string(),
            Style::default().fg(Color::DarkGray),
        ));
        spans.push(Span::styled(
            value.to_string(),
            Style::default().fg(*color).add_modifier(Modifier::BOLD),
        ));
    }
    let used: usize = spans.iter().map(|s| s.content.chars().count()).sum();
    let pad = (width as usize).saturating_sub(used);
    if pad > 0 {
        spans.push(Span::raw(" ".repeat(pad)));
    }
    Line::from(spans)
}

fn visible_len(s: &str) -> usize {
    s.chars().count()
}

// ---------- tests ----------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fmt_int_groups_thousands() {
        assert_eq!(fmt_int(0), "0");
        assert_eq!(fmt_int(999), "999");
        assert_eq!(fmt_int(1_000), "1,000");
        assert_eq!(fmt_int(1_234_567), "1,234,567");
    }

    #[test]
    fn task_queue_marks_active_phase() {
        let q = derive_task_queue(Some(FlightPhase::Final));
        let active: Vec<&str> = q
            .iter()
            .filter(|t| matches!(t.status, TaskStatus::Active))
            .map(|t| t.label)
            .collect();
        assert_eq!(active, vec!["Final approach"]);
        // Earlier phases must be DONE.
        assert!(q
            .iter()
            .take_while(|t| t.label != "Final approach")
            .all(|t| matches!(t.status, TaskStatus::Done)));
        // Later phases must be PENDING.
        assert!(q
            .iter()
            .skip_while(|t| t.label != "Final approach")
            .skip(1)
            .all(|t| matches!(t.status, TaskStatus::Pending)));
    }

    #[test]
    fn task_queue_with_no_phase_is_all_pending() {
        let q = derive_task_queue(None);
        assert!(q.iter().all(|t| matches!(t.status, TaskStatus::Pending)));
    }

    #[test]
    fn throttle_bar_full_and_empty() {
        assert_eq!(throttle_bar(0.0, 4), "░░░░");
        assert_eq!(throttle_bar(1.0, 4), "████");
        assert_eq!(throttle_bar(0.5, 4), "██░░");
    }
}
