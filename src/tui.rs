//! Terminal UI + status-line formatter. Mirrors sim_pilot/tui.py.
//!
//! Two surfaces live here:
//!   - `format_snapshot_display` / `parse_input_source`: pure functions used
//!     by the headless control loop and the test suite.
//!   - `run_tui`: a ratatui-based interactive UI with four panes (status,
//!     log, radio, input) + key bindings for submit / toggle-detail / exit.
//!
//! The interactive runner polls the shared `SimBus` and `PilotCore.latest_snapshot`
//! every 100 ms and redraws. Operator/ATC input is parsed via
//! `parse_input_source` and pushed onto the LLM input queue as
//! `IncomingMessage`.

use crate::core::mission_manager::StatusSnapshot;
use crate::types::FlightPhase;
#[cfg(test)]
use crate::types::wrap_degrees_180;

/// Parse an optional `atc:` / `[operator]` prefix from a user-entered line.
/// Returns `(source, payload)` where `source` is either "operator" or "atc".
pub fn parse_input_source(raw: &str) -> (String, String) {
    let text = raw.trim();
    let lowered = text.to_ascii_lowercase();
    for prefix in ["atc:", "[atc]"] {
        if lowered.starts_with(prefix) {
            return (
                "atc".to_string(),
                text[prefix.len()..].trim().to_string(),
            );
        }
    }
    for prefix in ["operator:", "[operator]"] {
        if lowered.starts_with(prefix) {
            return (
                "operator".to_string(),
                text[prefix.len()..].trim().to_string(),
            );
        }
    }
    ("operator".to_string(), text.to_string())
}

const SEP: &str = "────────────────────────────────────────────────────────────────";
const THR_BAR_WIDTH: usize = 8;
const THR_FILL: char = '█';
const THR_EMPTY: char = '░';

fn fmt_right(value: &str, width: usize) -> String {
    format!("{:>width$}", value, width = width)
}
fn fmt_left(value: &str, width: usize) -> String {
    format!("{:<width$}", value, width = width)
}

/// Render the plain-text (no-ANSI) status panel. Mirrors `format_snapshot_display`.
pub fn format_snapshot_display(snapshot: Option<&StatusSnapshot>) -> String {
    let Some(snapshot) = snapshot else {
        return "  (waiting for first pilot tick)".to_string();
    };

    let state = &snapshot.state;
    let guidance = snapshot.last_guidance.as_ref();
    let commands = &snapshot.last_commands;

    let profiles = if snapshot.active_profiles.is_empty() {
        "(none)".to_string()
    } else {
        decorate_profiles(
            &snapshot.active_profiles,
            &snapshot.profile_mode_line_suffixes,
        )
    };
    let phase = snapshot
        .phase
        .map(|p| p.value().to_string())
        .unwrap_or_else(|| "—".to_string());

    let tgt_hdg = guidance
        .and_then(|g| g.target_heading_deg.or(g.target_track_deg));
    let tgt_alt_msl = guidance.and_then(|g| g.target_altitude_ft);
    let tgt_spd = guidance.and_then(|g| g.target_speed_kt);

    let field_elev = state.alt_msl_ft - state.alt_agl_ft;
    let tgt_alt_agl = tgt_alt_msl.map(|a| a - field_elev);

    let mut out = String::new();

    let rwy_info = {
        let mut parts: Vec<String> = Vec::new();
        if let Some(a) = &snapshot.airport_ident {
            parts.push(a.clone());
        }
        if let Some(r) = &snapshot.runway_id {
            parts.push(format!("rwy {r}"));
        }
        let mut s = parts.join(" ");
        if let Some(e) = snapshot.field_elevation_ft {
            if !s.is_empty() {
                s.push_str(&format!(" · {:.0} ft", e));
            }
        }
        s
    };

    // Line 1
    if !rwy_info.is_empty() {
        out.push_str(&format!(" ▸ {}  {}  {}\n", phase.to_uppercase(), rwy_info, profiles));
    } else {
        out.push_str(&format!(" ▸ {}{}\n", fmt_left(&phase.to_uppercase(), 24), profiles));
    }

    out.push_str(&format!(" {}\n", SEP));
    out.push_str(&format!("{}{}\n", fmt_right("current", 27), fmt_right("target", 15)));

    let tgt_str = |v: Option<f64>, width: usize, unit: &str| -> String {
        match v {
            Some(val) => format!("{}{}", fmt_right(&format!("{:.0}", val), width), fmt_left(unit, 5)),
            None => format!("{}", fmt_right("—", width)),
        }
    };

    // heading
    out.push_str(&format!(
        "  heading        {}{}{}\n",
        fmt_right(&format!("{:.0}", state.heading_deg), 10),
        fmt_left("°", 8),
        tgt_str(tgt_hdg, 7, "°"),
    ));
    // altitude (AGL)
    out.push_str(&format!(
        "  altitude       {}{}{}\n",
        fmt_right(&format!("{:.0}", state.alt_agl_ft), 10),
        fmt_left(" AGL", 8),
        tgt_str(tgt_alt_agl, 7, " AGL"),
    ));
    // altitude (MSL) — blank label so the instrument name column stays
    // aligned and the eye reads this as the same metric at a different
    // reference.
    out.push_str(&format!(
        "                 {}{}{}\n",
        fmt_right(&format!("{:.0}", state.alt_msl_ft), 10),
        fmt_left(" MSL", 8),
        tgt_str(tgt_alt_msl, 7, " MSL"),
    ));
    // airspeed
    out.push_str(&format!(
        "  airspeed       {}{}{}\n",
        fmt_right(&format!("{:.0}", state.ias_kt), 10),
        fmt_left(" kt IAS", 8),
        tgt_str(tgt_spd, 7, " kt"),
    ));
    // groundspeed
    out.push_str(&format!(
        "  groundspeed    {}{}\n",
        fmt_right(&format!("{:.0}", state.gs_kt), 10),
        fmt_left(" kt", 8),
    ));
    // vertical
    out.push_str(&format!(
        "  vertical       {}{}\n",
        fmt_right(&format!("{:+.0}", state.vs_fpm), 10),
        fmt_left(" fpm", 8),
    ));

    out.push_str(&format!(" {}\n", SEP));

    // config row
    let filled = (commands.throttle * THR_BAR_WIDTH as f64).round() as usize;
    let filled = filled.min(THR_BAR_WIDTH);
    let mut throttle_bar = String::new();
    for _ in 0..filled {
        throttle_bar.push(THR_FILL);
    }
    for _ in filled..THR_BAR_WIDTH {
        throttle_bar.push(THR_EMPTY);
    }

    let flap_str = format!("{}°", state.flap_index);
    let gear_str = if state.gear_down { "dn" } else { "up" };
    let air_str = if state.on_ground { "on ground" } else { "airborne" };

    let rwy_str = match (state.runway_x_ft, state.runway_y_ft) {
        (Some(x), Some(y)) => format!("rwy x{:+.0} y{:+.0}", x, y),
        _ => "rwy —".to_string(),
    };

    // Break `brakes ± pivot_brake` out into left/right so the debug log
    // tells us exactly which wheel the bridge is about to write.
    let brake_left = (commands.brakes - commands.pivot_brake).clamp(0.0, 1.0);
    let brake_right = (commands.brakes + commands.pivot_brake).clamp(0.0, 1.0);
    out.push_str(&format!(
        "  throttle {} {:4.2}   rudder {:+.2}   brake L {:.2} R {:.2}   flaps {:<5}   gear {}   {}   {}",
        throttle_bar,
        commands.throttle,
        commands.rudder,
        brake_left,
        brake_right,
        flap_str,
        gear_str,
        air_str,
        rwy_str,
    ));

    // Per-profile debug lines (taxi leg progress, etc.) emitted after the
    // main status block so they show up in both the TUI and the log.
    for line in &snapshot.debug_lines {
        out.push('\n');
        out.push_str("  ");
        out.push_str(line);
    }

    // Drop a trailing empty line if any
    let trimmed: Vec<&str> = out.lines().collect();
    trimmed.join("\n")
}

// ---------------------------------------------------------------------------
// Interactive ratatui-based TUI
// ---------------------------------------------------------------------------

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
use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph, Wrap};
use ratatui::Terminal;

use crate::bus::{LogEntry, LogKind, SimBus};
use crate::core::mission_manager::PilotCore;
use crate::live_runner::HeartbeatPump;
use crate::llm::conversation::{IncomingMessage, PilotMode};
use crate::llm::responses_client::CacheStats;
use crate::transcribe::{PttController, PttMode, PttSnapshot};

/// Run the interactive TUI on the calling thread. Blocks until the user exits
/// (Ctrl-C / Ctrl-D / Esc); on return, `stop_event` is set so background
/// threads can shut down gracefully.
pub fn run_tui(
    bus: SimBus,
    input_queue: Sender<IncomingMessage>,
    stop_event: Arc<AtomicBool>,
    pilot: Arc<PLMutex<PilotCore>>,
    heartbeat_pump: Option<Arc<HeartbeatPump>>,
    cache_stats: Option<Arc<CacheStats>>,
    ptt: Option<Arc<PttController>>,
) -> Result<()> {
    // Set up terminal.
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    // Steady (non-blinking) block cursor in the input pane. Best-effort —
    // older terminals that don't understand DECSCUSR will just ignore the
    // CSI. Restored to the user's default shape on exit.
    let _ = execute!(stdout, SetCursorStyle::SteadyBlock);
    // Request press/repeat/release events on terminals that implement the
    // Kitty keyboard protocol (Ghostty, WezTerm, kitty, iTerm2 w/ CSI u).
    // `execute!` writes the CSI and returns Ok; we detect actual support by
    // observing a non-`Press` `KeyEventKind` on the first key event.
    let kitty_push_ok = execute!(
        stdout,
        PushKeyboardEnhancementFlags(KeyboardEnhancementFlags::REPORT_EVENT_TYPES)
    )
    .is_ok();
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut input_buffer = String::new();
    // Char offset into `input_buffer` where the next character will be
    // inserted. All editing ops (insert, backspace, arrow keys, C-a/e/k)
    // work relative to this point.
    let mut input_cursor: usize = 0;
    // Submitted lines, most-recent last. Up/Down walks this list.
    let mut history: Vec<String> = Vec::new();
    // Some(i) means the input pane is currently showing history[i]. None
    // means the user is editing a fresh draft (the default state, and what
    // Down-arrow restores when walking past the most-recent entry).
    let mut history_index: Option<usize> = None;
    // Draft saved when the user first presses Up from a fresh line, so
    // Down can restore it once they walk past the end of history.
    let mut pending_draft: String = String::new();
    let mut log_detail = true;
    // None = follow live tail (default). Some(offset) = paused at that
    // absolute scroll offset. Absolute rather than relative-to-bottom so
    // new log lines arriving while scrolled don't move the viewport.
    let mut log_scroll: Option<u16> = None;
    let mut last_log_pin: u16 = 0;
    let mut last_log_page: u16 = 10;
    // Content width of the input pane from the most recent draw. Used by
    // C-a / C-e / C-k to compute which wrap segment the cursor is on.
    let mut last_input_width: u16 = 0;
    let frame_interval = Duration::from_millis(100);
    let mut ptt_tracker = PttKeyTracker::default();
    // EMA-smoothed mic amplitude for the recording waveform. Reset to 0
    // whenever we re-enter Idle so the bar drops cleanly between holds.
    let mut ptt_amp_smoothed: f32 = 0.0;
    // Fixed epoch for driving time-based UI animations (the Finalizing
    // shimmer, etc.) without allocating a new Instant each render.
    let tui_epoch = Instant::now();

    let outcome = loop {
        let next_tick = Instant::now() + frame_interval;

        // Drain any final transcripts into the input buffer.
        if let Some(p) = &ptt {
            while let Some(final_text) = p.try_recv_final() {
                insert_transcript_at_cursor(
                    &mut input_buffer,
                    &mut input_cursor,
                    &final_text,
                );
            }
        }

        // Drain key events until the next render tick.
        loop {
            let timeout = next_tick.saturating_duration_since(Instant::now());
            if timeout.is_zero() {
                break;
            }
            if !event::poll(timeout)? {
                break;
            }
            if let Event::Key(k) = event::read()? {
                // Any non-Press event implies the Kitty enhancement flags
                // took effect, so we can trust true release events.
                if matches!(k.kind, KeyEventKind::Repeat | KeyEventKind::Release) {
                    ptt_tracker.kitty_observed = true;
                }
                // Ignore Release events on keys we don't track for PTT —
                // they'd otherwise fall through to the Char(c) arm below.
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
                        KeyCode::Char('a') => {
                            input_cursor =
                                smart_home(input_cursor, last_input_width);
                            continue;
                        }
                        KeyCode::Char('e') => {
                            input_cursor = smart_end(
                                input_cursor,
                                &input_buffer,
                                last_input_width,
                            );
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

                // PTT key handling — space or tab held with no modifiers.
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

                // Any non-PTT key dismisses a pending tap so we don't
                // retroactively treat it as a PTT hold.
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
                        // If PTT is mid-session, stop it so the final
                        // transcript lands before we submit.
                        if let Some(p) = &ptt {
                            if p.snapshot().mode != PttMode::Idle {
                                p.stop();
                                // Give the worker up to 300 ms to deliver
                                // the final.
                                let deadline = Instant::now()
                                    + Duration::from_millis(300);
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
                        if input_cursor > 0 {
                            input_cursor -= 1;
                        }
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
        // Repeat-gap release detection. Runs regardless of Kitty support —
        // some terminals (Ghostty observed) emit Press/Repeat but not
        // Release, so the explicit Release handler alone is not enough.
        // When a real Release does arrive, it fires first and clears the
        // pending state before this timeout trips.
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

        let status_fragments = {
            let snapshot = pilot.lock().latest_snapshot.clone();
            render_status_fragments(snapshot.as_ref(), cache_stats.as_deref())
        };
        let log_entries = bus.log_entries();
        let (_, _, radio) = bus.snapshot();
        let radio_lines: Vec<Line> = radio.iter().flat_map(|l| render_radio_line(l)).collect();

        terminal.draw(|f| {
            let area = f.area();
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Length(13), // status
                    Constraint::Min(3),     // log (scrolling)
                    Constraint::Length(8),  // radio
                    Constraint::Length(5),  // input (3 content lines + borders)
                ])
                .split(area);

            // Inside the draw closure: operator-row bg tint needs the
            // pane's live inner width to pad out to the border.
            let log_inner_width = chunks[1].width.saturating_sub(2);
            let mut log_lines = build_log_lines(&log_entries, log_detail, log_inner_width);
            if bus.is_llm_busy() {
                log_lines.push(render_thinking_line(tui_epoch.elapsed()));
            }

            let status_paragraph = Paragraph::new(status_fragments.clone())
                .block(Block::default().borders(Borders::ALL).title(" FLIGHT "))
                .wrap(Wrap { trim: false });
            f.render_widget(status_paragraph, chunks[0]);

            // Render both scrollback panes with `scroll((offset, 0))` pinned
            // to the bottom. Picking "last N logical lines" doesn't work
            // when lines wrap — long log entries become multiple screen rows
            // and spill below the pane border. Using `line_count(inner_width)`
            // gives us the true rendered row count so we can skip the right
            // amount from the top.
            let log_pin = pin_to_bottom(&log_lines, chunks[1]);
            let inner_height = chunks[1].height.saturating_sub(2);
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
            let base_title = if log_detail { " LOG " } else { " LOG (compact) " };
            let log_title = if scroll_indicator.is_empty() {
                base_title.to_string()
            } else {
                format!("{}{} ", base_title.trim_end(), scroll_indicator)
            };
            let log_paragraph = Paragraph::new(log_lines.clone())
                .block(Block::default().borders(Borders::ALL).title(log_title))
                .wrap(Wrap { trim: false });
            f.render_widget(
                log_paragraph.scroll((log_offset, 0)),
                chunks[1],
            );

            let radio_paragraph = Paragraph::new(radio_lines.clone())
                .block(Block::default().borders(Borders::ALL).title(" RADIO "))
                .wrap(Wrap { trim: false });
            f.render_widget(
                radio_paragraph.scroll((pin_to_bottom(&radio_lines, chunks[2]), 0)),
                chunks[2],
            );

            let input_title =
                " INPUT — space/tab hold to talk · enter to send · ↑/↓ history · c-a/e/k edit · ctrl-t log detail · pgup/pgdn/end scroll log ";
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
            // "keep holding…" warmup hint: PTT key is pressed but the
            // auto-repeat / kitty Repeat hasn't fired yet, so recording
            // hasn't started. Threshold keeps single-char taps clean.
            let warmup_hint = ptt_tracker
                .pending
                .as_ref()
                .filter(|p| !p.recording)
                .filter(|p| p.last_seen.elapsed() >= Duration::from_millis(120))
                .is_some();
            // Smooth the amp towards the live reading while recording,
            // and decay back to silence between holds so the next hold
            // starts from zero.
            if recording {
                if let Some(snap) = &ptt_snap {
                    ptt_amp_smoothed = smooth_amp(ptt_amp_smoothed, snap.amp);
                }
            } else {
                ptt_amp_smoothed *= 0.3;
            }
            if let Some(snap) = &ptt_snap {
                if snap.mode != PttMode::Idle && !snap.partial.is_empty() {
                    // Show prefix + partial dimmed while streaming.
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
                    // Amp glyph lives at the end of the line and acts as
                    // the cursor during PTT — no separator space, and the
                    // terminal cursor is suppressed below so this is the
                    // only thing the eye lands on.
                    let level = amp_level_from_smoothed(ptt_amp_smoothed);
                    spans.push(Span::styled(
                        amp_glyph(level).to_string(),
                        Style::default().fg(Color::LightGreen),
                    ));
                } else if finalizing {
                    // Pulsing ellipsis while we're waiting for the final
                    // transcript. 1.6 s period (half-sine → visible) so
                    // it's a gentle breath rather than a strobe.
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
            let pane = chunks[3];
            let content_width = pane.width.saturating_sub(2).max(1);
            let content_height = pane.height.saturating_sub(2).max(1);
            last_input_width = content_width;
            // Pre-split into visual rows on hard char boundaries so the
            // rendered glyphs line up with the cursor math below (ratatui's
            // `Wrap` is word-wrap, which disagrees with char-based cursor
            // positioning the moment the buffer contains spaces).
            let input_lines = wrap_input_spans(spans, content_width);
            let input_paragraph = Paragraph::new(input_lines)
                .block(Block::default().borders(Borders::ALL).title(input_title));
            f.render_widget(input_paragraph, chunks[3]);

            if !recording {
                // Park the terminal cursor at `input_cursor` (not the end
                // of the buffer) so emacs-style editing (C-a / C-e / C-k
                // / Left / Right) shows visibly. Prompt "> " is two
                // columns; input_buffer is ASCII in practice so char
                // count is the visible width. Wrap into subsequent
                // content rows if the line overflows; clamp to the last
                // content row so we never escape the pane. Skipped
                // during PTT recording so the animated amp-glyph is the
                // sole visual "cursor".
                let consumed = 2u16.saturating_add(input_cursor as u16);
                let row = (consumed / content_width).min(content_height.saturating_sub(1));
                let col = consumed % content_width;
                f.set_cursor_position((pane.x + 1 + col, pane.y + 1 + row));
            }
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

/// Format the active-profile list for the mode line, appending each
/// profile's optional `mode_line_suffix` in parentheses so the LLM
/// and operator can see at a glance what the profile is actually
/// doing (e.g. `taxi  (via A→B · leg 3/8 · 80ft)`). Suffix length
/// changes every tick — that's fine here because this string never
/// feeds back into the heartbeat diff.
fn decorate_profiles(names: &[String], suffixes: &[Option<String>]) -> String {
    names
        .iter()
        .enumerate()
        .map(|(i, name)| match suffixes.get(i).and_then(|s| s.as_deref()) {
            Some(s) if !s.is_empty() => format!("{name}  ({s})"),
            _ => name.clone(),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// Splice a finalized transcript into `buffer` at the caret so voice
/// input lands where the user is looking, not tacked onto the end of
/// the line. Adds an auto-space before/after only when the adjacent
/// characters aren't already whitespace. `cursor` is advanced to sit
/// immediately after the inserted text.
fn insert_transcript_at_cursor(
    buffer: &mut String,
    cursor: &mut usize,
    transcript: &str,
) {
    let trimmed = transcript.trim();
    if trimmed.is_empty() {
        return;
    }
    let byte_idx = char_byte_index(buffer, *cursor);
    let needs_leading_space = *cursor > 0 && {
        let prev_byte = char_byte_index(buffer, *cursor - 1);
        !buffer[prev_byte..]
            .chars()
            .next()
            .map(|c| c.is_whitespace())
            .unwrap_or(false)
    };
    let needs_trailing_space = byte_idx < buffer.len() && {
        !buffer[byte_idx..]
            .chars()
            .next()
            .map(|c| c.is_whitespace())
            .unwrap_or(false)
    };
    let mut insert = String::new();
    if needs_leading_space {
        insert.push(' ');
    }
    insert.push_str(trimmed);
    if needs_trailing_space {
        insert.push(' ');
    }
    let inserted_chars = insert.chars().count();
    buffer.insert_str(byte_idx, &insert);
    *cursor += inserted_chars;
}

/// Glyphs for the mic amplitude indicator, from silence to peak. The
/// bottom two rungs (space and light-shade) render as near-empty so
/// true silence shows as a quiet cell instead of a floor bar.
const AMP_GLYPHS: [char; 9] = [' ', '░', '▁', '▂', '▃', '▄', '▅', '▆', '█'];

fn amp_glyph(level: u8) -> char {
    AMP_GLYPHS[level.min(8) as usize]
}

/// EMA-smooth the raw 0..=8 amp bucket so the rendered bar doesn't
/// strobe at cpal's callback rate. `prev` is last render's smoothed
/// value; `raw` is the newest reading.
fn smooth_amp(prev: f32, raw: u8) -> f32 {
    const EMA_NEW_WEIGHT: f32 = 0.7;
    prev * (1.0 - EMA_NEW_WEIGHT) + (raw as f32) * EMA_NEW_WEIGHT
}

/// Map the smoothed float level back into the 0..=8 glyph index.
fn amp_level_from_smoothed(smoothed: f32) -> u8 {
    smoothed.round().clamp(0.0, 8.0) as u8
}

/// PTT key tracker. Lives on the TUI thread and translates raw key events
/// into calls to the `PttController`. Handles both the Kitty-protocol path
/// (real Press/Repeat/Release events) and the repeat-gap fallback.
#[derive(Default)]
struct PttKeyTracker {
    pending: Option<PendingPtt>,
    /// Set once we observe any non-Press KeyEventKind, which means the
    /// Kitty keyboard flags took effect. Until then we fall back to a
    /// repeat-gap timeout in the main loop.
    kitty_observed: bool,
}

struct PendingPtt {
    key: PttKey,
    /// True once we've confirmed this is a hold (not a single tap) and
    /// asked the `PttController` to start recording.
    recording: bool,
    last_seen: Instant,
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum PttKey {
    Space,
    Tab,
}

impl PttKey {
    fn prefix(self) -> &'static str {
        match self {
            PttKey::Space => "",
            PttKey::Tab => "[atc] ",
        }
    }

    fn atc_mode(self) -> bool {
        matches!(self, PttKey::Tab)
    }

    fn from_code(code: &KeyCode) -> Option<Self> {
        match code {
            KeyCode::Char(' ') => Some(PttKey::Space),
            KeyCode::Tab => Some(PttKey::Tab),
            _ => None,
        }
    }
}

/// Minimal interface the TUI needs from a PTT engine — keeping
/// `handle_ptt_key` decoupled from `PttController` makes it trivially
/// unit-testable with a stub.
trait PttControl {
    fn start(&self, prefix: String, atc_mode: bool);
    fn stop(&self);
}

impl PttControl for PttController {
    fn start(&self, prefix: String, atc_mode: bool) {
        PttController::start(self, prefix, atc_mode);
    }
    fn stop(&self) {
        PttController::stop(self);
    }
}

fn handle_ptt_key<P: PttControl + ?Sized>(
    kind: &KeyEventKind,
    code: &KeyCode,
    tracker: &mut PttKeyTracker,
    input_buffer: &mut String,
    input_cursor: &mut usize,
    ptt: Option<&P>,
) {
    let key = match PttKey::from_code(code) {
        Some(k) => k,
        None => return,
    };
    // Terminals without Kitty keyboard enhancement send auto-repeat as a
    // stream of `Press` events, not `Repeat`. Treat a `Press` of the same
    // key we're already tracking as a synthetic Repeat so the fallback
    // path fires without waiting for a real Repeat event.
    let effective = match kind {
        KeyEventKind::Press
            if matches!(&tracker.pending, Some(p) if p.key == key) =>
        {
            KeyEventKind::Repeat
        }
        other => *other,
    };
    match effective {
        KeyEventKind::Press => {
            // Insert the char at the cursor (not at end); we'll strip it
            // on first Repeat if it turns out to be a PTT hold. For a
            // bare tap the char stays where the user expected it.
            let ch = match key {
                PttKey::Space => ' ',
                PttKey::Tab => '\t',
            };
            let byte_idx = char_byte_index(input_buffer, *input_cursor);
            input_buffer.insert(byte_idx, ch);
            *input_cursor += 1;
            tracker.pending = Some(PendingPtt {
                key,
                recording: false,
                last_seen: Instant::now(),
            });
        }
        KeyEventKind::Repeat => {
            let should_start = matches!(
                &tracker.pending,
                Some(p) if p.key == key && !p.recording
            );
            if should_start {
                // Strip the stray character inserted on Press (at
                // cursor-1, since Press advanced the cursor past it).
                if *input_cursor > 0 {
                    let prev_byte = char_byte_index(input_buffer, *input_cursor - 1);
                    if matches!(input_buffer[prev_byte..].chars().next(), Some(' ') | Some('\t')) {
                        input_buffer.remove(prev_byte);
                        *input_cursor -= 1;
                    }
                }
                if !key.prefix().is_empty() {
                    // The [atc] prefix only parses correctly at the start
                    // of the submitted line, so insert it at the head of
                    // the buffer rather than at the cursor. Any in-progress
                    // typed text is preserved after it.
                    input_buffer.insert_str(0, key.prefix());
                    *input_cursor += key.prefix().chars().count();
                }
                if let Some(p) = ptt {
                    p.start(key.prefix().to_string(), key.atc_mode());
                }
                tracker.pending = Some(PendingPtt {
                    key,
                    recording: true,
                    last_seen: Instant::now(),
                });
            } else if let Some(p) = tracker.pending.as_mut() {
                if p.key == key {
                    p.last_seen = Instant::now();
                }
            }
        }
        KeyEventKind::Release => {
            let matches_pending = matches!(
                &tracker.pending,
                Some(p) if p.key == key && p.recording
            );
            if matches_pending {
                if let Some(p) = ptt {
                    p.stop();
                }
            }
            tracker.pending = None;
        }
    }
}

fn break_outer(stop: &AtomicBool) {
    stop.store(true, Ordering::Release);
}

/// Convert a char index into a byte index inside `s`. Clamps to `s.len()`
/// when the char index is beyond the end.
fn char_byte_index(s: &str, char_idx: usize) -> usize {
    s.char_indices().nth(char_idx).map(|(i, _)| i).unwrap_or(s.len())
}

/// The "> " prompt that precedes the input buffer in the INPUT pane. Every
/// editing helper needs to account for it since it shifts everything on
/// the first visual row two columns to the right.
const PROMPT_COLS: usize = 2;

/// Split the INPUT-pane spans into one `Line` per visual row, wrapping on
/// hard character boundaries that match the cursor math (not on word
/// boundaries the way `Wrap { trim: false }` would). Row capacity is
/// `content_width` cells across the board — the 2-col prompt is the first
/// span in `spans`, so it naturally consumes 2 cells of row 0. Per-span
/// `Style` is preserved on both sides of any split. Always returns at
/// least one `Line`.
fn wrap_input_spans(spans: Vec<Span<'_>>, content_width: u16) -> Vec<Line<'static>> {
    let cap = (content_width as usize).max(1);
    let mut lines: Vec<Line<'static>> = Vec::new();
    let mut current: Vec<Span<'static>> = Vec::new();
    let mut used: usize = 0;
    for span in spans {
        let style = span.style;
        let mut remaining: String = span.content.into_owned();
        while !remaining.is_empty() {
            let room = cap.saturating_sub(used);
            if room == 0 {
                lines.push(Line::from(std::mem::take(&mut current)));
                used = 0;
                continue;
            }
            let char_count = remaining.chars().count();
            if char_count <= room {
                used += char_count;
                current.push(Span::styled(remaining, style));
                break;
            }
            let split_idx = remaining
                .char_indices()
                .nth(room)
                .map(|(i, _)| i)
                .unwrap_or(remaining.len());
            let right: String = remaining[split_idx..].to_string();
            remaining.truncate(split_idx);
            current.push(Span::styled(remaining, style));
            lines.push(Line::from(std::mem::take(&mut current)));
            used = 0;
            remaining = right;
        }
    }
    if !current.is_empty() || lines.is_empty() {
        lines.push(Line::from(current));
    }
    lines
}

/// Return the visual wrap row that `cursor` (a char index) is on given
/// `width` content columns. Treats the buffer as single-width chars.
fn cursor_row(cursor: usize, width: u16) -> (usize, usize) {
    let w = (width as usize).max(1);
    let disp_col = PROMPT_COLS + cursor;
    (disp_col / w, w)
}

/// Char index of the first buffer char on visual row `row`, accounting
/// for the prompt columns consumed on row 0.
fn row_start_char(row: usize, width: u16) -> usize {
    let w = (width as usize).max(1);
    if row == 0 { 0 } else { row * w - PROMPT_COLS }
}

/// Char index one past the last char on visual row `row`, clamped to the
/// total char count of the buffer.
fn row_end_char(row: usize, width: u16, total: usize) -> usize {
    let w = (width as usize).max(1);
    ((row + 1) * w)
        .checked_sub(PROMPT_COLS)
        .unwrap_or(0)
        .min(total)
}

/// C-a: "smart home". Move the cursor to the start of the current visual
/// row; if already at the start of a row, jump up to the start of the
/// previous row. Bottoms out at char 0. Mirrors the behavior Claude Code
/// uses in its prompt input.
fn smart_home(cursor: usize, width: u16) -> usize {
    let (row, _) = cursor_row(cursor, width);
    let row_start = row_start_char(row, width);
    if cursor == row_start {
        if row == 0 { 0 } else { row_start_char(row - 1, width) }
    } else {
        row_start
    }
}

/// C-e: "smart end". Move the cursor to the end of the current visual
/// row; if already at the end of a row with more content below, jump to
/// the end of the next row. Saturates at the buffer's last char.
fn smart_end(cursor: usize, buffer: &str, width: u16) -> usize {
    let total = buffer.chars().count();
    let (row, _) = cursor_row(cursor, width);
    let row_end = row_end_char(row, width, total);
    if cursor == row_end && row_end < total {
        row_end_char(row + 1, width, total)
    } else {
        row_end
    }
}

/// C-k: kill from the cursor to the end of the current visual row. Mutates
/// `buffer` in place; the cursor position is unchanged (it still points at
/// what used to be `cursor` — the first char after the killed range).
fn kill_to_row_end(buffer: &mut String, cursor: usize, width: u16) {
    let total = buffer.chars().count();
    let (row, _) = cursor_row(cursor, width);
    let row_end = row_end_char(row, width, total);
    if cursor < row_end {
        let start_byte = char_byte_index(buffer, cursor);
        let end_byte = char_byte_index(buffer, row_end);
        buffer.drain(start_byte..end_byte);
    }
}

/// Cap on the number of retained history entries. Oldest are dropped when
/// the cap is exceeded.
const INPUT_HISTORY_MAX: usize = 200;

/// Record a submitted entry in the input history. Consecutive duplicates
/// are coalesced (bash-style), and the oldest entries are dropped once
/// `max_len` is exceeded.
fn history_record(history: &mut Vec<String>, entry: &str, max_len: usize) {
    if history.last().map(|s| s.as_str()) == Some(entry) {
        return;
    }
    history.push(entry.to_string());
    while history.len() > max_len {
        history.remove(0);
    }
}

/// Up-arrow: walk backward through history. If this is the first step
/// back (history_index is None), the current buffer is stashed into
/// `pending_draft` so Down-arrow can restore it later. Saturates at the
/// oldest entry.
fn history_go_back(
    history: &[String],
    input_buffer: &mut String,
    input_cursor: &mut usize,
    history_index: &mut Option<usize>,
    pending_draft: &mut String,
) {
    if history.is_empty() {
        return;
    }
    let new_index = match *history_index {
        None => {
            *pending_draft = std::mem::take(input_buffer);
            history.len() - 1
        }
        Some(0) => 0,
        Some(i) => i - 1,
    };
    *history_index = Some(new_index);
    *input_buffer = history[new_index].clone();
    *input_cursor = input_buffer.chars().count();
}

/// Down-arrow: walk forward through history. If the cursor was past the
/// most-recent entry, the in-progress `pending_draft` is restored and
/// history_index is cleared. No-op when not browsing history.
fn history_go_forward(
    history: &[String],
    input_buffer: &mut String,
    input_cursor: &mut usize,
    history_index: &mut Option<usize>,
    pending_draft: &mut String,
) {
    let Some(i) = *history_index else {
        return;
    };
    if i + 1 < history.len() {
        let ni = i + 1;
        *history_index = Some(ni);
        *input_buffer = history[ni].clone();
    } else {
        *history_index = None;
        *input_buffer = std::mem::take(pending_draft);
    }
    *input_cursor = input_buffer.chars().count();
}

/// Most kinds render as a colored left-gutter bar with default body
/// text. Errors keep a sigil + red-bold body so they still cut through
/// a stream of bar-styled entries.
enum LogStyle {
    Bar {
        bar_style: Style,
        body_style: Style,
    },
    Sigil {
        sigil: &'static str,
        sigil_style: Style,
        body_style: Style,
    },
}

const LOG_BAR: &str = "▎";

fn style_for_kind(kind: LogKind) -> LogStyle {
    match kind {
        LogKind::Operator => LogStyle::Bar {
            bar_style: Style::default().fg(Color::LightMagenta),
            body_style: Style::default(),
        },
        LogKind::Llm => LogStyle::Bar {
            bar_style: Style::default().fg(Color::LightGreen),
            body_style: Style::default(),
        },
        LogKind::Safety => LogStyle::Bar {
            bar_style: Style::default().fg(Color::Yellow),
            body_style: Style::default(),
        },
        LogKind::Error => LogStyle::Sigil {
            sigil: "!",
            sigil_style: Style::default()
                .fg(Color::LightRed)
                .add_modifier(Modifier::BOLD),
            body_style: Style::default()
                .fg(Color::LightRed)
                .add_modifier(Modifier::BOLD),
        },
        LogKind::Mode => LogStyle::Bar {
            bar_style: Style::default().fg(Color::Gray),
            body_style: Style::default().fg(Color::Gray),
        },
        LogKind::System => LogStyle::Bar {
            bar_style: Style::default().fg(Color::Gray),
            body_style: Style::default().fg(Color::Gray),
        },
        LogKind::ToolCall | LogKind::Tokens | LogKind::Heartbeat | LogKind::Voice => {
            LogStyle::Bar {
                bar_style: Style::default().fg(Color::DarkGray),
                body_style: Style::default().fg(Color::DarkGray),
            }
        }
    }
}

/// Slightly-lighter-than-black tint for operator blocks — reads like a
/// chat "user bubble" against the default terminal bg.
const OPERATOR_BG: Color = Color::Rgb(32, 32, 38);

/// Filter, insert a blank line at each operator↔llm turn, and tint
/// operator lines to the pane's inner width so the bg extends past the
/// text itself.
fn build_log_lines(
    entries: &[LogEntry],
    show_verbose: bool,
    inner_width: u16,
) -> Vec<Line<'static>> {
    let mut out: Vec<Line<'static>> = Vec::new();
    let mut prev_kind: Option<LogKind> = None;
    for entry in entries.iter().filter(|e| show_verbose || !e.kind.is_verbose()) {
        if prev_kind.is_some_and(|p| is_author_turn_boundary(p, entry.kind)) {
            out.push(Line::default());
        }
        let is_operator = entry.kind == LogKind::Operator;
        for line in render_log_entry(entry) {
            out.push(if is_operator {
                tint_line_bg(line, OPERATOR_BG, inner_width)
            } else {
                line
            });
        }
        prev_kind = Some(entry.kind);
    }
    out
}

fn is_author_turn_boundary(prev: LogKind, cur: LogKind) -> bool {
    matches!(
        (prev, cur),
        (LogKind::Operator, LogKind::Llm) | (LogKind::Llm, LogKind::Operator)
    )
}

/// Apply `bg` to every span on the line in place and pad the row out to
/// `inner_width` columns with a trailing bg-only space. Content wider
/// than the pane wraps onto an un-tinted continuation row — ratatui
/// wraps after styling, and manually rendering the wrap here would fight
/// the Paragraph widget.
fn tint_line_bg(mut line: Line<'static>, bg: Color, inner_width: u16) -> Line<'static> {
    let used = line.width();
    for span in line.spans.iter_mut() {
        span.style = span.style.bg(bg);
    }
    let width = inner_width as usize;
    if width > used {
        line.spans.push(Span::styled(
            " ".repeat(width - used),
            Style::default().bg(bg),
        ));
    }
    line
}

fn render_log_entry(entry: &LogEntry) -> Vec<Line<'static>> {
    // Defensive: a System entry whose body embeds "error" gets upgraded
    // to the Error style — catches any lingering raw `push_log("error:
    // …")` that wasn't migrated to push_log_kind.
    let style = if entry.kind == LogKind::System
        && contains_ascii_ignore_case(&entry.text, "error")
    {
        style_for_kind(LogKind::Error)
    } else {
        style_for_kind(entry.kind)
    };

    let segments = split_and_normalize(&entry.text);
    if segments.is_empty() {
        return Vec::new();
    }

    segments
        .into_iter()
        .enumerate()
        .map(|(i, segment)| {
            let mut spans: Vec<Span<'static>> = Vec::with_capacity(3);
            match &style {
                LogStyle::Bar { bar_style, body_style } => {
                    spans.push(Span::styled(LOG_BAR, *bar_style));
                    spans.push(Span::raw(" "));
                    spans.push(Span::styled(segment, *body_style));
                }
                LogStyle::Sigil { sigil, sigil_style, body_style } => {
                    if i == 0 {
                        spans.push(Span::styled(*sigil, *sigil_style));
                        spans.push(Span::raw(" "));
                    } else {
                        spans.push(Span::raw("  "));
                    }
                    spans.push(Span::styled(segment, *body_style));
                }
            }
            Line::from(spans)
        })
        .collect()
}

/// Classification of a line in the RADIO pane. Drives per-source coloring so
/// the operator can tell at a glance who's talking.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RadioSource {
    /// Outbound pilot transmission from the `broadcast_on_radio` tool —
    /// rendered as `[COM1 118.350] <message>`.
    Pilot,
    /// Inbound ATC / other-traffic traffic surfaced via the `[atc] ` prefix
    /// (operator's Tab-PTT, `--atc-message`, or typed `atc:` lines).
    Atc,
    /// System / bridge notes like tuning confirmations — no human speaker.
    System,
}

fn classify_radio_line(line: &str) -> RadioSource {
    // Byte-level match — avoids per-frame String allocation from
    // `to_ascii_lowercase` across up to RADIO_MAX_LINES on every redraw.
    let bytes = line.as_bytes();
    if bytes.first() != Some(&b'[') {
        return RadioSource::System;
    }
    let Some(close) = bytes.iter().position(|&b| b == b']') else {
        return RadioSource::System;
    };
    let tag = &bytes[1..close];
    if tag.len() >= 3 && tag[..3].eq_ignore_ascii_case(b"atc") {
        return RadioSource::Atc;
    }
    if tag.len() >= 3 && tag[..3].eq_ignore_ascii_case(b"com") {
        // Tuning/status notes embed the whole sentence inside the brackets
        // (`[COM1 tuned to 118.350]`) and have no body; a real pilot
        // transmission always has message text after `]`.
        if line[close + 1..].trim().is_empty() {
            return RadioSource::System;
        }
        return RadioSource::Pilot;
    }
    RadioSource::System
}

fn radio_color(source: RadioSource) -> Color {
    match source {
        RadioSource::Pilot => Color::LightCyan,
        RadioSource::Atc => Color::Yellow,
        RadioSource::System => Color::DarkGray,
    }
}

/// Render one RADIO entry as one-or-more styled `Line`s. The leading
/// `[...]` tag is split off and rendered bold so the source reads at a
/// glance; the body carries the same hue, un-bolded.
fn render_radio_line(line: &str) -> Vec<Line<'static>> {
    let color = radio_color(classify_radio_line(line));
    let tag_style = Style::default().fg(color).add_modifier(Modifier::BOLD);
    let body_style = Style::default().fg(color);
    let segments = split_and_normalize(line);
    segments
        .into_iter()
        .enumerate()
        .map(|(i, mut segment)| {
            if i == 0 {
                if let Some(split_at) = leading_bracket_tag_end(&segment) {
                    let rest = segment.split_off(split_at);
                    let mut spans: Vec<Span<'static>> = Vec::with_capacity(2);
                    spans.push(Span::styled(segment, tag_style));
                    if !rest.is_empty() {
                        spans.push(Span::styled(rest, body_style));
                    }
                    return Line::from(spans);
                }
            }
            Line::from(Span::styled(segment, body_style))
        })
        .collect()
}

/// Byte index just past the leading `[...]` tag (plus one space separator
/// if present), suitable for `String::split_off`. `None` if the line
/// doesn't start with `[` or has no closing bracket.
fn leading_bracket_tag_end(line: &str) -> Option<usize> {
    if !line.starts_with('[') {
        return None;
    }
    let bytes = line.as_bytes();
    let close = bytes.iter().position(|&b| b == b']')?;
    let mut split_at = close + 1;
    if bytes.get(split_at) == Some(&b' ') {
        split_at += 1;
    }
    Some(split_at)
}

/// Compute a `Paragraph::scroll` y-offset that keeps the most recent line at
/// the bottom of `area`. Accounts for line wrapping by using the same
/// `line_count(inner_width)` ratatui itself uses when rendering, so the pane
/// doesn't overflow its border when long sql-query results wrap onto multiple
/// screen rows.
fn pin_to_bottom(lines: &[Line<'static>], area: ratatui::layout::Rect) -> u16 {
    let inner_width = area.width.saturating_sub(2);
    let inner_height = area.height.saturating_sub(2) as usize;
    if inner_width == 0 || inner_height == 0 {
        return 0;
    }
    let probe = Paragraph::new(lines.to_vec()).wrap(Wrap { trim: false });
    let total = probe.line_count(inner_width);
    total.saturating_sub(inner_height) as u16
}

/// Split a bus entry on newlines and expand tabs to 4-space stops so
/// ratatui's unicode-width-based layout doesn't render `\t` as a single
/// column. DuckDB's `sql_query` tool returns tab-separated rows; tool
/// outputs like `takeoff_checklist` have embedded newlines.
fn split_and_normalize(text: &str) -> Vec<String> {
    text.split('\n').map(expand_tabs).collect()
}

fn expand_tabs(line: &str) -> String {
    if !line.contains('\t') {
        return line.to_string();
    }
    let tab_width = 4;
    let mut out = String::with_capacity(line.len());
    let mut col = 0usize;
    for c in line.chars() {
        if c == '\t' {
            let spaces = tab_width - (col % tab_width);
            for _ in 0..spaces {
                out.push(' ');
                col += 1;
            }
        } else {
            out.push(c);
            col += 1;
        }
    }
    out
}

fn contains_ascii_ignore_case(haystack: &str, needle: &str) -> bool {
    let h = haystack.as_bytes();
    let n = needle.as_bytes();
    h.len() >= n.len() && h.windows(n.len()).any(|w| w.eq_ignore_ascii_case(n))
}

const PROP_FRAMES: [char; 4] = ['│', '╱', '─', '╲'];
const PROP_FRAME_MS: u128 = 100;

/// Mirrors the `<bar> <body>` shape of bar-styled log entries so it
/// reads as a transient extra row rather than chrome.
fn render_thinking_line(elapsed: Duration) -> Line<'static> {
    Line::from(vec![
        Span::styled(
            PROP_FRAMES[prop_frame_index(elapsed)].to_string(),
            Style::default()
                .fg(Color::LightCyan)
                .add_modifier(Modifier::BOLD),
        ),
        Span::raw(" "),
        Span::styled("thinking…", Style::default().fg(Color::DarkGray)),
    ])
}

fn prop_frame_index(elapsed: Duration) -> usize {
    (elapsed.as_millis() / PROP_FRAME_MS) as usize % PROP_FRAMES.len()
}

/// Produce styled `Line`s for the status pane. Reimplements the lexer logic
/// from `format_snapshot_display` with color hints for deviation bands.
fn render_status_fragments(
    snapshot: Option<&StatusSnapshot>,
    cache_stats: Option<&CacheStats>,
) -> Vec<Line<'static>> {
    let Some(snap) = snapshot else {
        return vec![Line::from(Span::styled(
            "  (waiting for first pilot tick)".to_string(),
            Style::default().fg(Color::DarkGray),
        ))];
    };

    let state = &snap.state;
    let guidance = snap.last_guidance.as_ref();
    let commands = &snap.last_commands;
    let profiles = if snap.active_profiles.is_empty() {
        "(none)".to_string()
    } else {
        decorate_profiles(&snap.active_profiles, &snap.profile_mode_line_suffixes)
    };
    let phase_label = snap
        .phase
        .map(|p| p.value().to_string())
        .unwrap_or_else(|| "—".to_string());

    let tgt_hdg = guidance.and_then(|g| g.target_heading_deg.or(g.target_track_deg));
    let tgt_alt_msl = guidance.and_then(|g| g.target_altitude_ft);
    let tgt_spd = guidance.and_then(|g| g.target_speed_kt);
    let field_elev = state.alt_msl_ft - state.alt_agl_ft;
    let tgt_alt_agl = tgt_alt_msl.map(|a| a - field_elev);

    let phase_style = phase_style(snap.phase);

    let mut lines: Vec<Line> = Vec::with_capacity(10);

    // Header line (phase + rwy info + profiles).
    // Python pads phase to 24 chars only when there's no rwy_info, and
    // separates phase/rwy/profiles with explicit two-space spans.
    let mut rwy_parts: Vec<String> = Vec::new();
    if let Some(a) = &snap.airport_ident {
        rwy_parts.push(a.clone());
    }
    if let Some(r) = &snap.runway_id {
        rwy_parts.push(format!("rwy {}", r));
    }
    if let Some(e) = snap.field_elevation_ft {
        if !rwy_parts.is_empty() {
            rwy_parts.push(format!("· {:.0} ft", e));
        }
    }
    let has_rwy = !rwy_parts.is_empty();
    let mut header: Vec<Span<'static>> = vec![
        Span::styled(" ▸ ", Style::default().fg(Color::LightGreen)),
    ];
    if has_rwy {
        header.push(Span::styled(phase_label.to_uppercase(), phase_style));
        header.push(Span::raw("  "));
        header.push(Span::styled(rwy_parts.join(" "), Style::default().fg(Color::Gray)));
        header.push(Span::raw("  "));
    } else {
        header.push(Span::styled(
            format!("{:<24}", phase_label.to_uppercase()),
            phase_style,
        ));
    }
    header.push(Span::styled(profiles, Style::default().fg(Color::Gray)));
    lines.push(Line::from(header));
    lines.push(Line::from(Span::styled(
        " ".to_string() + &"─".repeat(64),
        Style::default().fg(Color::DarkGray),
    )));
    lines.push(Line::from(Span::styled(
        format!("{:>27}{:>15}", "current", "target"),
        Style::default().fg(Color::Gray),
    )));

    // heading
    let val_style = Style::default().fg(Color::White).add_modifier(Modifier::BOLD);
    let hdg_style = match tgt_hdg {
        Some(t) => dev_style((state.heading_deg - t).abs(), 3.0, 15.0),
        None => val_style,
    };
    lines.push(instrument_line(
        "heading",
        format!("{:>10.0}", state.heading_deg),
        "°",
        tgt_hdg.map(|v| format!("{:>7.0}", v)),
        "°",
        hdg_style,
    ));
    // altitude AGL
    let alt_style = match tgt_alt_agl {
        Some(t) => dev_style((state.alt_agl_ft - t).abs(), 50.0, 200.0),
        None => val_style,
    };
    lines.push(instrument_line(
        "altitude",
        format!("{:>10.0}", state.alt_agl_ft),
        " AGL",
        tgt_alt_agl.map(|v| format!("{:>7.0}", v)),
        " AGL",
        alt_style,
    ));
    // altitude MSL
    let msl_style = match tgt_alt_msl {
        Some(t) => dev_style((state.alt_msl_ft - t).abs(), 50.0, 200.0),
        None => val_style,
    };
    lines.push(instrument_line(
        "",
        format!("{:>10.0}", state.alt_msl_ft),
        " MSL",
        tgt_alt_msl.map(|v| format!("{:>7.0}", v)),
        " MSL",
        msl_style,
    ));
    // airspeed
    let spd_style = match tgt_spd {
        Some(t) => dev_style((state.ias_kt - t).abs(), 3.0, 10.0),
        None => val_style,
    };
    lines.push(instrument_line(
        "airspeed",
        format!("{:>10.0}", state.ias_kt),
        " kt IAS",
        tgt_spd.map(|v| format!("{:>7.0}", v)),
        " kt",
        spd_style,
    ));
    lines.push(instrument_line(
        "groundspeed",
        format!("{:>10.0}", state.gs_kt),
        " kt",
        None,
        "",
        val_style,
    ));
    lines.push(instrument_line(
        "vertical",
        format!("{:>+10.0}", state.vs_fpm),
        " fpm",
        None,
        "",
        val_style,
    ));
    lines.push(Line::from(Span::styled(
        " ".to_string() + &"─".repeat(64),
        Style::default().fg(Color::DarkGray),
    )));

    // Bottom config row. Label spans use the shared lbl style
    // (Gray); numeric values use val (White + BOLD). Gap strings match
    // the Python layout ("   flaps " / "   gear " — three spaces).
    let lbl_style = Style::default().fg(Color::Gray);
    let thr_width = 8usize;
    let filled = (commands.throttle * thr_width as f64).round() as usize;
    let filled = filled.min(thr_width);
    let mut config_row: Vec<Span<'static>> = Vec::new();
    config_row.push(Span::styled("  throttle ", lbl_style));
    config_row.push(Span::styled(
        "█".repeat(filled),
        Style::default().fg(Color::LightGreen),
    ));
    config_row.push(Span::styled(
        "░".repeat(thr_width - filled),
        Style::default().fg(Color::DarkGray),
    ));
    config_row.push(Span::styled(format!(" {:4.2}", commands.throttle), val_style));
    config_row.push(Span::styled("   flaps ", lbl_style));
    config_row.push(Span::styled(
        format!("{:<5}", format!("{}°", state.flap_index)),
        val_style,
    ));
    config_row.push(Span::styled("   gear ", lbl_style));
    config_row.push(Span::styled(
        if state.gear_down { "dn" } else { "up" }.to_string(),
        if state.gear_down {
            Style::default().fg(Color::LightGreen)
        } else {
            Style::default().fg(Color::Yellow)
        },
    ));
    config_row.push(Span::raw("   "));
    config_row.push(Span::styled(
        if state.on_ground { "on ground" } else { "airborne" }.to_string(),
        if state.on_ground {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default().fg(Color::LightCyan)
        },
    ));
    config_row.push(Span::raw("   "));
    let rwy_str = match (state.runway_x_ft, state.runway_y_ft) {
        (Some(x), Some(y)) => format!("rwy x{:+.0} y{:+.0}", x, y),
        _ => "rwy —".to_string(),
    };
    config_row.push(Span::styled(rwy_str, Style::default().fg(Color::DarkGray)));

    if let Some(stats) = cache_stats {
        let s = stats.snapshot();
        if s.total_requests > 0 {
            let rate = if s.total_input_tokens > 0 {
                s.total_cached_tokens as f64 / s.total_input_tokens as f64
            } else {
                0.0
            };
            config_row.push(Span::raw("   "));
            config_row.push(Span::styled("cache ", Style::default().fg(Color::DarkGray)));
            let style = if rate >= 0.5 {
                Style::default()
                    .fg(Color::LightGreen)
                    .add_modifier(Modifier::BOLD)
            } else if rate >= 0.2 {
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(Color::DarkGray)
            };
            config_row.push(Span::styled(format!("{:.0}%", rate * 100.0), style));
        }
    }

    lines.push(Line::from(config_row));
    lines
}

fn instrument_line(
    label: &str,
    current_text: String,
    current_unit: &str,
    target_text: Option<String>,
    target_unit: &str,
    value_style: Style,
) -> Line<'static> {
    let mut spans = vec![
        Span::styled(
            format!("  {:<15}", label),
            Style::default().fg(Color::Gray),
        ),
        Span::styled(current_text, value_style),
        Span::styled(format!("{:<8}", current_unit), value_style),
    ];
    match target_text {
        Some(t) => {
            spans.push(Span::styled(t, Style::default().fg(Color::LightCyan)));
            spans.push(Span::styled(
                format!("{:<5}", target_unit),
                Style::default().fg(Color::LightCyan),
            ));
        }
        None => {
            spans.push(Span::styled(
                format!("{:>7}", "—"),
                Style::default().fg(Color::DarkGray),
            ));
        }
    }
    Line::from(spans)
}

fn dev_style(deviation: f64, near: f64, far: f64) -> Style {
    if deviation <= near {
        Style::default().fg(Color::LightGreen).add_modifier(Modifier::BOLD)
    } else if deviation <= far {
        Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::LightRed).add_modifier(Modifier::BOLD)
    }
}

fn phase_style(phase: Option<FlightPhase>) -> Style {
    use crate::types::FlightPhase::*;
    let color = match phase {
        Some(TakeoffRoll | Rotate | Rollout | Final | Roundout | Flare) => Color::Yellow,
        Some(InitialClimb | Crosswind | EnrouteClimb | Cruise | Downwind | Base) => {
            Color::LightGreen
        }
        Some(Descent | PatternEntry) => Color::LightCyan,
        Some(GoAround) => Color::LightRed,
        Some(Preflight | RunwayExit | TaxiClear) | None => Color::White,
    };
    Style::default().fg(color).add_modifier(Modifier::BOLD)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_input_source_defaults_to_operator() {
        assert_eq!(parse_input_source("take off"), ("operator".into(), "take off".into()));
    }

    #[test]
    fn parse_input_source_handles_atc_prefixes() {
        assert_eq!(parse_input_source("atc: N1234 hold"), ("atc".into(), "N1234 hold".into()));
        assert_eq!(parse_input_source("[ATC] cleared"), ("atc".into(), "cleared".into()));
    }

    #[test]
    fn parse_input_source_handles_operator_prefix() {
        assert_eq!(parse_input_source("operator: standby"), ("operator".into(), "standby".into()));
        assert_eq!(parse_input_source("[Operator] hi"), ("operator".into(), "hi".into()));
    }

    #[test]
    fn parse_input_source_is_case_insensitive() {
        assert_eq!(parse_input_source("AtC: hello"), ("atc".into(), "hello".into()));
    }

    // Keeps wrap_degrees_180 import live for future TUI styling hooks that
    // will use it to color-code heading deltas.
    #[test]
    fn wrap_helper_available() {
        assert_eq!(wrap_degrees_180(190.0), -170.0);
    }

    // ---- Radio pane source classification + styling ----------------
    // ---- Propeller spinner for the LOG pane "thinking" line --------
    #[test]
    fn render_thinking_line_has_prop_glyph_and_label() {
        let line = render_thinking_line(Duration::from_millis(0));
        let glyph = PROP_FRAMES[0].to_string();
        assert_eq!(line.spans[0].content, glyph);
        assert_eq!(line.spans[0].style.fg, Some(Color::LightCyan));
        assert!(line.spans[0].style.add_modifier.contains(Modifier::BOLD));
        let body: String = line.spans.iter().skip(1).map(|s| s.content.as_ref()).collect();
        assert_eq!(body, " thinking…");
    }

    #[test]
    fn prop_frame_index_advances_and_wraps() {
        assert_eq!(prop_frame_index(Duration::from_millis(0)), 0);
        assert_eq!(prop_frame_index(Duration::from_millis(100)), 1);
        assert_eq!(prop_frame_index(Duration::from_millis(200)), 2);
        assert_eq!(prop_frame_index(Duration::from_millis(300)), 3);
        assert_eq!(prop_frame_index(Duration::from_millis(400)), 0);
        // sub-frame slop falls on the previous frame.
        assert_eq!(prop_frame_index(Duration::from_millis(99)), 0);
    }

    // ---- Log pane rendering: bar-gutter + sigil-for-errors ----------
    #[test]
    fn render_log_entry_bar_kinds_put_colored_bar_on_every_line() {
        let entry = LogEntry::new(LogKind::Llm, "line one\nline two");
        let lines = render_log_entry(&entry);
        assert_eq!(lines.len(), 2);
        for line in &lines {
            assert_eq!(line.spans[0].content, LOG_BAR);
            assert_eq!(line.spans[0].style.fg, Some(Color::LightGreen));
            assert_eq!(line.spans[1].content, " ");
        }
        assert_eq!(lines[0].spans[2].content, "line one");
        assert_eq!(lines[1].spans[2].content, "line two");
        // Bar-kinds render the body in the default style — color lives
        // on the gutter, not the text.
        assert!(!lines[0].spans[2].style.add_modifier.contains(Modifier::BOLD));
    }

    #[test]
    fn render_log_entry_error_kind_uses_sigil_not_bar() {
        let entry = LogEntry::new(LogKind::Error, "boom");
        let lines = render_log_entry(&entry);
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].spans[0].content, "!");
        assert_ne!(lines[0].spans[0].content, LOG_BAR);
        assert!(lines[0].spans[0].style.add_modifier.contains(Modifier::BOLD));
        // Error body stays bold + red for emphasis.
        assert_eq!(lines[0].spans[2].style.fg, Some(Color::LightRed));
        assert!(lines[0].spans[2].style.add_modifier.contains(Modifier::BOLD));
    }

    #[test]
    fn render_log_entry_system_with_error_substring_upgrades_to_error_style() {
        let entry = LogEntry::new(LogKind::System, "fatal error: boom");
        let lines = render_log_entry(&entry);
        assert_eq!(lines[0].spans[0].content, "!");
    }

    #[test]
    fn build_log_lines_inserts_blank_between_operator_and_llm() {
        let entries = vec![
            LogEntry::new(LogKind::Operator, "take off"),
            LogEntry::new(LogKind::Llm, "rolling"),
            LogEntry::new(LogKind::Llm, "v1"),
            LogEntry::new(LogKind::Operator, "abort"),
        ];
        let lines = build_log_lines(&entries, true, 40);
        let joined: Vec<String> = lines
            .iter()
            .map(|l| {
                l.spans.iter().map(|s| s.content.as_ref()).collect::<String>()
            })
            .collect();
        // operator → blank → llm → llm (no blank, same voice) → blank → operator
        assert_eq!(joined.len(), 6);
        assert!(joined[0].contains("take off"));
        assert_eq!(joined[1].trim(), "");
        assert!(joined[2].contains("rolling"));
        assert!(joined[3].contains("v1"));
        assert_eq!(joined[4].trim(), "");
        assert!(joined[5].contains("abort"));
    }

    #[test]
    fn build_log_lines_no_blank_between_unrelated_kinds() {
        let entries = vec![
            LogEntry::new(LogKind::System, "bridge up"),
            LogEntry::new(LogKind::Llm, "rolling"),
            LogEntry::new(LogKind::Safety, "go_around"),
        ];
        let lines = build_log_lines(&entries, true, 40);
        // No operator↔llm transitions → no separators.
        assert_eq!(lines.len(), 3);
    }

    #[test]
    fn build_log_lines_tints_operator_lines_to_inner_width() {
        let entries = vec![LogEntry::new(LogKind::Operator, "go")];
        let lines = build_log_lines(&entries, true, 40);
        assert_eq!(lines.len(), 1);
        let row = &lines[0];
        assert_eq!(row.width(), 40);
        // Every span on the operator row carries the tint bg.
        for span in &row.spans {
            assert_eq!(span.style.bg, Some(OPERATOR_BG));
        }
    }

    #[test]
    fn build_log_lines_does_not_tint_non_operator_lines() {
        let entries = vec![LogEntry::new(LogKind::Llm, "rolling")];
        let lines = build_log_lines(&entries, true, 40);
        assert_eq!(lines.len(), 1);
        for span in &lines[0].spans {
            assert_eq!(span.style.bg, None);
        }
    }

    #[test]
    fn classify_radio_pilot_transmission() {
        assert_eq!(
            classify_radio_line("[COM1 118.350] Whiteman Tower, N12345 ready for takeoff"),
            RadioSource::Pilot,
        );
        assert_eq!(
            classify_radio_line("[COM2 121.900] Ground, N12345 at ramp, taxi"),
            RadioSource::Pilot,
        );
    }

    #[test]
    fn classify_radio_atc_transmission() {
        assert_eq!(
            classify_radio_line("[atc] N12345 cleared for takeoff runway 24"),
            RadioSource::Atc,
        );
    }

    #[test]
    fn classify_radio_system_tuning_note() {
        assert_eq!(
            classify_radio_line("[COM1 tuned to 118.350]"),
            RadioSource::System,
        );
    }

    #[test]
    fn classify_radio_com_with_empty_body_is_system() {
        // Locks in the invariant used by classify_radio_line: any COM tag
        // with no body after `]` is treated as a system note.
        assert_eq!(
            classify_radio_line("[COM1 118.350]"),
            RadioSource::System,
        );
    }

    #[test]
    fn classify_radio_unknown_prefix_is_system() {
        assert_eq!(classify_radio_line("plain text"), RadioSource::System);
        assert_eq!(classify_radio_line("[weird] foo"), RadioSource::System);
    }

    #[test]
    fn render_radio_line_splits_tag_and_body_spans() {
        let lines = render_radio_line("[atc] cleared to land");
        assert_eq!(lines.len(), 1);
        let spans = &lines[0].spans;
        assert_eq!(spans.len(), 2);
        assert_eq!(spans[0].content, "[atc] ");
        assert!(spans[0].style.add_modifier.contains(Modifier::BOLD));
        assert_eq!(spans[0].style.fg, Some(Color::Yellow));
        assert_eq!(spans[1].content, "cleared to land");
        assert!(!spans[1].style.add_modifier.contains(Modifier::BOLD));
        assert_eq!(spans[1].style.fg, Some(Color::Yellow));
    }

    #[test]
    fn render_radio_line_pilot_uses_cyan() {
        let lines = render_radio_line("[COM1 118.350] tower n12345");
        let spans = &lines[0].spans;
        assert_eq!(spans[0].style.fg, Some(Color::LightCyan));
        assert_eq!(spans[1].style.fg, Some(Color::LightCyan));
    }

    #[test]
    fn render_radio_line_system_tuning_stays_dim_and_single_span() {
        let lines = render_radio_line("[COM1 tuned to 118.350]");
        let spans = &lines[0].spans;
        // Whole line is a bracketed tag with no trailing body.
        assert_eq!(spans.len(), 1);
        assert_eq!(spans[0].style.fg, Some(Color::DarkGray));
    }

    // ---- Emacs-style input editing ----------------------------------
    //
    // With content_width = 40 and a 2-col "> " prompt:
    //   row 0 covers buffer chars  0..38
    //   row 1 covers buffer chars 38..78
    //   row 2 covers buffer chars 78..118

    #[test]
    fn smart_home_single_row_goes_to_start() {
        assert_eq!(smart_home(15, 40), 0);
    }

    #[test]
    fn smart_home_at_start_of_row_walks_up() {
        // Cursor at start of row 2 (char 78) → start of row 1 (char 38).
        assert_eq!(smart_home(78, 40), 38);
        // Cursor at start of row 1 (char 38) → char 0.
        assert_eq!(smart_home(38, 40), 0);
        // Already at char 0 → no-op.
        assert_eq!(smart_home(0, 40), 0);
    }

    #[test]
    fn smart_home_mid_row_goes_to_row_start() {
        // Cursor at char 50 is on row 1 (covers 38..78). Start of row 1 = 38.
        assert_eq!(smart_home(50, 40), 38);
    }

    #[test]
    fn smart_end_to_row_end() {
        let buf: String = "x".repeat(100);
        // Cursor at char 5 on row 0; end of row 0 = 38.
        assert_eq!(smart_end(5, &buf, 40), 38);
    }

    #[test]
    fn smart_end_at_row_end_walks_down() {
        let buf: String = "x".repeat(100);
        // Cursor at char 38 (end of row 0); next press goes to end of row 1 = 78.
        assert_eq!(smart_end(38, &buf, 40), 78);
        // Cursor at char 78 (end of row 1); next press goes to char 100 (buffer end).
        assert_eq!(smart_end(78, &buf, 40), 100);
        // Cursor at char 100 (end of buffer); no-op.
        assert_eq!(smart_end(100, &buf, 40), 100);
    }

    #[test]
    fn smart_end_clamps_to_buffer_length() {
        let buf = "hello".to_string();
        assert_eq!(smart_end(2, &buf, 40), 5);
    }

    #[test]
    fn kill_to_row_end_truncates_within_row() {
        let mut buf: String = "x".repeat(100);
        // Cursor at char 20 on row 0 (covers 0..38). Kill → buffer[0..20] +
        // buffer[38..100], which still leaves 82 chars.
        kill_to_row_end(&mut buf, 20, 40);
        assert_eq!(buf.len(), 82);
    }

    #[test]
    fn kill_to_row_end_at_buffer_end_is_noop() {
        let mut buf: String = "x".repeat(100);
        // Cursor past the end of a 100-char buffer — nothing to kill.
        kill_to_row_end(&mut buf, 100, 40);
        assert_eq!(buf.len(), 100);
    }

    #[test]
    fn kill_to_row_end_at_row_start_kills_whole_row() {
        let mut buf: String = "x".repeat(100);
        // Cursor at char 38 is the start of row 1 (emacs semantics: a
        // position at a row transition belongs to the following row).
        // C-k from here kills row 1's content: chars 38..78 = 40 chars.
        kill_to_row_end(&mut buf, 38, 40);
        assert_eq!(buf.len(), 60);
    }

    #[test]
    fn kill_to_row_end_on_short_buffer_clamps() {
        let mut buf = "hello world".to_string();
        // Cursor at char 5; end of row 0 would be char 38, clamped to 11.
        kill_to_row_end(&mut buf, 5, 40);
        assert_eq!(buf, "hello");
    }

    // ---- wrap_input_spans ------------------------------------------------
    //
    // With content_width = 40 the INPUT pane's renderable rows are 40 cells
    // wide. The "> " prompt lives in the first span so it naturally takes
    // 2 cells of row 0 — no special-casing.

    fn joined(lines: &[Line<'_>]) -> String {
        lines
            .iter()
            .flat_map(|l| l.spans.iter().map(|s| s.content.as_ref()))
            .collect()
    }

    #[test]
    fn wrap_input_spans_empty_buffer_yields_prompt_only_line() {
        let spans = vec![Span::raw("> ")];
        let lines = wrap_input_spans(spans, 40);
        assert_eq!(lines.len(), 1);
        assert_eq!(joined(&lines), "> ");
    }

    #[test]
    fn wrap_input_spans_short_buffer_fits_one_row() {
        let spans = vec![Span::raw("> "), Span::raw("hello")];
        let lines = wrap_input_spans(spans, 40);
        assert_eq!(lines.len(), 1);
        assert_eq!(joined(&lines), "> hello");
    }

    #[test]
    fn wrap_input_spans_exactly_fills_row_zero_no_extra_row() {
        // Prompt (2) + 38-char buffer = 40 cells = exactly one row.
        let spans = vec![Span::raw("> "), Span::raw("x".repeat(38))];
        let lines = wrap_input_spans(spans, 40);
        assert_eq!(lines.len(), 1);
    }

    #[test]
    fn wrap_input_spans_overflows_to_row_one() {
        let spans = vec![Span::raw("> "), Span::raw("x".repeat(40))];
        let lines = wrap_input_spans(spans, 40);
        assert_eq!(lines.len(), 2);
        // Row 0: "> " + 38 x's (40 cells). Row 1: 2 x's.
        assert_eq!(joined(&lines[0..1]), format!("> {}", "x".repeat(38)));
        assert_eq!(joined(&lines[1..2]), "xx");
    }

    #[test]
    fn wrap_input_spans_does_not_word_wrap_on_spaces() {
        // The whole point of this function: unlike ratatui's Wrap, spaces
        // are preserved at hard row boundaries, not used as break points.
        let text = "hello world this is a test of wrapping abc";
        let spans = vec![Span::raw("> "), Span::raw(text)];
        let lines = wrap_input_spans(spans, 40);
        // 2 + 42 = 44 cells → 2 rows.
        assert_eq!(lines.len(), 2);
        // No chars lost or reordered.
        assert_eq!(joined(&lines), format!("> {}", text));
    }

    #[test]
    fn wrap_input_spans_splits_spans_preserving_style() {
        // A long second span straddling a row boundary must produce two
        // styled pieces with the same style.
        let style = Style::default().fg(Color::Cyan);
        let spans = vec![
            Span::styled("> ", style),
            Span::styled("a".repeat(50), Style::default()),
        ];
        let lines = wrap_input_spans(spans, 40);
        assert_eq!(lines.len(), 2);
        // Row 0 has two spans: the prompt + the first 38 a's.
        assert_eq!(lines[0].spans.len(), 2);
        assert_eq!(lines[0].spans[0].style, style);
        // Row 1 has the remaining 12 a's.
        assert_eq!(lines[1].spans.len(), 1);
        assert_eq!(lines[1].spans[0].content.as_ref().len(), 12);
    }

    #[test]
    fn wrap_input_spans_handles_tiny_width() {
        // Pane narrower than the prompt itself — don't panic, just wrap
        // character-by-character.
        let spans = vec![Span::raw("> "), Span::raw("ab")];
        let lines = wrap_input_spans(spans, 1);
        assert_eq!(lines.len(), 4);
        assert_eq!(joined(&lines), "> ab");
    }

    // ---- Input history ----------------------------------------------

    #[test]
    fn history_record_dedups_consecutive_duplicates() {
        let mut h: Vec<String> = Vec::new();
        history_record(&mut h, "a", 10);
        history_record(&mut h, "a", 10);
        history_record(&mut h, "b", 10);
        history_record(&mut h, "b", 10);
        history_record(&mut h, "a", 10);
        assert_eq!(h, vec!["a".to_string(), "b".into(), "a".into()]);
    }

    #[test]
    fn history_record_drops_oldest_past_cap() {
        let mut h: Vec<String> = Vec::new();
        for i in 0..5 {
            history_record(&mut h, &i.to_string(), 3);
        }
        // Oldest entries evicted; the most-recent 3 remain.
        assert_eq!(h, vec!["2".to_string(), "3".into(), "4".into()]);
    }

    #[test]
    fn up_arrow_walks_history_and_saves_draft() {
        let history = vec!["first".to_string(), "second".into(), "third".into()];
        let mut buf = String::from("draft");
        let mut cursor = 5;
        let mut idx: Option<usize> = None;
        let mut draft = String::new();

        // First Up: stash the draft and load "third" (most recent).
        history_go_back(&history, &mut buf, &mut cursor, &mut idx, &mut draft);
        assert_eq!(buf, "third");
        assert_eq!(cursor, 5);
        assert_eq!(idx, Some(2));
        assert_eq!(draft, "draft");

        // Up → "second".
        history_go_back(&history, &mut buf, &mut cursor, &mut idx, &mut draft);
        assert_eq!(buf, "second");
        assert_eq!(idx, Some(1));

        // Up → "first".
        history_go_back(&history, &mut buf, &mut cursor, &mut idx, &mut draft);
        assert_eq!(buf, "first");
        assert_eq!(idx, Some(0));

        // Up at oldest: saturates.
        history_go_back(&history, &mut buf, &mut cursor, &mut idx, &mut draft);
        assert_eq!(buf, "first");
        assert_eq!(idx, Some(0));
    }

    #[test]
    fn down_arrow_walks_forward_and_restores_draft() {
        let history = vec!["first".to_string(), "second".into(), "third".into()];
        let mut buf = String::from("original draft");
        let mut cursor = buf.chars().count();
        let mut idx: Option<usize> = None;
        let mut draft = String::new();

        // Two Ups: land on "second" (idx 1), draft "original draft" stashed.
        history_go_back(&history, &mut buf, &mut cursor, &mut idx, &mut draft);
        history_go_back(&history, &mut buf, &mut cursor, &mut idx, &mut draft);
        assert_eq!(buf, "second");
        assert_eq!(idx, Some(1));

        // Down → "third".
        history_go_forward(&history, &mut buf, &mut cursor, &mut idx, &mut draft);
        assert_eq!(buf, "third");
        assert_eq!(idx, Some(2));

        // Down past end → draft restored, idx cleared.
        history_go_forward(&history, &mut buf, &mut cursor, &mut idx, &mut draft);
        assert_eq!(buf, "original draft");
        assert_eq!(idx, None);
        assert_eq!(cursor, "original draft".chars().count());

        // Down while not browsing: no-op.
        history_go_forward(&history, &mut buf, &mut cursor, &mut idx, &mut draft);
        assert_eq!(buf, "original draft");
        assert_eq!(idx, None);
    }

    #[test]
    fn up_on_empty_history_is_noop() {
        let history: Vec<String> = Vec::new();
        let mut buf = String::from("typing");
        let mut cursor = 6;
        let mut idx: Option<usize> = None;
        let mut draft = String::new();
        history_go_back(&history, &mut buf, &mut cursor, &mut idx, &mut draft);
        assert_eq!(buf, "typing");
        assert_eq!(idx, None);
        assert!(draft.is_empty());
    }

    // ---- PTT key tracker tests ---------------------------------------

    use std::cell::RefCell;

    #[derive(Default)]
    struct FakePtt {
        calls: RefCell<Vec<String>>,
    }

    impl PttControl for FakePtt {
        fn start(&self, prefix: String, atc_mode: bool) {
            let tag = if atc_mode { "atc" } else { "conv" };
            self.calls
                .borrow_mut()
                .push(format!("start:{prefix}|mode={tag}"));
        }
        fn stop(&self) {
            self.calls.borrow_mut().push("stop".to_string());
        }
    }

    fn dispatch(
        kind: KeyEventKind,
        code: KeyCode,
        tracker: &mut PttKeyTracker,
        buffer: &mut String,
        cursor: &mut usize,
        ptt: &FakePtt,
    ) {
        handle_ptt_key(&kind, &code, tracker, buffer, cursor, Some(ptt));
    }

    // ---- Transcript insertion helper ----------------------------------

    #[test]
    fn insert_transcript_into_empty_buffer_lands_at_zero() {
        let mut buf = String::new();
        let mut cur = 0;
        insert_transcript_at_cursor(&mut buf, &mut cur, "hello world");
        assert_eq!(buf, "hello world");
        assert_eq!(cur, 11);
    }

    #[test]
    fn insert_transcript_at_end_of_word_adds_leading_space() {
        let mut buf = String::from("hello");
        let mut cur = 5;
        insert_transcript_at_cursor(&mut buf, &mut cur, "world");
        assert_eq!(buf, "hello world");
        assert_eq!(cur, 11);
    }

    #[test]
    fn insert_transcript_after_space_adds_no_leading_space() {
        let mut buf = String::from("hello ");
        let mut cur = 6;
        insert_transcript_at_cursor(&mut buf, &mut cur, "world");
        assert_eq!(buf, "hello world");
        assert_eq!(cur, 11);
    }

    #[test]
    fn insert_transcript_mid_buffer_splices_at_cursor() {
        // "hello|world" → hold PTT, say "brave new" → cursor goes between
        // the two words, and we auto-space on both sides because neither
        // neighboring char is whitespace.
        let mut buf = String::from("helloworld");
        let mut cur = 5;
        insert_transcript_at_cursor(&mut buf, &mut cur, "brave new");
        assert_eq!(buf, "hello brave new world");
        // Cursor lands just after the trailing auto-space, right before
        // "world".
        assert_eq!(cur, 16);
    }

    #[test]
    fn insert_transcript_between_spaces_no_extra_spaces() {
        let mut buf = String::from("hello  world");
        let mut cur = 6; // between the two spaces
        insert_transcript_at_cursor(&mut buf, &mut cur, "there");
        assert_eq!(buf, "hello there world");
        assert_eq!(cur, 11);
    }

    #[test]
    fn insert_transcript_trims_whitespace_payload() {
        let mut buf = String::new();
        let mut cur = 0;
        insert_transcript_at_cursor(&mut buf, &mut cur, "  hello  ");
        assert_eq!(buf, "hello");
        assert_eq!(cur, 5);
    }

    #[test]
    fn insert_empty_transcript_is_noop() {
        let mut buf = String::from("hello");
        let mut cur = 3;
        insert_transcript_at_cursor(&mut buf, &mut cur, "   ");
        assert_eq!(buf, "hello");
        assert_eq!(cur, 3);
    }

    #[test]
    fn insert_transcript_after_atc_prefix() {
        // "[atc] |" — cursor right after the prefix, transcript slots in
        // cleanly without doubling the space.
        let mut buf = String::from("[atc] ");
        let mut cur = 6;
        insert_transcript_at_cursor(&mut buf, &mut cur, "Whiteman Tower cleared");
        assert_eq!(buf, "[atc] Whiteman Tower cleared");
        assert_eq!(cur, 28);
    }

    #[test]
    fn single_space_press_inserts_char_no_ptt_start() {
        let mut tracker = PttKeyTracker::default();
        let mut buf = String::new();
        let mut cursor = 0;
        let ptt = FakePtt::default();
        dispatch(
            KeyEventKind::Press,
            KeyCode::Char(' '),
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        assert_eq!(buf, " ");
        assert_eq!(cursor, 1);
        assert!(ptt.calls.borrow().is_empty());
        assert!(tracker.pending.is_some());
    }

    #[test]
    fn space_tap_mid_buffer_inserts_at_cursor_not_at_end() {
        // Regression: tapping space with the cursor mid-buffer used to
        // append the space to the end of the buffer and snap the cursor
        // to the end. Now it should insert at the cursor and advance.
        let mut tracker = PttKeyTracker::default();
        let mut buf = String::from("helloworld");
        let mut cursor = 5; // between "hello" and "world"
        let ptt = FakePtt::default();
        dispatch(
            KeyEventKind::Press,
            KeyCode::Char(' '),
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        assert_eq!(buf, "hello world");
        assert_eq!(cursor, 6);
        // Release without a repeat — the space stays where the user put it.
        dispatch(
            KeyEventKind::Release,
            KeyCode::Char(' '),
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        assert_eq!(buf, "hello world");
        assert_eq!(cursor, 6);
        assert!(ptt.calls.borrow().is_empty());
        assert!(tracker.pending.is_none());
    }

    #[test]
    fn space_press_plus_repeat_starts_ptt_and_strips_stray_char() {
        let mut tracker = PttKeyTracker::default();
        let mut buf = String::from("hello ");
        let mut cursor = buf.chars().count();
        let ptt = FakePtt::default();
        dispatch(
            KeyEventKind::Press,
            KeyCode::Char(' '),
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        // After Press: "hello  " (the just-typed space).
        assert_eq!(buf, "hello  ");
        assert_eq!(cursor, 7);
        dispatch(
            KeyEventKind::Repeat,
            KeyCode::Char(' '),
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        // Stray trailing space stripped; no prefix for operator mode.
        assert_eq!(buf, "hello ");
        assert_eq!(cursor, 6);
        assert_eq!(*ptt.calls.borrow(), vec!["start:|mode=conv".to_string()]);
        assert!(tracker.pending.as_ref().unwrap().recording);
    }

    #[test]
    fn tab_press_plus_repeat_starts_ptt_with_atc_prefix() {
        let mut tracker = PttKeyTracker::default();
        let mut buf = String::new();
        let mut cursor = 0;
        let ptt = FakePtt::default();
        dispatch(
            KeyEventKind::Press,
            KeyCode::Tab,
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        assert_eq!(buf, "\t");
        assert_eq!(cursor, 1);
        dispatch(
            KeyEventKind::Repeat,
            KeyCode::Tab,
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        // Stray tab stripped, [atc] prefix prepended at buffer start
        // (not at cursor) so parse_input_source sees it.
        assert_eq!(buf, "[atc] ");
        assert_eq!(cursor, 6);
        assert_eq!(*ptt.calls.borrow(), vec!["start:[atc] |mode=atc".to_string()]);
    }

    #[test]
    fn release_event_stops_ptt_only_when_recording() {
        let mut tracker = PttKeyTracker::default();
        let mut buf = String::new();
        let mut cursor = 0;
        let ptt = FakePtt::default();
        // Press only (no repeat) → tap state, not recording.
        dispatch(
            KeyEventKind::Press,
            KeyCode::Char(' '),
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        dispatch(
            KeyEventKind::Release,
            KeyCode::Char(' '),
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        // Release after a tap should NOT fire stop (would be a no-op PTT).
        assert!(ptt.calls.borrow().is_empty());
        assert!(tracker.pending.is_none());

        // Now simulate press + repeat (recording) + release.
        let mut buf2 = String::new();
        let mut cursor2 = 0;
        let mut tracker2 = PttKeyTracker::default();
        let ptt2 = FakePtt::default();
        dispatch(
            KeyEventKind::Press,
            KeyCode::Char(' '),
            &mut tracker2,
            &mut buf2,
            &mut cursor2,
            &ptt2,
        );
        dispatch(
            KeyEventKind::Repeat,
            KeyCode::Char(' '),
            &mut tracker2,
            &mut buf2,
            &mut cursor2,
            &ptt2,
        );
        dispatch(
            KeyEventKind::Release,
            KeyCode::Char(' '),
            &mut tracker2,
            &mut buf2,
            &mut cursor2,
            &ptt2,
        );
        assert_eq!(
            *ptt2.calls.borrow(),
            vec!["start:|mode=conv".to_string(), "stop".to_string()]
        );
        assert!(tracker2.pending.is_none());
    }

    #[test]
    fn second_press_acts_as_synthetic_repeat_for_non_kitty_terminals() {
        // Simulates a terminal without Kitty keyboard enhancement —
        // auto-repeat arrives as more `Press` events, not `Repeat`.
        let mut tracker = PttKeyTracker::default();
        let mut buf = String::new();
        let mut cursor = 0;
        let ptt = FakePtt::default();
        dispatch(
            KeyEventKind::Press,
            KeyCode::Char(' '),
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        // First press inserts the space and marks pending.
        assert_eq!(buf, " ");
        assert_eq!(cursor, 1);
        dispatch(
            KeyEventKind::Press, // not Repeat!
            KeyCode::Char(' '),
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        // The second Press is treated as a synthetic Repeat, stripping
        // the stray space and starting PTT.
        assert_eq!(buf, "");
        assert_eq!(cursor, 0);
        assert_eq!(*ptt.calls.borrow(), vec!["start:|mode=conv".to_string()]);
        assert!(tracker.pending.as_ref().unwrap().recording);
    }

    #[test]
    fn non_ptt_key_between_repeats_clears_pending() {
        // Simulates the run_tui behavior where any non-PTT key dismisses
        // a pending tap. We test classification only — the dismiss happens
        // in the event loop, so here we just check that a Release without
        // prior recording is a no-op as expected.
        let mut tracker = PttKeyTracker::default();
        tracker.pending = Some(PendingPtt {
            key: PttKey::Space,
            recording: false,
            last_seen: Instant::now(),
        });
        let mut buf = String::new();
        let mut cursor = 0;
        let ptt = FakePtt::default();
        dispatch(
            KeyEventKind::Release,
            KeyCode::Char(' '),
            &mut tracker,
            &mut buf,
            &mut cursor,
            &ptt,
        );
        assert!(ptt.calls.borrow().is_empty());
        assert!(tracker.pending.is_none());
    }
}
