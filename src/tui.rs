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
        snapshot.active_profiles.join(", ")
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
    // altitude
    out.push_str(&format!(
        "  altitude       {}{}{}\n",
        fmt_right(&format!("{:.0}", state.alt_agl_ft), 10),
        fmt_left(" AGL", 8),
        tgt_str(tgt_alt_agl, 7, " AGL"),
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

    out.push_str(&format!(
        "  throttle {} {:4.2}   flaps {:<5}   gear {}   {}   {}",
        throttle_bar,
        commands.throttle,
        flap_str,
        gear_str,
        air_str,
        rwy_str,
    ));

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
use crossterm::event::{self, Event, KeyCode, KeyModifiers};
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

use crate::bus::SimBus;
use crate::core::mission_manager::PilotCore;
use crate::live_runner::HeartbeatPump;
use crate::llm::conversation::IncomingMessage;
use crate::llm::responses_client::CacheStats;

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
) -> Result<()> {
    // Set up terminal.
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut input_buffer = String::new();
    let mut log_detail = true;
    let frame_interval = Duration::from_millis(100);

    let outcome = loop {
        let next_tick = Instant::now() + frame_interval;

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
                if k.modifiers.contains(KeyModifiers::CONTROL) {
                    match k.code {
                        KeyCode::Char('c') | KeyCode::Char('d') => {
                            break_outer(&stop_event);
                            break;
                        }
                        KeyCode::Char('t') => {
                            log_detail = !log_detail;
                            continue;
                        }
                        _ => {}
                    }
                }
                match k.code {
                    KeyCode::Enter => {
                        let raw = input_buffer.trim().to_string();
                        if !raw.is_empty() {
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
                                bus.push_log(format!("[{}] {}", source, payload));
                            }
                        }
                        input_buffer.clear();
                    }
                    KeyCode::Esc => {
                        break_outer(&stop_event);
                        break;
                    }
                    KeyCode::Backspace => {
                        input_buffer.pop();
                    }
                    KeyCode::Char(c) => {
                        input_buffer.push(c);
                    }
                    _ => {}
                }
            }
        }
        if stop_event.load(Ordering::Acquire) {
            break Ok(());
        }

        let status_fragments = {
            let snapshot = pilot.lock().latest_snapshot.clone();
            render_status_fragments(snapshot.as_ref(), cache_stats.as_deref())
        };
        let (_, logs, radio) = bus.snapshot();
        // A single bus entry can span multiple physical lines (e.g. a
        // takeoff_checklist or sql_query tool result). Split on '\n' and
        // expand tabs to 4-space stops so ratatui doesn't render the
        // control characters as single-column glyphs that garble columns.
        let log_lines: Vec<Line> = logs
            .iter()
            .filter(|l| log_detail || !is_verbose_line(l))
            .flat_map(|l| {
                split_and_normalize(l)
                    .into_iter()
                    .map(|segment| Line::from(colorize_log_line(&segment)))
                    .collect::<Vec<_>>()
            })
            .collect();
        let radio_lines: Vec<Line> = radio
            .iter()
            .flat_map(|l| {
                split_and_normalize(l)
                    .into_iter()
                    .map(|segment| Line::from(Span::raw(segment)))
                    .collect::<Vec<_>>()
            })
            .collect();

        terminal.draw(|f| {
            let area = f.area();
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Length(12), // status
                    Constraint::Min(3),     // log (scrolling)
                    Constraint::Length(8),  // radio
                    Constraint::Length(3),  // input
                ])
                .split(area);

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
            let log_title = if log_detail { " LOG " } else { " LOG (compact) " };
            let log_paragraph = Paragraph::new(log_lines.clone())
                .block(Block::default().borders(Borders::ALL).title(log_title))
                .wrap(Wrap { trim: false });
            f.render_widget(
                log_paragraph.scroll((pin_to_bottom(&log_lines, chunks[1]), 0)),
                chunks[1],
            );

            let radio_paragraph = Paragraph::new(radio_lines.clone())
                .block(Block::default().borders(Borders::ALL).title(" RADIO "))
                .wrap(Wrap { trim: false });
            f.render_widget(
                radio_paragraph.scroll((pin_to_bottom(&radio_lines, chunks[2]), 0)),
                chunks[2],
            );

            let input_title = " INPUT — enter to send · ctrl-t log detail · ctrl-c to exit ";
            let input_paragraph = Paragraph::new(Line::from(vec![
                Span::styled("> ", Style::default().fg(Color::Cyan)),
                Span::raw(input_buffer.clone()),
            ]))
            .block(Block::default().borders(Borders::ALL).title(input_title));
            f.render_widget(input_paragraph, chunks[3]);
        })?;
    };

    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;
    outcome
}

fn break_outer(stop: &AtomicBool) {
    stop.store(true, Ordering::Release);
}

fn is_verbose_line(line: &str) -> bool {
    line.contains("[llm-worker] tool ") || line.contains("[heartbeat]")
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

fn colorize_log_line(line: &str) -> Vec<Span<'static>> {
    let lower = line.to_ascii_lowercase();
    let style = if lower.contains("error") {
        Style::default()
            .fg(Color::LightRed)
            .add_modifier(Modifier::BOLD)
    } else if lower.contains("[safety]") {
        Style::default().fg(Color::Yellow)
    } else if lower.contains("[heartbeat]") {
        Style::default().fg(Color::DarkGray)
    } else if lower.contains("[llm]") {
        Style::default().fg(Color::LightCyan)
    } else {
        Style::default()
    };
    vec![Span::styled(line.to_string(), style)]
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
        snap.active_profiles.join(", ")
    };
    let phase = snap.phase.map(|p| p.value().to_string()).unwrap_or_else(|| "—".to_string());

    let tgt_hdg = guidance.and_then(|g| g.target_heading_deg.or(g.target_track_deg));
    let tgt_alt_msl = guidance.and_then(|g| g.target_altitude_ft);
    let tgt_spd = guidance.and_then(|g| g.target_speed_kt);
    let field_elev = state.alt_msl_ft - state.alt_agl_ft;
    let tgt_alt_agl = tgt_alt_msl.map(|a| a - field_elev);

    let phase_style = phase_style(&phase);

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
        header.push(Span::styled(phase.to_uppercase(), phase_style));
        header.push(Span::raw("  "));
        header.push(Span::styled(rwy_parts.join(" "), Style::default().fg(Color::Gray)));
        header.push(Span::raw("  "));
    } else {
        header.push(Span::styled(
            format!("{:<24}", phase.to_uppercase()),
            phase_style,
        ));
    }
    header.push(Span::styled(profiles, Style::default().fg(Color::DarkGray)));
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

fn phase_style(phase: &str) -> Style {
    match phase {
        "takeoff_roll" | "rotate" | "rollout" => {
            Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
        }
        "initial_climb" | "crosswind" | "enroute_climb" | "cruise" | "downwind" => {
            Style::default().fg(Color::LightGreen).add_modifier(Modifier::BOLD)
        }
        "descent" | "pattern_entry" => {
            Style::default().fg(Color::LightCyan).add_modifier(Modifier::BOLD)
        }
        "base" => Style::default().fg(Color::LightGreen).add_modifier(Modifier::BOLD),
        "final" | "roundout" | "flare" => {
            Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
        }
        "go_around" => Style::default().fg(Color::LightRed).add_modifier(Modifier::BOLD),
        _ => Style::default().fg(Color::DarkGray).add_modifier(Modifier::BOLD),
    }
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
}
