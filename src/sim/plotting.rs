//! SVG plot writer for scenario runs.
//!
//! Produces two SVGs: a 5-panel overview (altitude, speed, attitude,
//! throttle, heading + phase timeline) and a north-up ground path with
//! runway overlay. The structure the tests check for — phase labels,
//! runway threshold label, "phase by time", "north-up world frame" — is
//! preserved.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::Result;

use crate::config::ConfigBundle;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::sim::scenario::{ScenarioLogRow, ScenarioResult};
use crate::types::{FlightPhase, Vec2};

pub struct PlotArtifacts {
    pub overview_svg: PathBuf,
    pub ground_path_svg: PathBuf,
}

fn phase_color(phase: FlightPhase) -> &'static str {
    match phase {
        FlightPhase::TakeoffRoll => "#475569",
        FlightPhase::Rotate => "#7c3aed",
        FlightPhase::InitialClimb => "#2563eb",
        FlightPhase::Crosswind => "#1d4ed8",
        FlightPhase::EnrouteClimb => "#0891b2",
        FlightPhase::Cruise => "#0f766e",
        FlightPhase::Descent => "#65a30d",
        FlightPhase::PatternEntry => "#f59e0b",
        FlightPhase::Downwind => "#ea580c",
        FlightPhase::Base => "#dc2626",
        FlightPhase::Final => "#be123c",
        FlightPhase::Roundout => "#db2777",
        FlightPhase::Flare => "#c026d3",
        FlightPhase::Rollout => "#4338ca",
        FlightPhase::RunwayExit => "#3730a3",
        FlightPhase::TaxiClear => "#334155",
        FlightPhase::GoAround => "#b91c1c",
        FlightPhase::Preflight => "#94a3b8",
    }
}

fn phase_label(phase: FlightPhase) -> String {
    phase.value().replace('_', " ")
}

#[derive(Debug, Clone)]
struct PhaseSegment {
    phase: FlightPhase,
    start_time_s: f64,
    end_time_s: f64,
    start_index: usize,
    end_index: usize,
}

fn phase_segments(rows: &[ScenarioLogRow]) -> Vec<PhaseSegment> {
    if rows.is_empty() {
        return vec![];
    }
    let mut segments = Vec::new();
    let mut current_phase = rows[0].phase;
    let mut start_index = 0usize;
    let mut start_time_s = rows[0].time_s;
    for (index, row) in rows.iter().enumerate().skip(1) {
        if row.phase == current_phase {
            continue;
        }
        segments.push(PhaseSegment {
            phase: current_phase,
            start_time_s,
            end_time_s: rows[index - 1].time_s,
            start_index,
            end_index: index - 1,
        });
        current_phase = row.phase;
        start_index = index - 1;
        start_time_s = rows[index - 1].time_s;
    }
    segments.push(PhaseSegment {
        phase: current_phase,
        start_time_s,
        end_time_s: rows.last().unwrap().time_s,
        start_index,
        end_index: rows.len() - 1,
    });
    segments
}

fn expanded_domain(values: &[f64], extra_padding: f64) -> (f64, f64) {
    let lower = values.iter().cloned().fold(f64::INFINITY, f64::min);
    let upper = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if (upper - lower).abs() <= 1e-9 {
        return (lower - 1.0, upper + 1.0);
    }
    let padding = (upper - lower) * extra_padding;
    (lower - padding, upper + padding)
}

fn expand_to_equal_aspect(
    x_domain: (f64, f64),
    y_domain: (f64, f64),
    plot_width: f64,
    plot_height: f64,
) -> (f64, f64, f64, f64) {
    let (x_min, x_max) = x_domain;
    let (y_min, y_max) = y_domain;
    let x_span = (x_max - x_min).max(1.0);
    let y_span = (y_max - y_min).max(1.0);
    let data_aspect = x_span / y_span;
    let plot_aspect = plot_width / plot_height;
    if data_aspect > plot_aspect {
        let desired_y = x_span / plot_aspect;
        let extra_y = (desired_y - y_span) / 2.0;
        (x_min, x_max, y_min - extra_y, y_max + extra_y)
    } else {
        let desired_x = y_span * plot_aspect;
        let extra_x = (desired_x - x_span) / 2.0;
        (x_min - extra_x, x_max + extra_x, y_min, y_max)
    }
}

fn scale(value: f64, domain_min: f64, domain_max: f64, screen_min: f64, screen_max: f64) -> f64 {
    if (domain_max - domain_min).abs() <= 1e-9 {
        return (screen_min + screen_max) / 2.0;
    }
    let ratio = (value - domain_min) / (domain_max - domain_min);
    screen_min + ratio * (screen_max - screen_min)
}

pub fn write_scenario_plots(
    result: &ScenarioResult,
    config: &ConfigBundle,
    scenario_name: &str,
    output_dir: impl AsRef<Path>,
) -> Result<PlotArtifacts> {
    let dir = output_dir.as_ref();
    fs::create_dir_all(dir)?;
    let overview_path = dir.join("flight_overview.svg");
    let ground_path = dir.join("ground_path.svg");
    fs::write(&overview_path, build_overview_svg(result, scenario_name))?;
    fs::write(&ground_path, build_ground_path_svg(result, config, scenario_name))?;
    Ok(PlotArtifacts {
        overview_svg: overview_path,
        ground_path_svg: ground_path,
    })
}

fn svg_header(width: i32, height: i32) -> String {
    format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w}\" height=\"{h}\" viewBox=\"0 0 {w} {h}\">",
        w = width,
        h = height
    )
}

fn build_polyline(
    x_values: &[f64],
    y_values: &[f64],
    x_domain: (f64, f64),
    y_domain: (f64, f64),
    bounds: (f64, f64, f64, f64),
) -> String {
    let (left, top, width, height) = bounds;
    let mut out = String::new();
    for (x, y) in x_values.iter().zip(y_values.iter()) {
        let sx = scale(*x, x_domain.0, x_domain.1, left, left + width);
        let sy = scale(*y, y_domain.0, y_domain.1, top + height, top);
        if !out.is_empty() {
            out.push(' ');
        }
        out.push_str(&format!("{:.2},{:.2}", sx, sy));
    }
    out
}

fn build_overview_svg(result: &ScenarioResult, scenario_name: &str) -> String {
    let rows = &result.history;
    let width = 1280;
    let height = 1180;
    let margin_left = 96.0;
    let margin_right = 28.0;
    let margin_top = 52.0;
    let margin_bottom = 84.0;
    let panel_gap = 22.0;
    let panel_count = 5;
    let plot_width = width as f64 - margin_left - margin_right;
    let panel_height =
        (height as f64 - margin_top - margin_bottom - panel_gap * (panel_count - 1) as f64)
            / panel_count as f64;

    let x_values: Vec<f64> = rows.iter().map(|r| r.time_s).collect();
    let (x_min, x_max) = if x_values.is_empty() {
        (0.0, 1.0)
    } else {
        (x_values[0], *x_values.last().unwrap())
    };
    let segments = phase_segments(rows);

    type Series<'a> = (&'a str, &'a str, Vec<f64>);
    type Panel<'a> = (&'a str, Vec<Series<'a>>, Option<(f64, f64)>);
    let panels: Vec<Panel> = vec![
        (
            "Altitude AGL (ft)",
            vec![("altitude_agl_ft", "#0f766e", rows.iter().map(|r| r.altitude_agl_ft).collect())],
            None,
        ),
        (
            "Speed (kt)",
            vec![
                ("ground_speed", "#2563eb", rows.iter().map(|r| r.gs_kt).collect()),
                ("ias", "#16a34a", rows.iter().map(|r| r.ias_kt).collect()),
            ],
            None,
        ),
        (
            "Attitude (deg)",
            vec![
                ("bank", "#dc2626", rows.iter().map(|r| r.bank_deg).collect()),
                ("pitch", "#d97706", rows.iter().map(|r| r.pitch_deg).collect()),
            ],
            None,
        ),
        (
            "Throttle (0..1)",
            vec![
                ("throttle_pos", "#7c3aed", rows.iter().map(|r| r.throttle_pos).collect()),
                ("throttle_cmd", "#6b7280", rows.iter().map(|r| r.throttle_cmd).collect()),
            ],
            Some((0.0, 1.0)),
        ),
        (
            "Heading (deg)",
            vec![("heading", "#0891b2", rows.iter().map(|r| r.heading_deg).collect())],
            Some((0.0, 360.0)),
        ),
    ];

    let mut elements: Vec<String> = Vec::new();
    elements.push(svg_header(width, height));
    elements.push("<rect width=\"100%\" height=\"100%\" fill=\"#f8fafc\" />".to_string());
    elements.push(format!(
        "<text x=\"{:.1}\" y=\"28\" font-family=\"monospace\" font-size=\"20\" fill=\"#0f172a\">Scenario: {}</text>",
        margin_left, scenario_name
    ));
    elements.push(format!(
        "<text x=\"{:.1}\" y=\"46\" font-family=\"monospace\" font-size=\"12\" fill=\"#334155\">duration={:.1}s final_phase={} success={}</text>",
        margin_left,
        result.duration_s,
        result.final_phase.value(),
        result.success
    ));

    for (index, (title, series_defs, fixed_domain)) in panels.iter().enumerate() {
        let panel_top = margin_top + index as f64 * (panel_height + panel_gap);
        let panel_bottom = panel_top + panel_height;
        let series_values: Vec<f64> = series_defs.iter().flat_map(|(_, _, v)| v.clone()).collect();
        let (y_min, y_max) = fixed_domain.unwrap_or_else(|| expanded_domain(&series_values, 0.06));
        elements.push(format!(
            "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"white\" stroke=\"#cbd5e1\" stroke-width=\"1\" />",
            margin_left, panel_top, plot_width, panel_height
        ));

        for frac in [0.25, 0.5, 0.75] {
            let y = panel_top + panel_height * frac;
            elements.push(format!(
                "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#e2e8f0\" stroke-width=\"1\" />",
                margin_left,
                y,
                margin_left + plot_width,
                y
            ));
        }

        for seg in segments.iter().skip(1) {
            let x = scale(seg.start_time_s, x_min, x_max, margin_left, margin_left + plot_width);
            elements.push(format!(
                "<line x1=\"{x:.1}\" y1=\"{:.1}\" x2=\"{x:.1}\" y2=\"{:.1}\" stroke=\"#e2e8f0\" stroke-width=\"1\" />",
                panel_top,
                panel_bottom,
                x = x,
            ));
        }

        let legend_x = margin_left + 8.0;
        let legend_y = panel_top + 16.0;
        elements.push(format!(
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"13\" fill=\"#0f172a\">{}</text>",
            legend_x, legend_y, title
        ));
        for (si, (label, color, values)) in series_defs.iter().enumerate() {
            let polyline = build_polyline(
                &x_values,
                values,
                (x_min, x_max),
                (y_min, y_max),
                (margin_left, panel_top, plot_width, panel_height),
            );
            elements.push(format!(
                "<polyline fill=\"none\" stroke=\"{}\" stroke-width=\"2\" points=\"{}\" />",
                color, polyline
            ));
            elements.push(format!(
                "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"12\" fill=\"{}\">{}</text>",
                legend_x + 180.0 + si as f64 * 150.0,
                legend_y,
                color,
                label
            ));
        }
        elements.push(format!(
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"11\" fill=\"#475569\">{:.1}</text>",
            margin_left - 36.0,
            panel_top + panel_height / 2.0,
            y_max
        ));
        elements.push(format!(
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"11\" fill=\"#475569\">{:.1}</text>",
            margin_left - 36.0,
            panel_bottom - 4.0,
            y_min
        ));
    }

    // Phase timeline at the bottom.
    let phase_axis_top = height as f64 - margin_bottom + 18.0;
    let phase_axis_height = 26.0;
    elements.push(format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"12\" fill=\"#475569\">phase by time</text>",
        margin_left,
        phase_axis_top - 8.0
    ));
    elements.push(format!(
        "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"white\" stroke=\"#cbd5e1\" stroke-width=\"1\" />",
        margin_left, phase_axis_top, plot_width, phase_axis_height
    ));
    for seg in &segments {
        let start_x = scale(seg.start_time_s, x_min, x_max, margin_left, margin_left + plot_width);
        let end_x = scale(seg.end_time_s, x_min, x_max, margin_left, margin_left + plot_width);
        let seg_width = (end_x - start_x).max(1.5);
        let color = phase_color(seg.phase);
        elements.push(format!(
            "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"{}\" fill-opacity=\"0.22\" stroke=\"{}\" stroke-width=\"0.8\" />",
            start_x, phase_axis_top, seg_width, phase_axis_height, color, color
        ));
        let label = phase_label(seg.phase);
        if seg_width >= 24.0 {
            let font_size = if seg_width < 70.0 { 9 } else { 10 };
            elements.push(format!(
                "<text x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"middle\" font-family=\"monospace\" font-size=\"{}\" fill=\"#0f172a\">{}</text>",
                (start_x + end_x) / 2.0,
                phase_axis_top + 16.5,
                font_size,
                label
            ));
        }
    }

    elements.push(format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"12\" fill=\"#475569\">time_s: {:.1}..{:.1}</text>",
        margin_left,
        height as f64 - 18.0,
        x_min,
        x_max
    ));
    elements.push("</svg>".to_string());
    elements.join("\n")
}

fn build_ground_path_svg(result: &ScenarioResult, config: &ConfigBundle, scenario_name: &str) -> String {
    let rows = &result.history;
    let width = 1280;
    let height = 900;
    let margin_left = 90.0;
    let margin_right = 40.0;
    let margin_top = 55.0;
    let margin_bottom = 70.0;
    let plot_width = width as f64 - margin_left - margin_right;
    let plot_height = height as f64 - margin_top - margin_bottom;

    let runway_frame = RunwayFrame::new(config.airport.runway.clone());
    let runway_length_ft = config.airport.runway.length_ft;
    let runway_half_width_ft = 75.0;
    let touchdown_zone_end_ft = config.airport.runway.touchdown_zone_ft.min(runway_length_ft / 3.0);

    let runway_polygon_world: Vec<Vec2> = [
        Vec2::new(0.0, -runway_half_width_ft),
        Vec2::new(0.0, runway_half_width_ft),
        Vec2::new(runway_length_ft, runway_half_width_ft),
        Vec2::new(runway_length_ft, -runway_half_width_ft),
    ]
    .iter()
    .map(|p| runway_frame.to_world_frame(*p))
    .collect();
    let touchdown_polygon_world: Vec<Vec2> = [
        Vec2::new(0.0, -runway_half_width_ft),
        Vec2::new(0.0, runway_half_width_ft),
        Vec2::new(touchdown_zone_end_ft, runway_half_width_ft),
        Vec2::new(touchdown_zone_end_ft, -runway_half_width_ft),
    ]
    .iter()
    .map(|p| runway_frame.to_world_frame(*p))
    .collect();
    let centerline = (
        runway_frame.to_world_frame(Vec2::ZERO),
        runway_frame.to_world_frame(Vec2::new(runway_length_ft, 0.0)),
    );
    let threshold_world = runway_frame.to_world_frame(Vec2::ZERO);
    let departure_end_world = runway_frame.to_world_frame(Vec2::new(runway_length_ft, 0.0));

    let mut xs: Vec<f64> = rows.iter().map(|r| r.position_x_ft).collect();
    let mut ys: Vec<f64> = rows.iter().map(|r| r.position_y_ft).collect();
    for p in &runway_polygon_world {
        xs.push(p.x);
        ys.push(p.y);
    }
    let (x_min, x_max) = expanded_domain(&xs, 0.08);
    let (y_min, y_max) = expanded_domain(&ys, 0.08);
    let (x_min, x_max, y_min, y_max) =
        expand_to_equal_aspect((x_min, x_max), (y_min, y_max), plot_width, plot_height);

    let segments = phase_segments(rows);

    let sx = |v: f64| scale(v, x_min, x_max, margin_left, margin_left + plot_width);
    let sy = |v: f64| scale(v, y_min, y_max, margin_top + plot_height, margin_top);

    let mut elements: Vec<String> = Vec::new();
    elements.push(svg_header(width, height));
    elements.push("<rect width=\"100%\" height=\"100%\" fill=\"#f8fafc\" />".to_string());
    elements.push(format!(
        "<text x=\"{:.1}\" y=\"28\" font-family=\"monospace\" font-size=\"20\" fill=\"#0f172a\">Ground Path: {}</text>",
        margin_left, scenario_name
    ));
    elements.push(format!(
        "<text x=\"{:.1}\" y=\"46\" font-family=\"monospace\" font-size=\"12\" fill=\"#334155\">north-up world frame: x=east_ft y=north_ft</text>",
        margin_left
    ));
    elements.push(format!(
        "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"white\" stroke=\"#cbd5e1\" stroke-width=\"1\" />",
        margin_left, margin_top, plot_width, plot_height
    ));

    for frac in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let x = margin_left + frac * plot_width;
        let y = margin_top + frac * plot_height;
        elements.push(format!(
            "<line x1=\"{x:.1}\" y1=\"{:.1}\" x2=\"{x:.1}\" y2=\"{:.1}\" stroke=\"#e2e8f0\" stroke-width=\"1\" />",
            margin_top,
            margin_top + plot_height,
            x = x
        ));
        elements.push(format!(
            "<line x1=\"{:.1}\" y1=\"{y:.1}\" x2=\"{:.1}\" y2=\"{y:.1}\" stroke=\"#e2e8f0\" stroke-width=\"1\" />",
            margin_left,
            margin_left + plot_width,
            y = y
        ));
    }

    let poly = runway_polygon_world
        .iter()
        .map(|p| format!("{:.2},{:.2}", sx(p.x), sy(p.y)))
        .collect::<Vec<_>>()
        .join(" ");
    let tdz = touchdown_polygon_world
        .iter()
        .map(|p| format!("{:.2},{:.2}", sx(p.x), sy(p.y)))
        .collect::<Vec<_>>()
        .join(" ");
    let center = format!(
        "{:.2},{:.2} {:.2},{:.2}",
        sx(centerline.0.x),
        sy(centerline.0.y),
        sx(centerline.1.x),
        sy(centerline.1.y)
    );
    elements.push(format!(
        "<polygon points=\"{}\" fill=\"#cbd5e1\" stroke=\"#64748b\" stroke-width=\"1.5\" />",
        poly
    ));
    elements.push(format!(
        "<polygon points=\"{}\" fill=\"#94a3b8\" opacity=\"0.45\" />",
        tdz
    ));
    elements.push(format!(
        "<polyline points=\"{}\" fill=\"none\" stroke=\"white\" stroke-width=\"2\" stroke-dasharray=\"16 10\" />",
        center
    ));
    let rwy_id = config.airport.runway.id.as_deref().unwrap_or("");
    elements.push(format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"12\" fill=\"#334155\">Runway {} threshold</text>",
        sx(threshold_world.x) + 8.0,
        sy(threshold_world.y) - 8.0,
        rwy_id
    ));
    elements.push(format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"12\" fill=\"#334155\">departure end</text>",
        sx(departure_end_world.x) + 8.0,
        sy(departure_end_world.y) - 8.0
    ));
    // "north up" text and N indicator
    elements.push(format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"12\" fill=\"#334155\">N (north up)</text>",
        margin_left + plot_width - 96.0,
        margin_top + 18.0
    ));

    // Per-phase trajectory polylines.
    for seg in &segments {
        let slice = &rows[seg.start_index..=seg.end_index];
        if slice.len() < 2 {
            continue;
        }
        let polyline = slice
            .iter()
            .map(|r| format!("{:.2},{:.2}", sx(r.position_x_ft), sy(r.position_y_ft)))
            .collect::<Vec<_>>()
            .join(" ");
        elements.push(format!(
            "<polyline fill=\"none\" stroke=\"{}\" stroke-width=\"2.5\" points=\"{}\" />",
            phase_color(seg.phase),
            polyline
        ));
    }

    // Phase legend (labels only — so test assertions like "takeoff roll" pass).
    let mut seen: Vec<FlightPhase> = Vec::new();
    for seg in &segments {
        if !seen.contains(&seg.phase) {
            seen.push(seg.phase);
        }
    }
    let legend_x = margin_left + 12.0;
    let legend_y = margin_top + 22.0;
    for (i, phase) in seen.iter().enumerate() {
        let row = (i % 5) as f64;
        let col = (i / 5) as f64;
        let x = legend_x + col * 180.0;
        let y = legend_y + row * 18.0;
        let color = phase_color(*phase);
        elements.push(format!(
            "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"12\" height=\"12\" fill=\"{}\" />",
            x,
            y - 9.0,
            color
        ));
        elements.push(format!(
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"11\" fill=\"#334155\">{}</text>",
            x + 18.0,
            y + 1.0,
            phase_label(*phase)
        ));
    }

    if let (Some(start), Some(end)) = (rows.first(), rows.last()) {
        elements.push(format!(
            "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"5\" fill=\"#16a34a\" />",
            sx(start.position_x_ft),
            sy(start.position_y_ft)
        ));
        elements.push(format!(
            "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"5\" fill=\"#dc2626\" />",
            sx(end.position_x_ft),
            sy(end.position_y_ft)
        ));
        elements.push(format!(
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"11\" fill=\"#166534\">start</text>",
            sx(start.position_x_ft) + 8.0,
            sy(start.position_y_ft) - 8.0
        ));
        elements.push(format!(
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"11\" fill=\"#991b1b\">end</text>",
            sx(end.position_x_ft) + 8.0,
            sy(end.position_y_ft) - 8.0
        ));
    }

    elements.push(format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"12\" fill=\"#475569\">x: east-west world feet, y: north-south world feet, north up</text>",
        margin_left,
        height as f64 - 18.0
    ));
    elements.push("</svg>".to_string());
    elements.join("\n")
}
