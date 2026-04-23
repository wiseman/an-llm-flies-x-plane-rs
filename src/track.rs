//! Live flight-track recorder.
//!
//! Streams a per-second position CSV to disk (crash-safe: each row is
//! flushed before we move on) and keeps a parallel in-memory buffer that
//! is rendered as a KML `gx:Track` on clean shutdown. The CSV is the
//! durable artifact — if the process is killed it's still valid; the
//! KML is a derived convenience for Google Earth replay and is only
//! written when `write_kml` is called from the shutdown path.
//!
//! The KML uses `<altitudeMode>absolute</altitudeMode>` with altitudes
//! in meters MSL, which is what Earth expects for a real 3D track.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use chrono::{DateTime, Utc};

const M_PER_FT: f64 = 0.3048;

#[derive(Debug, Clone)]
pub struct TrackPoint {
    pub wall_time: DateTime<Utc>,
    pub t_sim: f64,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_ft: f64,
    pub alt_agl_ft: f64,
    pub heading_deg: f64,
    pub track_deg: f64,
    pub ias_kt: f64,
    pub gs_kt: f64,
    pub vs_fpm: f64,
    pub phase: String,
    pub on_ground: bool,
}

pub struct TrackRecorder {
    csv_path: PathBuf,
    csv_writer: BufWriter<File>,
    points: Vec<TrackPoint>,
}

impl TrackRecorder {
    /// Open the CSV for streaming writes and emit the header row.
    pub fn new(csv_path: &Path) -> Result<Self> {
        if let Some(parent) = csv_path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("creating {}", parent.display()))?;
        }
        let file = File::create(csv_path)
            .with_context(|| format!("opening {}", csv_path.display()))?;
        let mut w = BufWriter::new(file);
        writeln!(
            w,
            "wall_time_utc,t_sim,lat_deg,lon_deg,alt_msl_ft,alt_agl_ft,heading_deg,track_deg,ias_kt,gs_kt,vs_fpm,phase,on_ground"
        )?;
        w.flush()?;
        Ok(Self {
            csv_path: csv_path.to_path_buf(),
            csv_writer: w,
            points: Vec::new(),
        })
    }

    pub fn csv_path(&self) -> &Path {
        &self.csv_path
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Append a single fix. Flushes the CSV before returning so a crash
    /// loses at most the last sample.
    pub fn record(&mut self, pt: TrackPoint) -> Result<()> {
        writeln!(
            self.csv_writer,
            "{},{:.3},{:.7},{:.7},{:.1},{:.1},{:.2},{:.2},{:.2},{:.2},{:.1},{},{}",
            pt.wall_time.to_rfc3339(),
            pt.t_sim,
            pt.lat_deg,
            pt.lon_deg,
            pt.alt_msl_ft,
            pt.alt_agl_ft,
            pt.heading_deg,
            pt.track_deg,
            pt.ias_kt,
            pt.gs_kt,
            pt.vs_fpm,
            pt.phase,
            pt.on_ground,
        )?;
        self.csv_writer.flush()?;
        self.points.push(pt);
        Ok(())
    }

    /// Render the in-memory buffer as a KML `gx:Track` at `path`. Safe
    /// to call with zero recorded points — produces an empty track that
    /// still validates.
    pub fn write_kml(&self, path: &Path, session_name: &str) -> Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).ok();
        }
        let file = File::create(path)
            .with_context(|| format!("opening {}", path.display()))?;
        let mut w = BufWriter::new(file);
        writeln!(w, r#"<?xml version="1.0" encoding="UTF-8"?>"#)?;
        writeln!(
            w,
            r#"<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">"#
        )?;
        writeln!(w, "  <Document>")?;
        writeln!(w, "    <name>{}</name>", xml_escape(session_name))?;
        writeln!(w, "    <Placemark>")?;
        writeln!(w, "      <name>Flight track</name>")?;
        writeln!(w, "      <gx:Track>")?;
        writeln!(w, "        <altitudeMode>absolute</altitudeMode>")?;
        // gx:Track expects all `<when>` elements first, then all
        // `<gx:coord>` elements in matching order.
        for p in &self.points {
            writeln!(w, "        <when>{}</when>", p.wall_time.to_rfc3339())?;
        }
        for p in &self.points {
            let alt_m = p.alt_msl_ft * M_PER_FT;
            writeln!(
                w,
                "        <gx:coord>{:.7} {:.7} {:.2}</gx:coord>",
                p.lon_deg, p.lat_deg, alt_m
            )?;
        }
        writeln!(w, "      </gx:Track>")?;
        writeln!(w, "    </Placemark>")?;
        writeln!(w, "  </Document>")?;
        writeln!(w, "</kml>")?;
        w.flush()?;
        Ok(())
    }
}

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_point(t_sim: f64, lat: f64, lon: f64, alt_ft: f64) -> TrackPoint {
        TrackPoint {
            wall_time: DateTime::<Utc>::from_timestamp(1_700_000_000 + t_sim as i64, 0).unwrap(),
            t_sim,
            lat_deg: lat,
            lon_deg: lon,
            alt_msl_ft: alt_ft,
            alt_agl_ft: alt_ft,
            heading_deg: 90.0,
            track_deg: 90.0,
            ias_kt: 100.0,
            gs_kt: 100.0,
            vs_fpm: 0.0,
            phase: "cruise".to_string(),
            on_ground: false,
        }
    }

    #[test]
    fn csv_row_round_trips_with_header() {
        let dir = tempfile::tempdir().unwrap();
        let csv = dir.path().join("t.csv");
        let mut rec = TrackRecorder::new(&csv).unwrap();
        rec.record(fake_point(0.0, 37.5, -122.4, 1000.0)).unwrap();
        rec.record(fake_point(1.0, 37.51, -122.41, 1100.0)).unwrap();
        let body = std::fs::read_to_string(&csv).unwrap();
        let lines: Vec<&str> = body.lines().collect();
        assert_eq!(lines.len(), 3);
        assert!(lines[0].starts_with("wall_time_utc,t_sim,"));
        assert!(lines[1].contains(",37.5000000,-122.4000000,"));
        assert!(lines[2].contains(",37.5100000,-122.4100000,"));
    }

    #[test]
    fn kml_contains_times_then_coords() {
        let dir = tempfile::tempdir().unwrap();
        let csv = dir.path().join("t.csv");
        let kml = dir.path().join("t.kml");
        let mut rec = TrackRecorder::new(&csv).unwrap();
        rec.record(fake_point(0.0, 37.5, -122.4, 1000.0)).unwrap();
        rec.record(fake_point(1.0, 37.51, -122.41, 1100.0)).unwrap();
        rec.write_kml(&kml, "test-session").unwrap();
        let body = std::fs::read_to_string(&kml).unwrap();
        assert!(body.contains("<gx:Track>"));
        assert!(body.contains("<altitudeMode>absolute</altitudeMode>"));
        let when_idx = body.find("<when>").unwrap();
        let coord_idx = body.find("<gx:coord>").unwrap();
        assert!(when_idx < coord_idx, "all <when>s come before <gx:coord>s");
        // KML altitude is in meters — 1000 ft → ~304.8 m.
        assert!(body.contains("304.80"));
    }

    #[test]
    fn empty_track_still_produces_valid_kml_skeleton() {
        let dir = tempfile::tempdir().unwrap();
        let csv = dir.path().join("t.csv");
        let kml = dir.path().join("t.kml");
        let rec = TrackRecorder::new(&csv).unwrap();
        rec.write_kml(&kml, "empty").unwrap();
        let body = std::fs::read_to_string(&kml).unwrap();
        assert!(body.starts_with("<?xml"));
        assert!(body.contains("</kml>"));
        assert!(body.contains("<gx:Track>"));
    }
}
