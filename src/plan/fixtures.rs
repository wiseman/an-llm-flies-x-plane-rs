//! KVNY airport data without the apt.dat parquet cache, for tests and
//! offline demos.

use anyhow::{bail, Result};

use crate::plan::resolve::{AirportData, AirportSource, RunwayInfo};
use crate::sim::xplane_bridge::GeoReference;

/// KVNY from apt.dat (X-Plane 12), for tests that shouldn't need the
/// parquet cache.
pub struct KvnyFixture;

impl AirportSource for KvnyFixture {
    fn airport(&self, ident: &str) -> Result<AirportData> {
        if ident != "KVNY" {
            bail!("fixture only has KVNY");
        }
        let (n_lat, n_lon, s_lat, s_lon) = (34.21891, -118.4912405, 34.196994, -118.4891688);
        let (ln_lat, ln_lon, ls_lat, ls_lon) = (34.218996, -118.490004, 34.208005, -118.488965);
        Ok(AirportData {
            ident: "KVNY".into(),
            elevation_ft: 800.0,
            lat: 34.209805556,
            lon: -118.489972222,
            runways: vec![
                rw("16R", n_lat, n_lon, s_lat, s_lon, 175.529, 8019.6, 1433.7),
                rw("34L", s_lat, s_lon, n_lat, n_lon, 355.529, 8019.6, 0.0),
                rw("16L", ln_lat, ln_lon, ls_lat, ls_lon, 175.530, 4021.9, 1433.7),
                rw("34R", ls_lat, ls_lon, ln_lat, ln_lon, 355.530, 4021.9, 0.0),
            ],
        })
    }
}

#[allow(clippy::too_many_arguments)]
fn rw(ident: &str, lat: f64, lon: f64, far_lat: f64, far_lon: f64, course: f64, len: f64, disp: f64) -> RunwayInfo {
    RunwayInfo {
        ident: ident.into(),
        lat,
        lon,
        far_lat,
        far_lon,
        course_deg: course,
        length_ft: len,
        width_ft: 150.0,
        displaced_threshold_ft: disp,
    }
}

pub fn kvny_georef() -> GeoReference {
    GeoReference { threshold_lat_deg: 34.209805556, threshold_lon_deg: -118.489972222 }
}

