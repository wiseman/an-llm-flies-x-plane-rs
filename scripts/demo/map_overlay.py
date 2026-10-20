# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "pillow", "requests", "scipy", "duckdb"]
# ///
"""Render a moving-map inset for a take: the FAA VFR sectional or
OpenStreetMap (--basemap), north up, with the aircraft's position and a
trail of where it has been.

    uv run scripts/demo/map_overlay.py output/sim_pilot-20261019-155704 \\
        --recording ~/Movies/"2026-10-19 15-57-34.mov" --out /tmp/map.mp4

The sectional comes from the FAA's VFR Sectional tile service, OSM from
tile.openstreetmap.org (fetched one at a time with an identifying
User-Agent, per the OSM tile usage policy); both are Web Mercator, so north
is up, and are cached under demo/assets/charts/ so reruns don't refetch. The extent covers the
take's track (its .csv) with a margin, plus any --include points, widened
to the inset's aspect and to at least the chart's native scale (it is
never shrunk below that, and --max-upscale limits how far it is blown up).

The output is an opaque video the size of the inset, one frame per video
frame of the recording, aligned like the comms: frame i shows the track at
the timeline's record start + i/fps - --offset-s (the recording's measured
offset; --plan reads it from a speed_ramp plan.json). Composite it with
`speed_ramp.py --map <this file>`.

--after-touchdown apd switches the inset, at the timeline's touchdown,
to the airport's FAA Airport Diagram (from the d-TPP cycle in effect on
the take's date, found through the d-TPP metafile and cached under
demo/assets/charts/dtpp/). The PDF carries no georeferencing, so it is
rasterized and fitted with an affine transform: the runway bars drawn on
it, matched to the runway ends in the sim's apt.dat cache. The fit's
residuals are printed, and it refuses a fit worse than --apd-max-resid-m.
The diagram view frames the rollout, exit and taxi, with the trail
continuing from the last part of final.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import io
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import requests
from PIL import Image, ImageDraw, ImageFont

BASEMAPS = {
    "sectional": {
        "url": "https://tiles.arcgis.com/tiles/ssFJjBXIUyZDrSYZ/arcgis/rest/services/VFR_Sectional/MapServer/tile/{z}/{y}/{x}",
        "zoom": 12,  # the service's most detailed level, about the chart's native scale
        "cache": "faa_vfr_sectional_tiles",
        "caption": "FAA VFR Sectional \u00b7 Los Angeles",
        "color": "#ff00e0",
        "delay_s": 0.0,
    },
    "osm": {
        "url": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        "zoom": 13,
        "cache": "osm_tiles",
        "caption": "\u00a9 OpenStreetMap contributors",
        "color": "#1565c0",  # strong blue: OSM's roads are pink, orange and yellow
        "delay_s": 0.5,  # be gentle: one request at a time
    },
}
USER_AGENT = "an-llm-flies-x-plane demo map_overlay/1.0 (+https://github.com/wiseman/an-llm-flies-x-plane-rs)"
SS = 2  # supersampling for the trail


def merc(lat: float, lon: float, z: int) -> tuple[float, float]:
    """Global Web Mercator pixel at zoom z."""
    n = 256 * 2**z
    s = math.sin(math.radians(lat))
    return (lon + 180) / 360 * n, (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * n


def ground_m_per_px(lat: float, z: int) -> float:
    return 156543.03392804097 * math.cos(math.radians(lat)) / 2**z


def fetch_tile(bm: dict, cache: Path, x: int, y: int, z: int) -> Image.Image:
    p = cache / str(z) / str(x) / f"{y}.img"
    if not p.exists():
        r = requests.get(bm["url"].format(z=z, x=x, y=y), headers={"User-Agent": USER_AGENT}, timeout=30)
        r.raise_for_status()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(r.content)
        print(f"  fetched tile {z}/{x}/{y}")
        time.sleep(bm["delay_s"])
    return Image.open(io.BytesIO(p.read_bytes())).convert("RGB")


def chart_image(bm: dict, cache: Path, z: int, px0: float, py0: float, px1: float, py1: float,
                size: tuple[int, int]) -> Image.Image:
    """The chart over global-pixel box (px0, py0)-(px1, py1), resized to size."""
    tx0, ty0, tx1, ty1 = int(px0 // 256), int(py0 // 256), int(px1 // 256), int(py1 // 256)
    mosaic = Image.new("RGB", ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256))
    for tx in range(tx0, tx1 + 1):
        for ty in range(ty0, ty1 + 1):
            mosaic.paste(fetch_tile(bm, cache, tx, ty, z), ((tx - tx0) * 256, (ty - ty0) * 256))
    box = (px0 - tx0 * 256, py0 - ty0 * 256, px1 - tx0 * 256, py1 - ty0 * 256)
    if abs((box[2] - box[0]) - size[0]) < 1e-6 and abs((box[3] - box[1]) - size[1]) < 1e-6:
        # Native scale: cut on whole pixels, no resampling.
        bx, by = round(box[0]), round(box[1])
        return mosaic.crop((bx, by, bx + size[0], by + size[1]))
    return mosaic.resize(size, Image.LANCZOS, box=box)


def read_track(stem: Path):
    t, lat, lon, hdg = [], [], [], []
    for row in csv.DictReader(stem.with_suffix(".csv").open()):
        try:
            t.append(dt.datetime.fromisoformat(row["wall_time_utc"]).timestamp())
            lat.append(float(row["lat_deg"]))
            lon.append(float(row["lon_deg"]))
            hdg.append(float(row["heading_deg"]))
        except (KeyError, ValueError):
            continue
    hdg_u = np.degrees(np.unwrap(np.radians(hdg)))
    return np.array(t), np.array(lat), np.array(lon), hdg_u


def record_t0(stem: Path) -> float:
    events = [json.loads(line) for line in stem.with_suffix(".demo.jsonl").read_text().splitlines() if line.strip()]
    for kinds in (("record",), ("roll",)):
        for e in events:
            if e["kind"] in kinds and (e["kind"] != "record" or e.get("action") == "start"):
                return dt.datetime.fromisoformat(e["wall_utc"]).timestamp()
    sys.exit("timeline has no record start or roll")


def touchdown_wall(stem: Path) -> float | None:
    for line in stem.with_suffix(".demo.jsonl").read_text().splitlines():
        if line.strip():
            e = json.loads(line)
            if e["kind"] == "touchdown":
                return dt.datetime.fromisoformat(e["wall_utc"]).timestamp()
    return None


# ---------------------------------------------------------------- airport diagram

DTPP = "https://aeronav.faa.gov/d-tpp"
UA = {"User-Agent": USER_AGENT}


def dtpp_cycle(day: dt.date) -> str:
    """The d-TPP cycle (YYNN) in effect on day: 28-day cycles, numbered within
    the year of their effective date (2501 took effect 2025-01-23)."""
    epoch = dt.date(2025, 1, 23)
    n = (day - epoch).days // 28
    eff = epoch + dt.timedelta(days=28 * n)
    first = eff
    while (first - dt.timedelta(days=28)).year == eff.year:
        first -= dt.timedelta(days=28)
    return f"{eff.year % 100:02d}{(eff - first).days // 28 + 1:02d}"


def fetch(url: str, path: Path) -> Path:
    if not path.exists():
        r = requests.get(url, headers=UA, timeout=120)
        r.raise_for_status()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(r.content)
        print(f"  fetched {url}")
    return path


def airport_diagram_raster(charts: Path, icao: str, day: dt.date, dpi: int = 600) -> tuple[Path, str]:
    import xml.etree.ElementTree as ET

    cycle = dtpp_cycle(day)
    d = charts / "dtpp"
    meta = fetch(f"{DTPP}/{cycle}/xml_data/d-TPP_Metafile.xml", d / f"d-TPP_Metafile_{cycle}.xml")
    pdf_name = None
    for a in ET.parse(meta).iter("airport_name"):
        if a.get("icao_ident") == icao:
            for r in a.findall("record"):
                if r.findtext("chart_code") == "APD":
                    pdf_name = r.findtext("pdf_name")
    if not pdf_name:
        sys.exit(f"no airport diagram for {icao} in d-TPP cycle {cycle}")
    pdf = fetch(f"{DTPP}/{cycle}/{pdf_name}", d / f"{icao}_{pdf_name.split('.')[0]}_{cycle}.pdf")
    png = pdf.with_name(f"{pdf.stem}_{dpi}.png")
    if not png.exists():
        subprocess.run(["pdftoppm", "-r", str(dpi), "-png", "-singlefile", str(pdf), str(png.with_suffix(""))], check=True)
    return png, cycle


def runway_ends(icao: str):
    """[(ident pair, (lat, lon) of each end)] from the newest apt.dat parquet cache."""
    import duckdb

    caches = sorted((Path.home() / ".cache/sim_pilot").glob("apt-*/runways.parquet"), key=lambda p: p.stat().st_mtime)
    if not caches:
        sys.exit("no apt.dat parquet cache under ~/.cache/sim_pilot")
    rows = duckdb.sql(
        f"select le_ident, le_latitude_deg, le_longitude_deg, he_ident, he_latitude_deg, he_longitude_deg "
        f"from read_parquet('{caches[-1]}') where airport_ident = '{icao}' and closed = 0").fetchall()
    return [((r[0], r[3]), (r[1], r[2]), (r[4], r[5])) for r in rows]


def georeference(png: Path, icao: str, max_resid_m: float):
    """Affine from diagram pixels to local metres (east, north) about the
    airport, fitted to the runway bars. Returns (M, enu, residuals)."""
    from scipy import ndimage

    g = np.asarray(Image.open(png).convert("L"))
    op = ndimage.binary_opening(g < 80, structure=np.ones((13, 13)))  # solid bars; drops lines and text
    lab, n = ndimage.label(op)
    bars = []
    for i in range(1, n + 1):
        ys, xs = np.nonzero(lab == i)
        if len(xs) < 2000:
            continue
        pts = np.c_[xs, ys].astype(float)
        c = pts.mean(0)
        _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
        ax = vt[0] if vt[0][1] > 0 else -vt[0]  # pointing down the page (south-ish)
        pr = (pts - c) @ ax
        wd = (pts - c) @ vt[1]
        length, width = pr.max() - pr.min(), np.percentile(wd, 99) - np.percentile(wd, 1)
        if length > 8 * width:
            bars.append((length, c + ax * pr.min(), c + ax * pr.max()))
    rwys = runway_ends(icao)
    lat0 = float(np.mean([e[k][0] for e in rwys for k in (1, 2)]))
    lon0 = float(np.mean([e[k][1] for e in rwys for k in (1, 2)]))
    kx, ky = 111320 * math.cos(math.radians(lat0)), 110574.0

    def enu(la, lo):
        return (lo - lon0) * kx, (la - lat0) * ky

    def rlen(r):
        (e1, n1), (e2, n2) = enu(*r[1]), enu(*r[2])
        return math.hypot(e2 - e1, n2 - n1)

    # Match bars to runways by length rank (works while runway lengths differ).
    bars = sorted(bars, key=lambda b: -b[0])[: len(rwys)]
    rwys = sorted(rwys, key=lambda r: -rlen(r))
    if len(bars) < len(rwys) or len(rwys) < 2:
        sys.exit(f"found {len(bars)} runway bars on the diagram for {len(rwys)} runways; can't georeference")
    P, G, names = [], [], []
    for (length, top, bot), r in zip(bars, rwys):
        north, south = (r[1], r[2]) if r[1][0] > r[2][0] else (r[2], r[1])
        nn, sn = (r[0][0], r[0][1]) if r[1][0] > r[2][0] else (r[0][1], r[0][0])
        P += [top, bot]
        G += [enu(*north), enu(*south)]
        names += [nn, sn]
    A = np.c_[np.array(P), np.ones(len(P))]
    M, *_ = np.linalg.lstsq(A, np.array(G), rcond=None)
    resid = np.hypot(*(A @ M - np.array(G)).T)
    for nm, rr in zip(names, resid):
        print(f"  diagram fit: runway end {nm}: residual {rr:.1f} m")
    mpp = (np.hypot(M[0, 0], M[0, 1]) + np.hypot(M[1, 0], M[1, 1])) / 2
    print(f"  diagram scale {mpp:.3f} m/px, rotation {math.degrees(math.atan2(M[0, 1], M[0, 0])):+.2f} deg")
    if resid.max() > max_resid_m:
        sys.exit(f"airport diagram fit is poor (worst residual {resid.max():.0f} m)")
    return M, enu, resid, g


def diagram_view(png: Path, icao: str, max_resid_m: float, pts_ll, W: int, H: int, margin_m: float,
                 pan_m: tuple[float, float] = (0.0, 0.0)):
    """North-up view of the diagram framing pts_ll, as (RGBA image, to_inset)."""
    M, enu, _, g = georeference(png, icao, max_resid_m)
    # Whiten outside the diagram panel (the outermost long ruled lines).
    dark = g < 128
    rows = np.flatnonzero(dark[:, int(g.shape[1] * 0.1):int(g.shape[1] * 0.9)].mean(1) > 0.6)
    cols = np.flatnonzero(dark[int(g.shape[0] * 0.1):int(g.shape[0] * 0.9), :].mean(0) > 0.6)
    img = Image.open(png).convert("L")
    if len(rows) >= 2 and len(cols) >= 2:
        r0, r1, c0, c1 = rows.min() + 6, rows.max() - 6, cols.min() + 6, cols.max() - 6
        panel = Image.new("L", img.size, 255)
        panel.paste(img.crop((c0, r0, c1, r1)), (c0, r0))
        img = panel
    E, N = zip(*(enu(la, lo) for la, lo in pts_ll))
    e0, e1, n0, n1 = min(E) - margin_m, max(E) + margin_m, min(N) - margin_m, max(N) + margin_m
    s = max((e1 - e0) / W, (n1 - n0) / H)  # metres per inset pixel
    ec, nc = (e0 + e1) / 2 + pan_m[0], (n0 + n1) / 2 + pan_m[1]
    # Pre-shrink the raster close to the target scale, then a ~1:1 affine warp.
    mpp = (np.hypot(M[0, 0], M[0, 1]) + np.hypot(M[1, 0], M[1, 1])) / 2
    k = max(1.0, s / mpp / 1.25)
    small = img.resize((round(img.width / k), round(img.height / k)), Image.LANCZOS)
    Mi = np.linalg.inv(np.r_[M.T, [[0, 0, 1]]])  # (east, north, 1) -> diagram px

    # inset (u, v) -> east = ec + (u - W/2) s, north = nc - (v - H/2) s -> diagram px / k
    T = Mi @ np.array([[s, 0, ec - W / 2 * s], [0, -s, nc + H / 2 * s], [0, 0, 1]])
    T[:2] /= k
    view = small.transform((W, H), Image.AFFINE, tuple(T[:2].ravel()), resample=Image.BICUBIC, fillcolor=255)
    print(f"  diagram view: {W * s / 1000:.2f} x {H * s / 1000:.2f} km, {s:.2f} m per inset px")

    def to_inset(la, lo):
        e, n = enu(la, lo)
        return (e - ec) / s + W / 2, (nc - n) / s + H / 2

    return view.convert("RGBA"), to_inset


DARK = (15, 15, 25, 230)
WHITE = (255, 255, 255, 255)


def trail_strokes(style: str, color) -> list:
    """Layered strokes, outermost first, as (rgba, width in inset px).
    outline: the color with a dark outline (reads on the map tiles).
    halo: the color with a white halo and a thin dark rim, so it reads on
    black pavement and white paper alike (the airport diagram)."""
    if style == "halo":
        return [(DARK, 9), (WHITE, 7), (color, 3)]
    return [(DARK, 6), (color, 3)]


class Trail:
    """A trail drawn incrementally, supersampled, as layered strokes."""

    def __init__(self, W: int, H: int, strokes):
        self.W, self.H = W, H
        self.strokes = [(c, w * SS) for c, w in strokes]
        self.layers = [Image.new("RGBA", (W * SS, H * SS), (0, 0, 0, 0)) for _ in strokes]
        self.draws = [ImageDraw.Draw(im) for im in self.layers]
        self.image = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        self.last = None

    def add(self, p) -> None:
        last = self.last
        if last is None:
            self.last = p
            return
        if abs(p[0] - last[0]) + abs(p[1] - last[1]) <= 0.05:
            return
        a, b = (last[0] * SS, last[1] * SS), (p[0] * SS, p[1] * SS)
        for d, (c, w) in zip(self.draws, self.strokes):
            d.line([a, b], fill=c, width=w)
            for q in (a, b):
                d.ellipse((q[0] - w / 2, q[1] - w / 2, q[0] + w / 2, q[1] + w / 2), fill=c)
        ow = self.strokes[0][1]
        W, H = self.W, self.H
        bx0 = max(int(min(a[0], b[0]) - ow) // SS * SS, 0)
        by0 = max(int(min(a[1], b[1]) - ow) // SS * SS, 0)
        bx1 = min((int(max(a[0], b[0]) + ow) // SS + 1) * SS, W * SS)
        by1 = min((int(max(a[1], b[1]) + ow) // SS + 1) * SS, H * SS)
        if bx1 > bx0 and by1 > by0:
            box = (bx0, by0, bx1, by1)
            reg = self.layers[0].crop(box)
            for im in self.layers[1:]:
                reg = Image.alpha_composite(reg, im.crop(box))
            self.image.paste(reg.resize(((bx1 - bx0) // SS, (by1 - by0) // SS), Image.BOX), (bx0 // SS, by0 // SS))
        self.last = p


def finish_base(base: Image.Image, caption: str, font, dark_caption: bool = True) -> None:
    W, H = base.size
    d = ImageDraw.Draw(base)
    bb = d.textbbox((0, 0), caption, font=font)
    shade = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    ImageDraw.Draw(shade).rectangle((0, H - (bb[3] - bb[1]) - 10, bb[2] - bb[0] + 12, H), fill=(20, 20, 20, 170))
    base.alpha_composite(shade)
    d = ImageDraw.Draw(base)
    d.text((6, H - (bb[3] - bb[1]) - 6 - bb[1]), caption, font=font, fill=(240, 240, 240, 255))
    d.rectangle((0, 0, W - 1, H - 1), outline=(20, 20, 20, 255), width=2)


def plane_icon(color, size: int, style: str = "outline") -> Image.Image:
    """A small top-down airplane pointing up, antialiased: outlined in black
    (outline), or with a white halo and thin dark rim (halo)."""
    k = 4
    s = size * k
    im = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    c = s / 2
    u = s / 24  # design units on a 24-unit square
    pts = [(0, -11), (1.4, -9), (1.6, -3), (10.5, 1), (10.5, 3), (1.6, 1), (1.2, 7), (4.5, 9.5), (4.5, 11),
           (0, 10), (-4.5, 11), (-4.5, 9.5), (-1.2, 7), (-1.6, 1), (-10.5, 3), (-10.5, 1), (-1.6, -3), (-1.4, -9)]
    if style == "halo":
        u = s / 30  # leave room for the halo
    poly = [(c + x * u, c + y * u) for x, y in pts]
    if style == "halo":
        px = s / size  # one inset pixel, in icon units
        ring = poly + poly[:1]
        d.line(ring, fill=DARK, width=int(6 * px), joint="curve")
        d.polygon(poly, fill=DARK)
        d.line(ring, fill=WHITE, width=int(4.4 * px), joint="curve")
        d.polygon(poly, fill=WHITE)
        d.polygon(poly, fill=color)
    else:
        d.polygon(poly, fill=color, outline=(0, 0, 0, 255), width=int(1.4 * u))
    return im.resize((size, size), Image.LANCZOS)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("take", type=Path, help="take stem, e.g. output/sim_pilot-20261019-155704")
    ap.add_argument("--recording", type=Path, required=True, help="the OBS recording (for duration)")
    ap.add_argument("--out", type=Path, required=True, help="overlay video to write (.mp4)")
    ap.add_argument("--basemap", choices=sorted(BASEMAPS), default="sectional",
                    help="sectional (FAA VFR sectional, default) or osm (OpenStreetMap)")
    ap.add_argument("--zoom", type=int, help="tile zoom level (default: 12 for the sectional, 13 for OSM)")
    ap.add_argument("--size", default="640x512", help="inset WxH in pixels (default 640x512)")
    ap.add_argument("--margin-km", type=float, default=1.5, help="margin around the track (default 1.5)")
    ap.add_argument("--include", action="append", default=[], metavar="LAT,LON",
                    help="also keep this point in view (repeatable)")
    ap.add_argument("--max-upscale", type=float, default=0.75,
                    help="widen the extent so the chart is scaled at most this much (default 0.75: the tile service at z12 is about 1.33x the chart's native scan)")
    ap.add_argument("--scale", type=float,
                    help="inset px per tile px at --zoom (sectional z12: 0.75 is about the chart's native scan, 1.5 is "
                         "2x that; 1.0 is native and crisp, the OSM default); centered on the track, widened only if "
                         "the track doesn't fit. Overrides --max-upscale")
    ap.add_argument("--charts-dir", type=Path, default=Path(__file__).resolve().parents[2] / "demo/assets/charts",
                    help="tile cache (default demo/assets/charts)")
    ap.add_argument("--offset-s", type=float, default=None, help="recording offset vs the timeline (s)")
    ap.add_argument("--plan", type=Path, help="speed_ramp plan.json to take --offset-s from")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--trail-color", help="trail and marker color (default: magenta on the sectional, blue on OSM)")
    ap.add_argument("--label", action="append", default=[], metavar="TEXT@LAT,LON",
                    help="add a map label, left end at LAT,LON (e.g. a street the tiles don't name at this zoom)")
    ap.add_argument("--after-touchdown", choices=["none", "apd"], default="none",
                    help="apd: switch to the airport's FAA Airport Diagram at touchdown (default none)")
    ap.add_argument("--apd-airport", default="KVNY", help="ICAO of the airport diagram (default KVNY)")
    ap.add_argument("--apd-final-s", type=float, default=30.0,
                    help="seconds of final before touchdown the diagram's trail starts with (default 30)")
    ap.add_argument("--apd-frame-final-s", type=float, default=8.0,
                    help="seconds of final before touchdown the diagram view frames (default 8)")
    ap.add_argument("--apd-margin-m", type=float, default=150.0, help="margin around the rollout and taxi (default 150)")
    ap.add_argument("--apd-style", choices=["halo", "outline"], default="halo",
                    help="diagram trail and marker: colored core with a white halo (default), or the map's dark outline")
    ap.add_argument("--apd-color", default="#ff4500", help="diagram trail and marker color (default orange-red)")
    ap.add_argument("--apd-marker-px", type=int, default=28, help="diagram marker size (default 28)")
    ap.add_argument("--apd-pan-m", default="0,0", metavar="EAST,NORTH",
                    help="shift the diagram view (metres), e.g. to keep a note off its edge (default 0,0)")
    ap.add_argument("--apd-max-resid-m", type=float, default=30.0, help="refuse a worse diagram fit (default 30)")
    ap.add_argument("--crossfade-s", type=float, default=0.5, help="source seconds to crossfade at touchdown (default 0.5)")
    ap.add_argument("--limit-s", type=float, help="stop after this many seconds (testing)")
    ap.add_argument("--preview", type=float, action="append", default=[], metavar="SECONDS",
                    help="also save a PNG of the inset at this recording time (repeatable)")
    args = ap.parse_args()

    W, H = (int(v) for v in args.size.lower().split("x"))
    bm = BASEMAPS[args.basemap]
    Z = args.zoom or bm["zoom"]
    if args.scale is None and args.basemap == "osm":
        args.scale = 1.0
    trail_color = args.trail_color or bm["color"]
    offset = args.offset_s
    if offset is None:
        offset = json.loads(args.plan.read_text()).get("offset_s", 0.0) if args.plan else 0.0
    t0 = record_t0(args.take)
    tt, lat, lon, hdg = read_track(args.take)
    dur = float(subprocess.check_output(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of",
                                         "csv=p=0", str(args.recording.expanduser())], text=True))
    nframes = math.ceil(dur * args.fps - 1e-6)
    if args.limit_s:
        nframes = min(nframes, int(args.limit_s * args.fps))

    # Extent, in global Web Mercator pixels at ZOOM.
    pts = [(a, b) for a, b in zip(lat, lon)] + [tuple(float(v) for v in p.split(",")) for p in args.include]
    xs, ys = zip(*(merc(a, b, Z) for a, b in pts))
    mid_lat = float(np.mean([p[0] for p in pts]))
    mpp = ground_m_per_px(mid_lat, Z)
    m = args.margin_km * 1000 / mpp
    x0, x1, y0, y1 = min(xs) - m, max(xs) + m, min(ys) - m, max(ys) + m
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    w, h = x1 - x0, y1 - y0
    w, h = max(w, h * W / H), max(h, w * H / W)  # the inset's aspect
    if args.scale:
        tw, th = max(xs) - min(xs), max(ys) - min(ys)  # the track (and includes) without margin
        w = W / args.scale
        h = w * H / W
        if tw > w or th > h:
            w = max(tw * 1.05, th * 1.05 * W / H)
            h = w * H / W
            print(f"  !! the track doesn't fit at --scale {args.scale}; widened to {W / w:.2f}x")
    else:
        min_w = W / args.max_upscale  # don't blow the chart up past max_upscale
        if w < min_w:
            w, h = min_w, min_w * H / W
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    if abs(W / w - 1) < 1e-9:  # native: whole pixels
        x0, y0 = round(x0), round(y0)
        x1, y1 = x0 + W, y0 + H
    scale = W / w  # inset px per chart px
    print(f"{args.basemap} z{Z} extent: {w * mpp / 1000:.1f} x {h * mpp / 1000:.1f} km, scaled {scale:.2f}x "
          f"({mpp / scale:.1f} m per inset px)")
    # lat/lon corners for the report
    def inv(px, py):
        n = 256 * 2**Z
        lo = px / n * 360 - 180
        la = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * py / n))))
        return la, lo
    (la_n, lo_w), (la_s, lo_e) = inv(x0, y0), inv(x1, y1)
    print(f"  north {la_n:.4f}, south {la_s:.4f}, west {lo_w:.4f}, east {lo_e:.4f}")

    cache = args.charts_dir / bm["cache"]
    base = chart_image(bm, cache, Z, x0, y0, x1, y1, (W, H)).convert("RGBA")
    # Caption and border.
    d = ImageDraw.Draw(base)
    try:
        font = ImageFont.truetype("/System/Library/Fonts/SFNS.ttf", 13)
    except OSError:
        font = ImageFont.load_default()
    for spec in args.label:
        text, at = spec.rsplit("@", 1)
        la_, lo_ = (float(v) for v in at.split(","))
        px_, py_ = merc(la_, lo_, Z)
        lx, ly = (px_ - x0) * scale, (py_ - y0) * scale
        try:
            lf = ImageFont.truetype("/System/Library/Fonts/SFNSItalic.ttf", 12)
        except OSError:
            lf = font
        lb = d.textbbox((0, 0), text, font=lf)
        d.text((lx, ly - (lb[1] + lb[3]) / 2), text, font=lf, fill=(60, 60, 60, 255),
               stroke_width=2, stroke_fill=(255, 255, 255, 230))
    finish_base(base, bm["caption"], font)

    def to_inset(la, lo):
        px, py = merc(la, lo, Z)
        return (px - x0) * scale, (py - y0) * scale

    color = tuple(int(trail_color.lstrip("#")[i:i + 2], 16) for i in (0, 2, 4)) + (255,)
    icon = plane_icon(color, 22)
    trail = Trail(W, H, trail_strokes("outline", color))

    apd = None
    if args.after_touchdown == "apd":
        td = touchdown_wall(args.take)
        if td is None:
            sys.exit("the timeline has no touchdown event")
        day = dt.datetime.fromtimestamp(t0, dt.timezone.utc).date()
        png, cycle = airport_diagram_raster(args.charts_dir, args.apd_airport, day)
        print(f"airport diagram: {args.apd_airport}, d-TPP cycle {cycle}, {png}")
        sel = (tt >= td - args.apd_frame_final_s)
        frame_pts = list(zip(lat[sel], lon[sel]))
        apd_base, apd_to_inset = diagram_view(png, args.apd_airport, args.apd_max_resid_m, frame_pts, W, H,
                                              args.apd_margin_m,
                                              tuple(float(v) for v in args.apd_pan_m.split(",")))
        finish_base(apd_base, f"FAA Airport Diagram \u00b7 {args.apd_airport}", font)
        acolor = tuple(int(args.apd_color.lstrip("#")[i:i + 2], 16) for i in (0, 2, 4)) + (255,)
        apd = {"td": td, "base": apd_base, "to_inset": apd_to_inset,
               "trail": Trail(W, H, trail_strokes(args.apd_style, acolor)),
               "icon": plane_icon(acolor, args.apd_marker_px, args.apd_style),
               "start": td - args.apd_final_s}
        print(f"  switch at {td - t0 + offset:.2f}s in the recording, crossfade {args.crossfade_s}s")

    previews = sorted(args.preview)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    enc = subprocess.Popen(
        ["nice", "-n", "19", "ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
         "-r", str(args.fps), "-i", "pipe:0", "-c:v", "libx264", "-preset", "medium", "-crf", "12", "-g", str(args.fps),
         "-pix_fmt", "yuv420p", "-color_primaries", "bt709", "-color_trc", "bt709", "-colorspace", "bt709",
         str(args.out)],
        stdin=subprocess.PIPE,
    )
    for i in range(nframes):
        wall = t0 + i / args.fps - offset
        la = float(np.interp(wall, tt, lat))
        lo = float(np.interp(wall, tt, lon))
        hd = float(np.interp(wall, tt, hdg)) % 360
        p = to_inset(la, lo)
        trail.add(p)
        def compose(b, tr, q, icn=icon):
            ic = icn.rotate(-hd, resample=Image.BICUBIC, expand=True)
            f = Image.alpha_composite(b, tr.image)
            f.alpha_composite(ic, (int(round(q[0] - ic.width / 2)), int(round(q[1] - ic.height / 2))))
            return f

        # Before the switch, mix = 0 (the map); after, 1 (the diagram).
        mix = 0.0
        if apd:
            if wall >= apd["start"]:
                apd["trail"].add(apd["to_inset"](la, lo))
            half = args.crossfade_s / 2
            mix = 0.0 if wall < apd["td"] - half else 1.0 if wall >= apd["td"] + half or half == 0 else \
                (wall - (apd["td"] - half)) / (2 * half)
        if mix <= 0:
            frame = compose(base, trail, p)
        elif mix >= 1:
            frame = compose(apd["base"], apd["trail"], apd["to_inset"](la, lo), apd["icon"])
        else:
            frame = Image.blend(compose(base, trail, p),
                                compose(apd["base"], apd["trail"], apd["to_inset"](la, lo), apd["icon"]), mix)
        rgb = frame.convert("RGB")
        enc.stdin.write(rgb.tobytes())
        while previews and previews[0] <= i / args.fps:
            pt = previews.pop(0)
            pp = args.out.with_name(f"{args.out.stem}_{pt:07.2f}s.png")
            rgb.save(pp)
            print(f"  preview {pp}")
        if i % (args.fps * 60) == 0:
            print(f"  {i / args.fps:6.0f}s / {dur:.0f}s", flush=True)
    enc.stdin.close()
    if enc.wait() != 0:
        sys.exit("ffmpeg failed")
    print(f"wrote {args.out} ({nframes} frames, {W}x{H})")


if __name__ == "__main__":
    main()
