"""Pilot vocabulary: build a flight plan the way a pilot or controller says it.

Agent code runs once per clearance and *builds a plan*; nothing here flies
the aircraft. Statements register themselves:

  - a lateral step or a `.then()` chain becomes the step sequence (one per call);
  - a lone climb/descend/maintain/speed applies right away, alongside it;
  - pattern edits on `p` (the live pattern) amend it in place;
  - remain()/prefer()/report() add constraints and reports.

Units: bare numbers are MSL feet, knots or degrees. Distances need nm() or
ft(); agl() converts to MSL. Must stay Python 3.9-compatible and stdlib-only.
"""

import difflib
import math

NM_FT = 6076.12


class PlanError(Exception):
    """A mistake in the plan code, reported back to the agent verbatim."""


# ---------------------------------------------------------------- state

_ctx = {}
_b = None


class _Builder:
    def __init__(self):
        self.top = []          # registered steps/chains, creation order
        self.constraints = []
        self.reports = []
        self.release = []
        self.cancel = []
        self.replace = False
        self.next_id = 1
        self.live_pattern = None
        self.go_around = False
        self.permits = []
        self.preview = False

    def new_id(self, prefix):
        existing = set(c["id"] for c in (_ctx.get("plan") or {}).get("constraints", []))
        existing |= set(r["id"] for r in (_ctx.get("plan") or {}).get("reports", []))
        while True:
            i = "%s%d" % (prefix, self.next_id)
            self.next_id += 1
            if i not in existing:
                return i

    def register(self, item):
        self.top.append(item)

    def unregister(self, item):
        self.top = [x for x in self.top if x is not item]


def _reset(ctx):
    global _ctx, _b
    _ctx = ctx
    _b = _Builder()


# ---------------------------------------------------------------- units


class Ft(float):
    """A distance (or altitude) in feet."""

    def __repr__(self):
        if abs(self) >= 3000:
            return "%.1f nm" % (self / NM_FT)
        return "%.0f ft" % self


class Agl(float):
    """Height above the field; converted to MSL where it's used."""

    def __repr__(self):
        return "%.0f ft AGL" % self


def ft(x):
    return Ft(x)


def nm(x):
    return Ft(x * NM_FT)


def agl(x):
    return Agl(x)


def kt(x):
    return float(x)


def deg(x):
    return float(x)


def _alt(x, what="altitude"):
    if isinstance(x, Agl):
        return float(_ctx["field_elevation_ft"] + x)
    if isinstance(x, (int, float)):
        return float(x)
    raise PlanError("%s must be a number (MSL feet), ft(...) or agl(...); got %r" % (what, x))


def _dist(x, what="distance"):
    if isinstance(x, Ft):
        return float(x)
    if isinstance(x, (int, float)) and x == 0:
        return 0.0
    if isinstance(x, (int, float)):
        raise PlanError("%s needs units: write nm(%s) or ft(%s)" % (what, x, x))
    raise PlanError("%s must be nm(...) or ft(...); got %r" % (what, x))


def _speed(x):
    if isinstance(x, (int, float)):
        return float(x)
    raise PlanError("speed must be knots (a number or a named speed like Vy); got %r" % (x,))


# ---------------------------------------------------------------- constants


class _Const(object):
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self.name


LEFT, RIGHT = "left", "right"
NORTH, EAST, SOUTH, WEST = 0.0, 90.0, 180.0, 270.0
OPEN = _Const("OPEN")
PRESENT = _Const("PRESENT")
RUNWAY = _Const("RUNWAY")
TIGHT = _Const("TIGHT")
NORMAL = _Const("NORMAL")
WIDE = _Const("WIDE")
SHORT = _Const("SHORT")
FULL_STOP = "full_stop"
TOUCH_AND_GO = "touch_and_go"
OPTION = TOUCH_AND_GO
LOW_APPROACH = "low_approach"
EITHER = "either"
# depart(via=...) and enter(via=...)
STRAIGHT_OUT = _Const("STRAIGHT_OUT")
CROSSWIND = _Const("CROSSWIND")
DOWNWIND = _Const("DOWNWIND")
FORTY_FIVE = _Const("FORTY_FIVE")
MIDFIELD_CROSSWIND = _Const("MIDFIELD_CROSSWIND")
OVERHEAD = _Const("OVERHEAD")

_LEGS = ("upwind", "crosswind", "downwind", "base", "final")
_TURN_LEGS = ("crosswind", "downwind", "base", "final")


def _dir(d):
    if d in (LEFT, RIGHT):
        return d
    raise PlanError("direction must be LEFT or RIGHT; got %r" % (d,))


# ---------------------------------------------------------------- geometry helpers


def _normalize(s):
    cleaned = "".join(c if c.isalnum() else ("" if c in ".'" else " ") for c in s.lower())
    return " ".join(w for w in cleaned.split() if w != "the")


def _haversine_nm(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi, dlam = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
    return 2 * 6371008.8 * math.asin(math.sqrt(a)) / 1852.0


def _bearing(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlam)
    return math.degrees(math.atan2(x, y)) % 360


_COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


def _where(lat, lon):
    apt = _ctx.get("airport_latlon")
    if not apt:
        return ""
    d = _haversine_nm(apt[0], apt[1], lat, lon)
    b = _bearing(apt[0], apt[1], lat, lon)
    return "%.1f nm %s of %s" % (d, _COMPASS[int((b + 22.5) // 45) % 8], _ctx.get("airport", "the field"))


# ---------------------------------------------------------------- references


class Ref(object):
    """A place on the map: point, line or area. Print one to see where it is."""

    def __init__(self, js, label, kind, latlon=None):
        self.js = js
        self.label = label
        self.kind = kind
        self.latlon = latlon

    def __repr__(self):
        where = _where(*self.latlon) if self.latlon else ""
        return "<%s %s%s>" % (self.kind, self.label, (", " + where) if where else "")


def _places():
    return _ctx.get("places", [])


def place(name, near=None):
    """A named place from the local list (roads, landmarks, nicknames)."""
    q = _normalize(name)
    for p in _places():
        if _normalize(p["name"]) == q or any(_normalize(a) == q for a in p.get("aliases", [])):
            return Ref({"ref": "place", "name": p["name"]}, p["name"], p["kind"], (p["lat"], p["lon"]))
    names = [p["name"] for p in _places()] + [a for p in _places() for a in p.get("aliases", [])]
    close = difflib.get_close_matches(name, names, n=4, cutoff=0.5)
    hint = (" Did you mean: %s?" % ", ".join(close)) if close else ""
    known = ", ".join(p["name"] for p in _places())
    raise PlanError("unknown place %r.%s Known places: %s. For anything else use latlon(lat, lon)." % (name, hint, known))


def road(name):
    r = place(name)
    if r.kind not in ("road", "line"):
        raise PlanError("%s is a %s, not a road" % (r.label, r.kind))
    return r


line = road
landmark = place


def latlon(lat, lon):
    return Ref({"ref": "latlon", "lat": float(lat), "lon": float(lon)}, "%.4f,%.4f" % (lat, lon), "point", (lat, lon))


def interchange(a, b):
    ra = a if isinstance(a, Ref) else road(a)
    rb = b if isinstance(b, Ref) else road(b)
    return Ref({"ref": "intersection", "a": ra.js, "b": rb.js}, "%s/%s interchange" % (ra.label, rb.label), "point")


class Runway(object):
    def __init__(self, airport_ident, info):
        self.airport = airport_ident
        self.ident = info["ident"]
        self.course = info["course_deg"]
        self.length = Ft(info["length_ft"])
        self.info = info

    def _part(self, part, kind):
        return Ref({"ref": "runway", "airport": self.airport, "runway": self.ident, "part": part},
                   "%s %s %s" % (self.airport, self.ident, part.replace("_", " ")), kind,
                   (self.info["lat"], self.info["lon"]))

    threshold = property(lambda self: self._part("threshold", "point"))
    numbers = threshold
    approach_end = property(lambda self: self._part("approach_end", "point"))
    aim_point = property(lambda self: self._part("aim_point", "point"))
    midfield = property(lambda self: self._part("midfield", "point"))
    departure_end = property(lambda self: self._part("departure_end", "point"))
    end = departure_end
    centerline = property(lambda self: self._part("centerline", "line"))
    extended_centerline = property(lambda self: self._part("extended_centerline", "line"))

    def at(self, along=None, right=None):
        """A point in this runway's frame: `along` past the approach end
        (negative = before it), `right` of the runway's course (negative =
        left). Both need units: rwy.at(along=nm(1), right=nm(-1))."""
        a = _dist(along, "along") if along is not None else 0.0
        r_ = _dist(right, "right") if right is not None else 0.0
        return Ref({"ref": "runway_frame", "airport": self.airport, "runway": self.ident, "along_ft": a, "right_ft": r_},
                   "%s %s %+.1f/%+.1f nm" % (self.airport, self.ident, a / NM_FT, r_ / NM_FT), "point")

    def __repr__(self):
        return "<runway %s %s, course %03.0f, %.0f ft>" % (self.airport, self.ident, self.course, self.length)


class Airport(Ref):
    def __init__(self, ident):
        lat, lon = _ctx.get("airport_latlon", (0.0, 0.0))
        Ref.__init__(self, {"ref": "airport", "ident": ident}, ident, "area", (lat, lon))
        self.ident = ident

    def runway(self, ident):
        want = str(ident).upper().strip()
        for r in _ctx.get("runways", []):
            if r["ident"].upper() == want:
                return Runway(self.ident, r)
        raise PlanError("no runway %s at %s; runways: %s"
                        % (ident, self.ident, ", ".join(r["ident"] for r in _ctx.get("runways", []))))

    @property
    def runways(self):
        return [Runway(self.ident, r) for r in _ctx.get("runways", [])]


def airport(ident=None):
    here = _ctx.get("airport")
    if ident is not None and here and ident.upper() != here.upper():
        raise PlanError("only %s's data is loaded for this plan; fly to %s with latlon(...) for now" % (here, ident))
    if not here:
        raise PlanError("no airport context; the plan needs an airport")
    return Airport(here)


# ---------------------------------------------------------------- conditions


class Cond(object):
    """When something happens. Combine with & (and), | (or), ~ (not)."""

    def __init__(self, js, text):
        self.js = js
        self.text = text

    def __and__(self, other):
        return Cond({"cond": "all", "conds": [self.js, _cond(other).js]}, "%s and %s" % (self.text, _cond(other).text))

    def __or__(self, other):
        return Cond({"cond": "any", "conds": [self.js, _cond(other).js]}, "%s or %s" % (self.text, _cond(other).text))

    def __invert__(self):
        return Cond({"cond": "not", "of": self.js}, "not %s" % self.text)

    def __repr__(self):
        return "<when %s>" % self.text


def _cond(c):
    if isinstance(c, Cond):
        return c
    raise PlanError("expected a condition (crossing(...), at(...), NOW, ...); got %r" % (c,))


def _ref(x):
    if isinstance(x, Ref):
        return x
    if isinstance(x, str):
        return place(x)
    raise PlanError("expected a place, road, runway point or latlon(); got %r" % (x,))


NOW = Cond({"cond": "now"}, "now")
ASAP = Cond({"cond": "asap"}, "as soon as able")
NEVER = Cond({"cond": "never"}, "never")


def atc(token):
    """A gate the controller opens later ("I'll call your base" -> atc("base")).
    Open it in a later call with release("base")."""
    return Cond({"cond": "atc", "token": str(token)}, "ATC calls %s" % token)


def advised():
    return Cond({"cond": "advised"}, "advised")


def at(x, radius=None):
    r = _ref(x)
    js = {"cond": "at", "target": r.js}
    if radius is not None:
        js["radius_ft"] = _dist(radius, "radius")
    return Cond(js, "at %s" % r.label)


def over(x, width=None):
    r = _ref(x)
    js = {"cond": "over", "target": r.js}
    if width is not None:
        js["width_ft"] = _dist(width, "width")
    return Cond(js, "over %s" % r.label)


def abeam(x):
    r = _ref(x)
    return Cond({"cond": "abeam", "target": r.js}, "abeam %s" % r.label)


def crossing(x):
    r = _ref(x)
    return Cond({"cond": "crossing", "target": r.js}, "crossing %s" % r.label)


def past(x):
    r = _ref(x)
    return Cond({"cond": "past", "target": r.js}, "past %s" % r.label)


def side_of(x, side):
    r = _ref(x)
    names = {0.0: "north", 90.0: "east", 180.0: "south", 270.0: "west"}
    return Cond({"cond": "side_of", "target": r.js, "side_deg": float(side)},
                "%s of %s" % (names.get(float(side), "%.0f° side" % side), r.label))


def north_of(x):
    return side_of(x, NORTH)


def south_of(x):
    return side_of(x, SOUTH)


def east_of(x):
    return side_of(x, EAST)


def west_of(x):
    return side_of(x, WEST)


def within(distance, x):
    r = _ref(x)
    d = _dist(distance)
    return Cond({"cond": "within", "target": r.js, "distance_ft": d}, "within %r of %s" % (Ft(d), r.label))


def inside(x):
    r = _ref(x)
    return Cond({"cond": "inside", "target": r.js}, "inside %s" % r.label)


def outside(x):
    r = _ref(x)
    return Cond({"cond": "outside", "target": r.js}, "outside %s" % r.label)


def at_or_below(alt):
    a = _alt(alt)
    return Cond({"cond": "at_or_below", "altitude_ft": a}, "at or below %.0f" % a)


def at_or_above(alt):
    a = _alt(alt)
    return Cond({"cond": "at_or_above", "altitude_ft": a}, "at or above %.0f" % a)


below = at_or_below
above = at_or_above


def reaching(alt):
    a = _alt(alt)
    return Cond({"cond": "reaching", "altitude_ft": a}, "reaching %.0f" % a)


def agl_below(h):
    return Cond({"cond": "agl_below", "ft": float(h)}, "below %.0f ft AGL" % h)


def agl_above(h):
    return Cond({"cond": "agl_above", "ft": float(h)}, "above %.0f ft AGL" % h)


def distance_flown(d):
    return Cond({"cond": "distance_flown", "ft": _dist(d)}, "after %r" % Ft(_dist(d)))


def elapsed(seconds):
    return Cond({"cond": "elapsed", "s": float(seconds)}, "after %.0f s" % seconds)


def _leg_name(leg):
    if isinstance(leg, LegRef):
        return leg.leg
    if leg in _LEGS:
        return leg
    raise PlanError("leg must be one of %s (or p.base etc.); got %r" % (", ".join(_LEGS), leg))


def on(leg):
    l = _leg_name(leg)
    return Cond({"cond": "on_leg", "leg": l}, "on %s" % l)


def turning(leg):
    l = _leg_name(leg)
    return Cond({"cond": "turning", "leg": l}, "turning %s" % l)


def established(leg):
    l = _leg_name(leg)
    return Cond({"cond": "established", "leg": l}, "established on %s" % l)


def on_final(distance=None):
    c = on("final")
    if distance is None:
        return c
    rwy = _live_runway()
    if rwy is None:
        raise PlanError("on_final(distance) needs a pattern runway; fly a pattern first")
    w = within(distance, rwy.threshold)
    return Cond({"cond": "all", "conds": [c.js, w.js]}, "on a %r final" % Ft(_dist(distance)))


class _Angle(object):
    def __init__(self, r):
        self.r = r

    def __ge__(self, degrees):
        return Cond({"cond": "angle_behind_wing", "target": self.r.js, "deg": float(degrees)},
                    "%s %.0f° behind the wing" % (self.r.label, degrees))


def angle_behind_wing(x):
    """Use as angle_behind_wing(rwy.threshold) >= 45."""
    return _Angle(_ref(x))


class Traffic(object):
    """Other aircraft, by the controller's description ("the Cherokee on a
    3 mile final") or a target id from status traffic. The autopilot binds
    it to a sensed target by type, leg and miles on final; until one
    matches, traffic conditions stay false and show as open items."""

    def __init__(self, description):
        self.description = description

    def candidates(self):
        return _ctx.get("traffic") or []

    def __repr__(self):
        c = self.candidates()
        if not c:
            return "<traffic %r: nothing sensed nearby>" % self.description
        seen = "; ".join("%s %s %s (%s)" % (t.get("id"), t.get("type"), t.get("where") or "", t.get("relative"))
                         for t in c[:4])
        return "<traffic %r; sensed: %s>" % (self.description, seen)


def traffic(description):
    """traffic("Cherokee on a 3 mile final"), or traffic("t2") by id."""
    return Traffic(str(description))


def in_sight(t):
    return Cond({"cond": "in_sight", "traffic": t.description}, "%s in sight" % t.description)


def passed(t):
    return Cond({"cond": "passed", "traffic": t.description}, "past %s" % t.description)


def following(t):
    """t (the traffic you follow) is on final and past you, or has landed:
    the follower's cue to turn base. p.follow(t) uses it."""
    return Cond({"cond": "following", "traffic": t.description}, "%s past you on final (or landed)" % t.description)


def behind(t, distance=None):
    js = {"cond": "behind", "traffic": t.description}
    if distance is not None:
        js["distance_ft"] = _dist(distance)
    return Cond(js, "behind %s" % t.description)


# ---------------------------------------------------------------- steps


_LATERAL = ("heading", "turn", "direct", "toward", "orbit", "pattern", "follow", "circle", "rectangular_course", "takeoff")


class Step(object):
    def __init__(self, js, text, register=True):
        self.js = js
        self.text = text
        self._until = None
        if register:
            _b.register(self)

    def until(self, cond):
        self._until = _cond(cond)
        return self

    def then(self, other):
        return Chain([self]).then(other)

    def lateral(self):
        return self.js["step"] in _LATERAL

    def to_json(self):
        js = dict(self.js)
        if self.js["step"] == "pattern":
            js["pattern"] = self.pattern_obj.spec()
        if self._until is not None:
            js["until"] = self._until.js
        return js

    def describe(self):
        return self.text + ((" until " + self._until.text) if self._until else "")

    def __repr__(self):
        return "<step %s>" % self.describe()


class Chain(object):
    """Steps flown one after another: a.then(b).then(c)."""

    def __init__(self, items):
        self.items = []
        for it in items:
            self._add(it)
        _b.register(self)

    def _add(self, it):
        if isinstance(it, Chain):
            _b.unregister(it)
            self.items.extend(it.items)
        elif isinstance(it, Step):
            _b.unregister(it)
            self.items.append(it)
        else:
            raise PlanError("then() takes a step (heading(), join(p), ...); got %r" % (it,))

    def then(self, other):
        self._add(other)
        return self

    def until(self, cond):
        self.items[-1].until(cond)
        return self

    def lateral(self):
        return True

    def __repr__(self):
        return "<chain %s>" % " -> ".join(s.describe() for s in self.items)


def heading(h=PRESENT, turn=None):
    js = {"step": "heading", "heading_deg": None, "turn": None}
    if h is RUNWAY:
        rwy = _live_runway()
        if rwy is not None:
            h = rwy.course
        elif _ctx.get("reference_runway_course_deg") is not None:
            h = float(_ctx["reference_runway_course_deg"])
        else:
            raise PlanError("heading(RUNWAY) needs a runway; use heading(<degrees>)")
    if h is not PRESENT:
        js["heading_deg"] = float(h) % 360
    if turn is not None:
        js["turn"] = _dir(turn)
    text = "present heading" if h is PRESENT else "heading %03.0f" % (float(h) % 360)
    return Step(js, text)


fly_heading = heading


def turn(direction, by=None, to=None):
    d = _dir(direction)
    if (by is None) == (to is None):
        raise PlanError("turn() needs exactly one of by= (degrees) or to= (a heading)")
    js = {"step": "turn", "dir": d}
    if by is not None:
        js["by_deg"] = float(by)
        text = "turn %s %.0f°" % (d, by)
    else:
        js["to_deg"] = float(to) % 360
        text = "turn %s heading %03.0f" % (d, float(to) % 360)
    return Step(js, text)


def direct(x):
    r = _ref(x)
    return Step({"step": "direct", "target": r.js}, "direct %s" % r.label)


overfly = direct


def toward(x):
    r = _ref(x)
    return Step({"step": "toward", "target": r.js}, "toward %s" % r.label)


def orbit(direction, degrees=360, bank=None):
    js = {"step": "orbit", "dir": _dir(direction), "degrees": float(degrees)}
    if bank is not None:
        js["bank_deg"] = float(bank)
    return Step(js, "%s %.0f" % (direction, degrees))


def steep_turn(direction, bank=45, times=1):
    """A steep turn (ACS): 360° per time at `bank` (default 45°), holding
    altitude, rolling out on the entry heading."""
    return orbit(direction, 360 * times, bank=bank)


def three_sixty(direction):
    return orbit(direction, 360)


def follow(line, direction=None, keep=None, side=None, offset=None):
    """Fly along a road or the shoreline. direction=NORTH/EAST/... picks which
    way along it (default: the way you're heading). keep=LEFT puts the line
    off your left wing; side=NORTH keeps you north of it; neither = over
    it. offset defaults to ft(2000) with keep/side. Done at the line's end;
    usually ended sooner with .until()."""
    r = _ref(line)
    if r.kind not in ("road", "line"):
        raise PlanError("follow() needs a road or line; %s is a %s" % (r.label, r.kind))
    js = {"step": "follow", "line": r.js}
    text = "follow %s" % r.label
    if direction is not None:
        js["direction_deg"] = float(direction)
        text += " %sbound" % {0.0: "north", 90.0: "east", 180.0: "south", 270.0: "west"}.get(float(direction), "%03.0f" % direction)
    if keep is not None:
        js["keep"] = _dir(keep)
        text += ", off your %s wing" % keep
    if side is not None:
        js["side_deg"] = float(side)
        text += ", %s of it" % {0.0: "north", 90.0: "east", 180.0: "south", 270.0: "west"}.get(float(side), "%03.0f side" % side)
    if offset is not None:
        js["offset_ft"] = _dist(offset, "offset")
    return Step(js, text)


def circle(x, direction=LEFT, times=None, radius=None):
    """Circle a point or area: circle(place("the pier"), LEFT, times=2).
    times=None circles until something ends the step."""
    r = _ref(x)
    js = {"step": "circle", "target": r.js, "dir": _dir(direction)}
    if times is not None:
        js["turns"] = float(times)
    if radius is not None:
        js["radius_ft"] = _dist(radius, "radius")
    return Step(js, "circle %s %s%s" % (r.label, direction, (" %g times" % times) if times else ""))


def rectangular_course(x, direction=LEFT, offset=None, laps=None, size=None, altitude=None):
    """The ground-reference rectangular course around a field (an area,
    or a `size` square on a point): fly parallel to its edges `offset`
    out (default nm(0.25)), turning LEFT or RIGHT at the corners, holding
    the track against the wind. laps=None flies it until something ends
    the step; altitude= sets the altitude to hold (e.g. agl(1000))."""
    r = _ref(x)
    if r.kind in ("road", "line"):
        raise PlanError("rectangular_course() goes around an area or point; %s is a %s" % (r.label, r.kind))
    js = {"step": "rectangular_course", "target": r.js, "dir": _dir(direction)}
    if offset is not None:
        js["offset_ft"] = _dist(offset, "offset")
    if laps is not None:
        js["laps"] = float(laps)
    if size is not None:
        js["size_ft"] = _dist(size, "size")
    if altitude is not None:
        js["altitude_ft"] = _alt(altitude)
    text = "rectangular course around %s, %s turns%s" % (r.label, _dir(direction), (", %g laps" % laps) if laps else "")
    return Step(js, text)


def hold_over(x, direction=LEFT, radius=None):
    """Hold over a place (circle it until told otherwise)."""
    return circle(x, direction, None, radius)


def s_turn(direction, by=30):
    """One S-turn: turn `by` degrees one way, then back."""
    other = RIGHT if _dir(direction) == LEFT else LEFT
    return turn(direction, by=by).then(turn(other, by=by))


def s_turns(first=LEFT, n=2, by=30):
    """S-turns to lose ground or time: n turns out and back, alternating."""
    c = s_turn(first, by)
    d = RIGHT if first == LEFT else LEFT
    for _ in range(n - 1):
        c = c.then(s_turn(d, by))
        d = RIGHT if d == LEFT else LEFT
    return c


def takeoff(rwy):
    """Take off from the runway you're lined up on. Done at 400 ft AGL,
    climbing on runway heading toward pattern altitude; follow it with
    .then(...) for the departure."""
    if not isinstance(rwy, Runway):
        raise PlanError("takeoff() takes a runway: takeoff(apt.runway(\"21\"))")
    return Step({"step": "takeoff", "airport": rwy.airport, "runway": rwy.ident}, "take off %s" % rwy.ident)


def _pattern_alt(p):
    if p.altitude_ft is not None:
        return p.altitude_ft
    return _ctx["field_elevation_ft"] + _ctx.get("pattern_defaults", {}).get("altitude_agl_ft", 1000.0)


def _pattern_width(p):
    return p.width_ft or _ctx.get("pattern_defaults", {}).get("downwind_offset_ft", 6000.0)


def _side_sign(p):
    return 1.0 if p.side == RIGHT else -1.0


def depart(p, via=STRAIGHT_OUT):
    """Leave the pattern the AFH way: climb on runway heading past the
    departure end, then STRAIGHT_OUT, a FORTY_FIVE toward the pattern side
    at pattern altitude, or a CROSSWIND / DOWNWIND departure (turn at
    least 400 ft up, within 300 ft of pattern altitude). The last heading
    holds; add .until()/.then() for what comes after."""
    if not isinstance(p, Pattern):
        raise PlanError("depart() takes a pattern: depart(pattern(rwy, LEFT), via=DOWNWIND)")
    rwy, s, side = p.rwy, _side_sign(p), p.side
    course = rwy.course
    alt = _pattern_alt(p)
    out = heading(course)
    if via is STRAIGHT_OUT:
        return out
    if via is FORTY_FIVE:
        return out.until(past(rwy.departure_end) & at_or_above(alt - 100)).then(heading((course + s * 45) % 360, turn=side))
    if via in (CROSSWIND, DOWNWIND):
        c = out.until(past(rwy.departure_end) & agl_above(400) & at_or_above(alt - 300)).then(
            heading((course + s * 90) % 360, turn=side))
        if via is DOWNWIND:
            c = c.until(distance_flown(ft(_pattern_width(p)))).then(heading((course + 180) % 360, turn=side))
        return c
    raise PlanError("depart via must be STRAIGHT_OUT, FORTY_FIVE, CROSSWIND or DOWNWIND")


def climb(to, at=None):
    js = {"step": "climb", "altitude_ft": _alt(to)}
    if at is not None:
        js["speed_kt"] = _speed(at)
    return Step(js, "climb to %.0f" % js["altitude_ft"])


def descend(to, at=None):
    js = {"step": "descend", "altitude_ft": _alt(to)}
    if at is not None:
        js["speed_kt"] = _speed(at)
    return Step(js, "descend to %.0f" % js["altitude_ft"])


def maintain(alt):
    a = _alt(alt)
    return Step({"step": "maintain", "altitude_ft": a}, "maintain %.0f" % a)


def speed(v):
    s = _speed(v)
    return Step({"step": "speed", "kt": s}, "speed %.0f kt" % s)


# ---------------------------------------------------------------- the pattern


class LegRef(object):
    def __init__(self, pattern, leg):
        self.pattern = pattern
        self.leg = leg

    def __repr__(self):
        return "<%s leg of %s>" % (self.leg, self.pattern.describe())


class Pattern(object):
    """The traffic pattern for a runway. Edits change it in place and return
    it, so they chain: pattern(rwy, RIGHT).width(TIGHT).extend("downwind", nm(1))."""

    def __init__(self, rwy, side, spec=None, live=False):
        self.rwy = rwy
        self.side = _dir(side)
        self.live = live
        self.dirty = False
        spec = spec or {}
        self.altitude_ft = spec.get("altitude_ft")
        self.width_ft = spec.get("width_ft")
        self.extends = dict((k, v) for k, v in (spec.get("extend") or {}).items() if v is not None)
        self.turns = dict((k, v) for k, v in (spec.get("turns") or {}).items() if v is not None)
        self.squared = list(spec.get("square") or [])
        self._turn_text = {}

    def _touch(self):
        self.dirty = True
        return self

    def spec(self):
        js = {"airport": self.rwy.airport, "runway": self.rwy.ident, "side": self.side,
              "extend": {k: v for k, v in self.extends.items() if v is not None},
              "turns": self.turns}
        if self.altitude_ft is not None:
            js["altitude_ft"] = self.altitude_ft
        if self.width_ft is not None:
            js["width_ft"] = self.width_ft
        if self.squared:
            js["square"] = list(self.squared)
        return js

    def extend(self, leg, distance=OPEN, more=False):
        """Extend upwind/crosswind/downwind by a distance, or OPEN (until a
        turn trigger or a later call says otherwise). more=True adds to
        the current extension."""
        if leg not in ("upwind", "crosswind", "downwind"):
            raise PlanError("only upwind, crosswind and downwind extend; got %r" % (leg,))
        if distance is OPEN:
            self.extends[leg] = "open"
        else:
            d = _dist(distance)
            cur = self.extends.get(leg)
            if more and isinstance(cur, (int, float)):
                d += cur
            self.extends[leg] = d
        return self._touch()

    def extension(self, leg):
        v = self.extends.get(leg)
        return OPEN if v == "open" else Ft(v or 0.0)

    def width(self, w):
        default = _ctx.get("pattern_defaults", {}).get("downwind_offset_ft", 6000.0)
        if w is TIGHT:
            self.width_ft = default * 0.6
        elif w is WIDE:
            self.width_ft = default * 1.4
        elif w is NORMAL:
            self.width_ft = None
        else:
            self.width_ft = _dist(w, "width")
        return self._touch()

    def altitude(self, alt):
        self.altitude_ft = _alt(alt)
        return self._touch()

    def turn(self, leg, when=None, earliest=None, latest=None):
        """When to turn onto `leg` (crosswind, downwind, base, final).
        when= replaces the normal turn point; earliest=/latest= bound it."""
        l = _leg_name(leg)
        if l not in _TURN_LEGS:
            raise PlanError("turn() legs are %s; got %r" % (", ".join(_TURN_LEGS), leg))
        if when is None and earliest is None and latest is None:
            raise PlanError("turn(%r) needs when=, earliest= or latest=" % l)
        t = {}
        parts = []
        for k, v in (("when", when), ("earliest", earliest), ("latest", latest)):
            if v is not None:
                t[k] = _cond(v).js
                parts.append("%s %s" % ({"when": "", "earliest": "not before", "latest": "no later than"}[k], v.text))
        self.turns[l] = t
        self._turn_text[l] =" ".join(p.strip() for p in parts)
        return self._touch()

    def final_length(self, d):
        """How long a final to fly: SHORT (base abeam the threshold, a short
        approach), NORMAL (the usual base turn), or a distance (base when
        abeam that far out on the extended centerline)."""
        if d is NORMAL:
            self.turns.pop("base", None)
            self._turn_text.pop("base", None)
            return self._touch()
        dist = 0.0 if d is SHORT else _dist(d, "final length")
        self.turns["base"] = {"when": past(self.rwy.at(along=ft(-dist), right=ft(0))).js}
        self._turn_text["base"] = "short approach (base abeam the threshold)" if d is SHORT else "final about %r long" % Ft(dist)
        return self._touch()

    def square(self, leg="base"):
        """Fly a leg's corners square: a definite turn that rolls out on the
        next leg, no rounding off or cutting in. "base" squares the turns
        onto and off base; "final" just the turn onto final."""
        l = _leg_name(leg)
        if l not in ("base", "final"):
            raise PlanError("square() takes \"base\" or \"final\"; got %r" % (leg,))
        if l not in self.squared:
            self.squared.append(l)
        return self._touch()

    def follow(self, t):
        """Number 2 behind traffic t: keep the downwind going until t is on
        final and past you (or has landed), then turn base, not before the
        approach end. ATC can still call the base: release("base")."""
        if not isinstance(t, Traffic):
            raise PlanError("p.follow() takes traffic(...), e.g. p.follow(traffic(\"Cherokee on a 3 mile final\"))")
        self.extends["downwind"] = "open"
        self.turns["base"] = {"when": (following(t) | atc("base")).js, "earliest": past(self.rwy.threshold).js}
        self._turn_text["base"] = "following %s (base when it's past you on final, or on ATC's call)" % t.description
        return self._touch()

    upwind = property(lambda self: LegRef(self, "upwind"))
    crosswind = property(lambda self: LegRef(self, "crosswind"))
    downwind = property(lambda self: LegRef(self, "downwind"))
    base = property(lambda self: LegRef(self, "base"))
    final = property(lambda self: LegRef(self, "final"))

    def describe(self):
        bits = ["%s traffic %s %s" % (self.side, self.rwy.airport, self.rwy.ident)]
        for k, v in sorted(self.extends.items()):
            bits.append("%s extended %s" % (k, "open" if v == "open" else repr(Ft(v))))
        for k in _TURN_LEGS:
            if k in self.turns:
                bits.append("turn %s: %s" % (k, self._turn_text.get(k) or "custom"))
        if self.width_ft:
            bits.append("width %r" % Ft(self.width_ft))
        if "base" in self.squared:
            bits.append("square base")
        elif "final" in self.squared:
            bits.append("square turn to final")
        return ", ".join(bits)

    def __repr__(self):
        return "<pattern %s>" % self.describe()


def pattern(rwy, side=LEFT):
    if not isinstance(rwy, Runway):
        raise PlanError("pattern() takes a runway: pattern(airport().runway(\"16R\"), RIGHT)")
    return Pattern(rwy, side)


def _pattern_step(p, entry=None, land_kind=FULL_STOP, text=None):
    if not isinstance(p, Pattern):
        raise PlanError("expected a pattern, e.g. join(p) or join(pattern(rwy, RIGHT)); got %r" % (p,))
    s = Step({"step": "pattern", "entry": entry, "land": land_kind}, text or "fly the pattern, %s" % p.describe())
    s.pattern_obj = p
    return s


def join(p, land=FULL_STOP, on_go=None):
    """Join the pattern from wherever the aircraft is, fly it and land.
    land=TOUCH_AND_GO (or OPTION) / LOW_APPROACH keeps flying the pattern
    afterwards unless on_go gives what to do instead
    (on_go=depart(p, via=STRAIGHT_OUT))."""
    step = _pattern_step(p, None, land, "join %s" % p.describe())
    if on_go is None:
        return step
    if land == FULL_STOP:
        raise PlanError("on_go needs land=TOUCH_AND_GO, OPTION or LOW_APPROACH")
    return step.until(on("upwind") & agl_above(400)).then(on_go)


circuit = join


def enter(leg, via=None, land=FULL_STOP):
    """Enter the pattern on a leg: enter(p.downwind), enter(p.base).
    Downwind entries: via=FORTY_FIVE (the standard 45 to midfield),
    MIDFIELD_CROSSWIND (cross midfield at pattern altitude from the other
    side), or OVERHEAD (cross midfield 500 ft above, continue 2 nm, turn
    back to the 45, descending). Without via the autopilot picks the way in."""
    if not isinstance(leg, LegRef):
        raise PlanError("enter() takes a pattern leg, e.g. enter(p.downwind)")
    if leg.leg not in ("downwind", "base", "final"):
        raise PlanError("enter() supports downwind, base and final; got %s" % leg.leg)
    p = leg.pattern
    step = _pattern_step(p, leg.leg, land, "enter %s on %s" % (p.describe(), leg.leg))
    if via is None:
        return step
    if leg.leg != "downwind":
        raise PlanError("via= entries are for the downwind")
    rwy, s, w = p.rwy, _side_sign(p), _pattern_width(p)
    mid = float(rwy.length) / 2
    on_45 = rwy.at(along=ft(2.1 * w), right=ft(s * 2 * w))
    if via is FORTY_FIVE:
        return direct(on_45).then(step)
    if via is MIDFIELD_CROSSWIND:
        return direct(rwy.at(along=ft(mid), right=ft(-s * 0.8 * w))).then(
            direct(rwy.at(along=ft(mid), right=ft(s * w)))).then(step)
    if via is OVERHEAD:
        alt = _pattern_alt(p)
        return maintain(alt + 500).then(direct(rwy.at(along=ft(mid), right=ft(-s * 0.5 * w)))).then(
            direct(rwy.at(along=ft(mid), right=ft(s * (w + 2 * NM_FT))))).then(descend(alt).until(NOW)).then(
            direct(on_45)).then(step)
    raise PlanError("enter via must be FORTY_FIVE, MIDFIELD_CROSSWIND or OVERHEAD")


def straight_in(rwy, side=LEFT, land=FULL_STOP):
    return enter(Pattern(rwy, side).final, land)


def land(rwy=None, kind=FULL_STOP, on_go=None, side=None):
    """Land on rwy (default: the live pattern's runway) from wherever you
    are, flying its pattern. kind=TOUCH_AND_GO / OPTION / LOW_APPROACH,
    with on_go= for what to do after."""
    p = _b.live_pattern
    if rwy is not None and (p is None or p.rwy.ident != rwy.ident):
        p = Pattern(rwy, side or (p.side if p is not None else LEFT))
    if p is None:
        raise PlanError("land() needs a runway or an active pattern")
    return join(p, kind, on_go)


def go_around():
    """Go around now. The pattern continues (climb out, crosswind, around
    again) unless the same call gives new steps."""
    _b.go_around = True


# ---------------------------------------------------------------- constraints and reports


class Constraint(object):
    def __init__(self, cond, soft=False):
        c = _cond(cond)
        if c.js["cond"] not in _STATE_CONDS:
            raise PlanError("remain() takes a state (at_or_below(), north_of(), over(), outside(), ...); "
                            "%r is an event" % c.text)
        self.id = _b.new_id("c")
        self.cond = c
        self.soft = soft
        self._until = None
        self._after = None
        _b.constraints.append(self)

    def until(self, cond):
        self._until = _cond(cond)
        return self

    def after(self, cond):
        self._after = _cond(cond)
        return self

    def text(self):
        t = ("prefer " if self.soft else "remain ") + self.cond.text
        if self._after:
            t += " after " + self._after.text
        if self._until:
            t += " until " + self._until.text
        return t

    def to_json(self):
        return {"id": self.id, "text": self.text(), "pred": self.cond.js,
                "until": self._until.js if self._until else None,
                "after": self._after.js if self._after else None, "soft": self.soft}

    def __repr__(self):
        return "<%s: %s>" % (self.id, self.text())


_STATE_CONDS = ("at_or_below", "at_or_above", "side_of", "over", "inside", "outside", "within", "any", "all", "not")


def remain(cond):
    return Constraint(cond)


def prefer(cond):
    return Constraint(cond, soft=True)


def permit(x):
    """Record an option ATC granted ("S-turns approved to the west",
    "right turn at the shoreline approved"). Nothing is flown; use it
    yourself when you need it."""
    if isinstance(x, (Step, Chain)):
        _b.unregister(x)
        text = x.describe() if isinstance(x, Step) else repr(x)
    elif isinstance(x, Constraint):
        _b.constraints = [c for c in _b.constraints if c is not x]
        text = x.text()
    elif isinstance(x, Cond):
        text = x.text
    else:
        text = str(x)
    _b.permits.append(text)


def can(*_things):
    """Ask, don't fly: can(p.final_length(SHORT)) answers "can you make a
    short approach?". The whole call becomes a preview: the autopilot
    flies the change ahead in a simulation and reports what would happen
    (timeline, touchdown, traffic, problems), changing nothing. If it
    looks flyable, say "able" and fly the same code without can()."""
    _b.preview = True


def report(text, when):
    rid = _b.new_id("r")
    _b.reports.append({"id": rid, "text": str(text), "when": _cond(when).js})
    return rid


def release(token):
    """Open an atc() gate: release("base") after "turn base"."""
    _b.release.append(str(token))


def cancel(item_id):
    """Drop a constraint or report by id (see plan.constraints)."""
    _b.cancel.append(str(item_id))


class _Plan(object):
    """The live plan. plan.replace() starts over (new clearance)."""

    @property
    def constraints(self):
        return list((_ctx.get("plan") or {}).get("constraints", []))

    @property
    def reports(self):
        return list((_ctx.get("plan") or {}).get("reports", []))

    @property
    def steps(self):
        return list((_ctx.get("plan") or {}).get("steps", []))

    @property
    def active(self):
        return (_ctx.get("plan") or {}).get("active")

    @property
    def open_items(self):
        return list((_ctx.get("plan") or {}).get("open_items", []))

    @property
    def pattern(self):
        return _b.live_pattern

    def replace(self, *_items):
        """Amend instructions: drop earlier constraints and reports."""
        _b.replace = True

    def change_runway(self, rwy, land=FULL_STOP):
        """Sidestep to rwy: the same pattern side, flown from here."""
        p = _b.live_pattern
        return join(Pattern(rwy, p.side if p is not None else LEFT), land)

    clear = replace

    def __repr__(self):
        pl = _ctx.get("plan") or {}
        if not pl:
            return "<plan: none>"
        return "<plan: step %s of %s; constraints %s>" % (
            pl.get("active"), [s for s in pl.get("steps", [])], [c["id"] + " " + c["text"] for c in pl.get("constraints", [])])


plan = _Plan()


def _live_runway():
    if _b.live_pattern is not None:
        return _b.live_pattern.rwy
    return None


# ---------------------------------------------------------------- namespace + finish


def _speeds():
    s = _ctx.get("speeds", {})
    return dict((k, float(v)) for k, v in s.items())


def namespace():
    """Names visible to plan code."""
    ns = {}
    for k, v in globals().items():
        if k.startswith("_") or k in ("difflib", "math", "namespace", "NM_FT"):
            continue
        ns[k] = v
    ns.update(_speeds())
    live = (_ctx.get("plan") or {}).get("pattern")
    here = _ctx.get("airport")
    if here:
        ns["apt"] = Airport(here)
    if live:
        rwy = Airport(live["airport"]).runway(live["runway"])
        p = Pattern(rwy, live["side"], live, live=True)
        _b.live_pattern = p
        ns["p"] = p
        ns["rwy"] = rwy
    return ns


def _finish():
    tops = list(_b.top)
    chains = [t for t in tops if isinstance(t, Chain) or (isinstance(t, Step) and t.lateral())]
    directives = [t for t in tops if isinstance(t, Step) and not t.lateral()]
    if len(chains) > 1:
        raise PlanError("one call can start only one step sequence; join steps with .then(): %s"
                        % ", ".join(repr(c) for c in chains))
    steps = None
    if chains:
        c = chains[0]
        steps = [s.to_json() for s in (c.items if isinstance(c, Chain) else [c])]
    delta = {
        "steps": steps,
        "directives": [d.to_json() for d in directives],
        "pattern": _b.live_pattern.spec() if (_b.live_pattern is not None and _b.live_pattern.dirty) else None,
        "constraints": [c.to_json() for c in _b.constraints],
        "reports": _b.reports,
        "release": _b.release,
        "cancel": _b.cancel,
        "replace": _b.replace,
        "go_around": _b.go_around,
        "permits": _b.permits,
        "preview": _b.preview,
    }
    return delta
