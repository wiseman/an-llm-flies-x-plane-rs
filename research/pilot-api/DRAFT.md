# A pilot-vocabulary API for agent-authored guidance — first draft

Status: exploratory design draft, 2026-10-15. Nothing here is implemented.

The idea: instead of a fixed menu of parameterized profiles, the agent writes
short Python that builds a **plan** out of primitives named the way pilots and
controllers talk — Vy, abeam the numbers, extend downwind, remain south of the
101 until the interchange. A deterministic Rust executor flies the plan. Agent
code runs when a clearance arrives, never inside the control loop.

This draft was built backwards from real instructions:

- **ATC**: 1.22M transmissions (16,231 Whisper transcripts of LiveATC recordings
  from 26 facilities, mostly SoCal GA towers — VNY, WHP, SMO, EMT, BUR, plus
  LAX, ONT, BIL, CHO, PHNL and others). See `../atc-corpus/`.
- **Instructors**: the FAA *Airplane Flying Handbook* chapters on takeoffs,
  ground reference maneuvers, traffic patterns, and landings, and the Private
  Pilot ACS for tolerances. See `../afh/`.

116 ATC instructions and 25 instructor items were curated into
`instructions.yaml`. Every one is translated below. Where a translation is
awkward, that's recorded as a gap.

---

## 1. What the corpus says

### How often the structures occur

Rough regex counts over all 1.22M transmissions (`../atc-corpus/structure.py`).
Whisper noise means these undercount; treat them as orders of magnitude.

| Structure | Count | Example |
|---|---:|---|
| Maneuver permission ("…approved", "you can…", "at your discretion") | 19,183 | "right turn at the shoreline approved" |
| Sequencing ("number 2, following…") | 11,317 | "number 2, follow the Cessna on a 2 mile final" |
| Altitude window ("at or above/below") | 5,529 | "remain at or below 2500" |
| Question to pilot ("can you…", "are you able…") | 4,959 | "are you able to turn base at or before the freeway?" |
| Soft request ("if able", "when able") | 4,375 | "if able, keep it over the 101" |
| Sequence ("…then <action>") | 2,970 | "fly present heading to the 101, then make right traffic" |
| Report traffic/field in sight | 2,945 | |
| Extend upwind/downwind/crosswind | 2,742 | |
| **Constraint with end condition** ("remain … until …") | 2,385 | "remain at or below 1800 until north of the field" |
| Short approach | 2,215 | |
| Side of a line ("remain north of…", "west side of…") | 1,697 | "remain on the west side of the 405" |
| Change to runway (sidestep) | 1,627 | |
| Left/right 360 or 270 | 1,608 | |
| "I'll call your base/turn" | 1,591 | |
| Follow a linear feature | 1,553 | "follow the 5, enter right downwind" |
| Relative turn ("turn 20 degrees left") | 1,433 | |
| "On the go, …" | 1,291 | |
| Circle / orbit / hold over a place | 913 | "cross midfield, hold over the business park" |
| Turn a leg at/over/abeam/before a place | 420 | "turn base at the VA hospital" |

### Twelve findings

1. **Most clearances amend a plan already in progress.** "Extend downwind",
   "turn base now", "change to 16L", and "keep your base in close" all assume an
   active pattern. The plan has to be a live object with named parts that the
   agent edits, not something rebuilt from scratch for each call.

2. **Pattern turn timing is where most of the editing happens.** The same
   turn can be ordered with many kinds of trigger:
   - **now:** "turn base now"
   - **as soon as able:** "turn base as soon as you're able"
   - **at a place:** "at the 118 turn base", "turn base at the VA"
   - **abeam a place:** "turn base abeam the VA hospital"
   - **a deadline:** "prior to the interchange", "at or before the freeway"
   - **an earliest point:** "take it out to the shoreline before you turn crosswind"
   - **ATC's call:** "I'll call your base"
   - **traffic:** "just behind the traffic, turn base"; "don't turn base now, you're following the Cherokee"

   So every pattern turn needs a replaceable trigger, plus earliest and
   latest bounds.

3. **Constraints come with expiry conditions.** "Remain at or below 1800 until
   north of the field", "remain on the east side of the 405 until the
   interchange", "maintain 2000 until you turn base". `remain(X).until(Y)`
   needs to be first-class, and the executor needs to enforce it.

4. **Places are local nicknames, not identifiers.** Examples: "the 118", "the
   flood basin", "the VA", "the new Getty", "Warner Center", "Topanga", "the
   interchange". Resolving them needs OSM-style geography plus airport-local
   context. apt.dat has none of this.

5. **Lines matter as much as points.** Freeways, the shoreline, ridge lines,
   the runway centerline and the Santa Monica final are all lines.
   Instructions use side-of-line ("north of the 101"), crossing ("cross the 10,
   then turn crosswind"), and line-following with a side ("follow the 405,
   keep it off your left wing"). Freeways aren't straight, so "north of the
   101" has to be defined against the local stretch of road.

6. **Traffic is a first-class noun.** "Follow the Cherokee", "number 2", "pass
   behind that traffic", and "report in sight" are among the most common
   instructions. Followers time the base turn off the traffic ahead. ATC also
   describes other aircraft's intentions in the same vocabulary ("he's going
   to circle the pier a couple times, then head out to the Hollywood sign").
   So the plan language doubles as a way to represent other aircraft's
   expected paths.

7. **Commands, permissions, preferences and questions are different speech
   acts.** "Make a right 360" is a command. "S-turns approved to the west if
   you need them" is a permission. "If able, keep it over the 101" is a soft
   request. "Can you make a short approach?" is a question. Each needs its own
   construct (`do`, `permit`, `prefer`, `can`). A question is answered by
   checking whether the amended plan is feasible.

8. **Qualitative words need per-aircraft definitions.** For example: tight,
   wide, square, "in close", short approach, slowest practical speed, best
   forward speed, expedite, immediate. These belong in aircraft config, the
   same way V-speeds do.

9. **Many instructions never say when they end.** "Fly runway heading",
   "maintain at or above 4000", "extend downwind", "for now". The plan should
   mark these items as open-ended, so the agent knows a follow-up is expected
   and can ask before an extension runs too long.

10. **ATC instructions override training defaults, and that has
    consequences.** "Maintain pattern altitude until you turn final" overrides
    the AFH's "begin descent abeam the numbers". The result is a steep final
    that may need full flaps or a slip, or may need an "unable". Feasibility
    checks and the ability to say "unable" are part of the API, not extras.

11. **Radio calls are plan steps too.** "Report 3 miles", "report over the
    interchange", "contact SoCal prior to entering Class C", "after you clear
    the D, frequency change approved". These are triggers whose action wakes
    the agent to speak.

12. **Instructor knowledge becomes defaults, tolerances and gates.** The AFH
    pattern written in this vocabulary is the standard library. ACS tolerances
    define "established" and "deviation". The stabilized-approach rule
    ("stabilized by 500 ft or go around") is a gate: a check at a point,
    with a fallback action.

---

## 2. The vocabulary

Python, because it's what the model writes most reliably. The agent's code
**builds a plan value**. Nothing here blocks or touches actuators. The executor
validates the plan, previews it in the simple sim, and flies it.

### 2.1 References (nouns)

References resolve once to geometry, and the agent can inspect them
(`print(place("the VA"))` shows the name, coordinates, and distance and bearing
from the airport).

```python
apt = airport("KVNY")
rwy = apt.runway("16R")

# Runway anatomy (points unless noted)
rwy.threshold, rwy.numbers, rwy.aim_point, rwy.departure_end, rwy.midfield
rwy.centerline, rwy.extended_centerline          # lines
rwy.exit("H"), rwy.intersection("D"), rwy.end    # ground references

# Geography (resolved via OSM + airport-local aliases)
place("Encino Reservoir")                        # point or area
place("the VA", near=apt)                        # local nickname; resolver picks the nearby VA hospital
road("the 118")                                  # line (CA-118 near this airport)
interchange("I-405", "US-101")                   # point
shoreline(), water(), land()                     # line / areas
ridge("Santa Monica Mountains")                  # line
airspace("KBUR", "C"), apt.class_d               # volumes

# Traffic: binds to a sensed target when possible, otherwise unbound/estimated
t = traffic("Cherokee on a 3 mile final")

# Speeds, by name (from aircraft config)
Vr, Vx, Vy, Vref, Vapp, Vbg, Vso, PATTERN_SPEED
SLOWEST_PRACTICAL, BEST_FORWARD                  # phase-dependent, aircraft-defined

# Units — bare numbers are MSL feet / knots / degrees
ft(1800), agl(1000), nm(1), kt(90), deg(20)
```

### 2.2 The pattern as a value

The pattern is the main object, because most clearances edit it. It's built
from config (pattern altitude, offset) and is immutable. Edits return a new
pattern, so the agent and the operator can diff and preview a change before it
takes effect.

```python
p = pattern(rwy, side=RIGHT)                     # standard rectangle per AFH ch8

p.upwind, p.crosswind, p.downwind, p.base, p.final     # legs
p.abeam(rwy.numbers), p.key_45, p.entry_45, p.midfield_crosswind  # derived points

# Shape
p.extend("downwind", nm(1))                      # also "upwind", "crosswind"
p.extend("downwind", OPEN)                       # until a trigger says otherwise
p.width(TIGHT | WIDE | nm(0.75))
p.square("base")                                 # no corner-cutting into or out of base
p.final_length(SHORT | nm(0.5))                  # "keep your base in close", "short approach"
p.base_through(point)                            # base leg passes over a place

# Turn timing: each pattern turn has one trigger and optional earliest/latest bounds
p.turn("base", when=crossing(road("the 118")))
p.turn("base", when=abeam(place("the VA")))
p.turn("base", latest=crossing(interchange(...)))       # "prior to"
p.turn("crosswind", earliest=crossing(shoreline()))     # "not before"
p.turn("base", when=NOW | ASAP | atc("base"))
```

Defaults come from the AFH, so with no edits `p` flies the textbook pattern:

- **Crosswind turn:** past the departure end, within 300 ft of pattern altitude.
- **Downwind:** 0.5–1 nm out.
- **Descent:** start abeam the threshold.
- **Base turn:** when the threshold is 45° behind the wing.

### 2.3 Steps (verbs)

A plan is a sequence of steps. Each step has targets on one or more axes, an
optional `.until(cond)`, and an optional `.then(step)`.

```python
# Lateral
heading(270) / heading(PRESENT) / heading(RUNWAY)
turn(LEFT, by=20) / turn(RIGHT, to=170)
direct(point) / toward(point)                    # toward = heading at it, no overfly required
track(course) / fly(leg_or_line)
follow(line, direction=EAST, keep=LEFT)          # "keep the 405 off your left wing" (where the line is, body-relative)
follow(line, direction=EAST, side=NORTH)         # "stay north of the freeway, follow it" (where the aircraft is)
overfly(point)
orbit(LEFT, 360, size=NORMAL | WIDE)
circle(point, times=2)
hold_over(point)
s_turn(RIGHT) / s_turns(side=WEST)

# Vertical / speed / configuration
climb(to=ft(3500), at=Vy, rate=NORMAL | EXPEDITE)
descend(to=..., fpm=500)
maintain(ft(2000))
glidepath(deg(3), aim=rwy.aim_point)
speed(Vy | kt(90) | at_most(kt(170)) | SLOWEST_PRACTICAL)
flaps(10) / power(rpm=1500) / power(IDLE) / slip(FORWARD)

# Standard library (written in the primitives above; see section 4)
takeoff(rwy, kind=NORMAL | SHORT | SOFT, immediate=False)
depart(p, via=STRAIGHT_OUT | FORTY_FIVE | CROSSWIND | DOWNWIND)
circuit(p)                                       # one full pattern
enter(p.downwind, via=FORTY_FIVE | MIDFIELD_CROSSWIND | OVERHEAD)
join(p)                                          # from present position, planner picks the geometry
straight_in(rwy)
land(rwy, kind=FULL_STOP | TOUCH_AND_GO | STOP_AND_GO | LOW_APPROACH | OPTION)
go_around()
rollout(exit=rwy.exit("H") | NEXT | END, turn=LEFT)
taxi(via=["B", "C"], hold_short=apt.runway("16L"), urgency=NORMAL | EXPEDITE)
line_up(rwy, at=rwy.intersection("D")) / hold_position()
```

### 2.4 Conditions (when)

```python
# Geometric
at(point)            # overhead if on the track, else closest approach
abeam(point)
crossing(line)
past(point_or_line)
north_of(x) / side_of(line, WEST)
over(area_or_line)
inside(area) / outside(area)
within(nm(2), point) / on_final(nm(2))
distance_flown(nm(10))
angle_behind_wing(point) >= deg(45)

# Aircraft state
reaching(alt), positive_rate(), at_speed(Vy)
agl_below(500), established(leg), turning(p.base), stabilized()

# Traffic
in_sight(t), abeam(t), passed(t), behind(t, nm(1))

# Radio / ATC
atc("base")          # gate opened by a controller call
advised()            # ATC lifts an instruction ("until advised")

# Constants and combinators
NOW, ASAP
a & b, a | b, ~a
```

### 2.5 Constraints, gates, speech acts

```python
remain(at_or_below(ft(1800))).until(north_of(apt))
remain(side_of(road("I-405"), EAST)).until(at(interchange("I-405", "US-101")))
remain(over(land()))
remain(at_or_below(2500) | west_of(place("Topanga")))   # disjunction, from E8

gate(agl(500), require=stabilized(), otherwise=go_around())   # AFH stabilized-approach rule

permit(s_turns(side=WEST))          # granted option; the agent may use it
prefer(over(road("US-101")))        # "if able": a soft constraint
can(p.turn("base", latest=crossing(road("the freeway"))))  # feasibility query → "able"/"unable"

report("3 miles", when=within(nm(3), rwy.threshold))       # wakes the agent to transmit
contact("SoCal", when=within(nm(2), airspace("KBUR", "C")))
```

Every target carries a default tolerance from the ACS (altitude ±100 ft,
heading ±10°, airspeed ±10 kt, approach speed +10/−5 kt). `established()`,
violation events, and sim-preview pass/fail all use them.

### 2.6 Working on the live plan

```python
plan.active           # the step in progress
plan.pattern          # handle to the active pattern, if any
plan.amend(...)       # replace part of the plan; previewed before it's committed
plan.replace(...)     # "amend instructions" / new clearance
plan.open_items()     # open-ended items awaiting a follow-up call
```

---

## 3. Worked examples

`p` is the active pattern, `rwy` is its runway, and `t` is the traffic named in
the instruction. "Gap" marks something the vocabulary handles poorly or
doesn't handle.

### Pattern shape

| ID | Instruction | Translation |
|---|---|---|
| A1 | Extend downwind, I'll call your base. | `p.extend("downwind", OPEN); p.turn("base", when=atc("base"))` |
| A2 | Extend downwind one mile. | `p.extend("downwind", nm(1))` |
| A3 | Extend about another half mile on the upwind before you turn crosswind. | `p.extend("upwind", nm(0.5))` |
| A4 | Extend your upwind, I'll call your crosswind. | `p.extend("upwind", OPEN); p.turn("crosswind", when=atc("crosswind"))` |
| A5 | Extend downwind one more mile and square your left base turn and final. | `p.extend("downwind", nm(1)); p.square("base")` |
| A6 | Keep your base in close. | `p.final_length(SHORT)` |
| A7 | Keep it wide on the downwind (no-radio straight-in traffic). | `p.width(WIDE)` |
| A8 | Keep your base square. | `p.square("base")` |
| A9 | Make a tight right pattern. | `p = pattern(rwy, RIGHT).width(TIGHT)` |

Gaps:

- **No limit on open extensions.** `OPEN` extensions need a cap, or a point
  at which the agent asks. Otherwise a missed "I'll call your base" flies the
  aircraft to the next county.
- **TIGHT / WIDE / SHORT / square are undefined.** They need numbers per
  aircraft.
- **The executor cuts corners today.** The L1 follower curves into base and
  final, so "square" means a constrained turn radius, not just geometry.

### Turn timing

| ID | Instruction | Translation |
|---|---|---|
| B1 | At the 118, turn base. | `p.turn("base", when=crossing(road("the 118")))` |
| B2 | Turn base at the Veterans Hospital. | `p.turn("base", when=at(place("VA hospital", near=apt)))`, where `at` is abeam because the place is off the track |
| B3 | Turn base abeam the VA hospital. | `p.turn("base", when=abeam(place("the VA")))` |
| B4 | Are you able to turn base at or before the freeway? | `can(p.turn("base", latest=crossing(road("the freeway"))))`, then apply it if able |
| B5 | Turn base prior to the interchange. | `p.turn("base", latest=crossing(interchange(...)))` |
| B6 | Cross the 10 freeway, then turn crosswind. | `p.turn("crosswind", when=crossing(road("I-10")))` |
| B7 | You can turn crosswind at the shoreline. | `permit(p.turn("crosswind", when=crossing(shoreline())))` |
| B8 | Take it out to the shoreline before you turn crosswind. | `p.turn("crosswind", earliest=crossing(shoreline()))` |
| B9 | Turn base now. | `p.turn("base", when=NOW)` |
| B10 | Turn base as soon as you're able. | `p.turn("base", when=ASAP)` |
| B11 | Don't turn base now; you're following the Cherokee across from you on base. | `t = traffic("Cherokee on base, opposite side"); p.follow(t)`; the base trigger becomes `passed(t)` |
| B12 | Over the VA, turn base, number 2, follow the Cherokee turning base. | `p.turn("base", when=at(place("the VA"))); p.follow(traffic("Cherokee turning base"))` |

Gaps:

- **"At" is ambiguous** for points off the flight path. The resolver should
  decide between overhead and abeam, and show which it chose.
- **"The freeway" and "the interchange" have no name.** Resolving them needs
  geometric context: the freeway the base leg would cross.
- **What `follow(t)` does to the base turn needs defining.** A standard
  technique is to turn base when the traffic being followed is abeam or
  behind the wing.

### Entries and arrivals

| ID | Instruction | Translation |
|---|---|---|
| C1 | Enter left base, runway 34L. | `enter(pattern(apt.runway("34L"), LEFT).base)` |
| C2 | Fly over the field, then enter right downwind, runway 21. | `overfly(apt).then(enter(p.downwind))` |
| C3 | Cross midfield, circle east for left traffic, runway 12. | `overfly(rwy.midfield).then(join(p, from_side=EAST))` |
| C4 | Turn towards Chatsworth Reservoir, then enter right base from there. | `toward(place("Chatsworth Reservoir")).then(enter(p.base))` |
| C5 | Remain outside Burbank's Class C; enter left base for 12 over the 210/118 interchange. | `remain(outside(airspace("KBUR","C"))); enter(p.base_through(interchange("I-210","CA-118")).base)` |
| C6 | Report over the 210/118 interchange for a left base 12, number 2 following a Mooney on a 2 mile final. | `ix = interchange("I-210", "CA-118"); report("over the interchange", when=at(ix)); enter(p.base); p.follow(traffic("Mooney, 2 mile final"))` |
| C7 | Turn 20° left, remain north of the 118, then enter right base 16L. | `turn(LEFT, by=20); remain(north_of(road("the 118"))); then enter(p.base)` |
| C8 | Turn 15° left, stay north of the freeway, follow it eastbound, then make right traffic 34L. | `turn(LEFT, by=15).then(follow(road("the freeway"), direction=EAST, side=NORTH)).then(join(p))` |
| C9 | Fly present heading to the 101, then make right traffic. | `heading(PRESENT).until(crossing(road("US-101"))).then(join(p))` |
| C10 | Remain on the east side of the 405 until the interchange, then you can make straight in 34R. | `ix = interchange("I-405", "US-101"); remain(side_of(road("I-405"), EAST)).until(at(ix)); permit(straight_in(apt.runway("34R")))` |
| C11 | Make straight in runway 12, report 3 miles. | `straight_in(rwy); report("3 miles", when=within(nm(3), rwy.threshold))` |
| C12 | Maintain at or above 4000, overfly the field and make right traffic 21. | `remain(at_or_above(4000)).until(???); overfly(apt).then(join(p))` |
| C13 | Fly direct to midfield, cross midfield, enter right downwind 34R. | `direct(rwy.midfield).then(overfly(rwy.midfield)).then(enter(p.downwind))` |
| C14 | Turn right, fly direct to the 405/101 interchange. | `direct(interchange("I-405", "US-101"))` (turn direction `RIGHT` kept as a hint) |
| C15 | Amend instructions: turn 20° left, overfly Encino Reservoir to make right traffic. | `plan.replace(turn(LEFT, by=20).then(overfly(place("Encino Reservoir"))).then(join(p)))` |
| C16 | Once you cross midfield, make a right turn around the tower, cleared to land. | `when(past(rwy.midfield), circle(apt.tower, RIGHT).until(aligned(rwy)))` |
| C17 | Follow the 5, enter right downwind 12. | `follow(road("I-5")).then(enter(p.downwind))` |
| C18 | Follow the 405, keep it off your left wing, make left traffic 16L. | `follow(road("I-405"), keep=LEFT).then(join(p))` |
| C19 | Start a left turn, keep the 710 off your right-hand side. | `turn(LEFT).then(follow(road("I-710"), keep=RIGHT))` |

Gaps:

- **Unstated end conditions.** In C12, when does "at or above 4000" end? It
  presumably ends at the field. The agent has to infer this, or the
  constraint stays `until(advised())` and blocks the pattern descent. The
  plan validator should flag constraints that make later steps impossible.
- **`then` without a trigger.** In C8 and C9 the pilot chooses when to turn
  off the freeway and join. `join()` has to plan that geometry.
- **Odd geometry.** C16's "turn around the tower" is a curved arrival path
  around a point. `circle(...).until(aligned(...))` covers it, awkwardly.
- **Turn direction as a hint.** In C14 the turn direction is a requirement
  layered on `direct()`.

### Lateral constraints

| ID | Instruction | Translation |
|---|---|---|
| D1 | Remain on the west side of the 405. | `remain(side_of(road("I-405"), WEST))` |
| D2 | Remain south of the 101 for traffic. | `remain(south_of(road("US-101")))` |
| D3 | Remain over the 101. | `remain(over(road("US-101")))` |
| D4 | Remain north of the new Getty, outside the Class D for now. | `remain(north_of(place("the new Getty"))).until(advised()); remain(outside(apt.class_d)).until(advised())` |
| D5 | Remain over land for that north downwind entry. | `remain(over(land()))` |
| D6 | Remain north of the centerline. | `remain(side_of(rwy.extended_centerline, NORTH))` |
| D7 | I want you on the other side of the freeway, the north side, so turn 20° left. | `remain(north_of(road("the freeway"))); turn(LEFT, by=20)` (the intent is the constraint; the turn is how to meet it) |
| D8 | Remain east of the 15 for the jump zone. | `remain(east_of(road("I-15")))` |
| D9 | If able, keep it over the 101 for eastbound traffic. | `prefer(over(road("US-101")))` |
| D10 | Remain over or east of the 405 until south of Encino Reservoir. | `remain(over(road("I-405")) \| east_of(road("I-405"))).until(south_of(place("Encino Reservoir")))` |
| D11 | Remain on the west side of the 405 when you turn crosswind, then proceed northbound. | `p.crosswind.remain(west_of(road("I-405"))); then track(NORTH)` |

Gaps:

- **Side-of-line on a curving road.** "North of the 101" for a freeway that
  bends has to be defined by signed distance to the local polyline, not a
  half-plane.
- **"Over" needs a width tolerance.** How far from the road still counts as
  over it?
- **Constraints scoped to one leg**, as in D11, need a home: `p.crosswind.remain(...)`.

### Vertical constraints

| ID | Instruction | Translation |
|---|---|---|
| E1 | Left downwind departure approved, remain at or below 1800 until north of the field. | `depart(p, via=DOWNWIND); remain(at_or_below(1800)).until(north_of(apt))` |
| E2 | Left crosswind departure, remain at or below 2000 until south of the 101. | `depart(p, via=CROSSWIND); remain(at_or_below(2000)).until(south_of(road("US-101")))` |
| E3 | Remain at or above 3500 until established on base. | `remain(at_or_above(3500)).until(established(p.base))` |
| E4 | Maintain pattern altitude 2000 until you turn base. | `remain(at_or_above(2000)).until(turning(p.base))` |
| E5 | Maintain pattern altitude 1800 until you turn final (helicopter below). | `remain(at_or_above(1800)).until(turning(p.final))` |
| E6 | Remain at or below 2000 until you clear the Class D; after that, frequency change approved. | `remain(at_or_below(2000)).until(outside(apt.class_d)); permit(freq_change, when=outside(apt.class_d))` |
| E7 | Maintain above 3000 for the next 10 miles until east of the Santa Monica final. | `remain(above(3000)).until(east_of(airport("KSMO").runway("21").extended_centerline) \| distance_flown(nm(10)))` |
| E8 | Remain below 2500 or west of Topanga, then resume navigation. | `remain(below(2500) \| west_of(place("Topanga")))` |
| E9 | As soon as you get south of the 101 you can make that left turn; remain at or below 2000. | `permit(turn(LEFT), when=south_of(road("US-101"))); remain(at_or_below(2000))` |
| E10 | Right turn at the shoreline approved, maintain at or below 1500 until advised. | `permit(turn(RIGHT), when=crossing(shoreline())); remain(at_or_below(1500)).until(advised())` |

Gaps:

- **Constraints that defeat the defaults.** E4 and E5 override the AFH
  descent abeam the numbers. E5 leaves about 1000 ft to lose on a short
  final. That needs `slip(FORWARD)` or full flaps, or an "unable". This is
  the clearest case for previewing amendments in the simple sim before
  accepting them.
- **"North of the field"** needs the airport boundary polygon, not just the
  airport reference point.

### Departures

| ID | Instruction | Translation |
|---|---|---|
| F1 | Right turn at the shoreline approved, runway 21, cleared for takeoff. | `takeoff(rwy).then(heading(RUNWAY).until(crossing(shoreline()))).then(turn(RIGHT, to=...))` |
| F2 | Fly straight out and clear the ridge line, then turn left towards the Griffith Park Observatory. | `takeoff(rwy).then(heading(RUNWAY).until(past(ridge("the ridge line")))).then(toward(place("Griffith Observatory")))` |
| F3 | Fly straight out, no turns until the 118; contact SoCal prior to entering Class C. | `heading(RUNWAY).until(crossing(road("the 118"))); contact("SoCal", when=within(nm(2), airspace("KBUR","C")))` |
| F4 | Fly straight out, I'll call your turn. | `heading(RUNWAY).until(atc("turn"))` |
| F5 | Cleared for takeoff, fly runway heading. | `takeoff(rwy).then(heading(RUNWAY).until(advised()))` |
| F6 | Turn left 10 degrees for departing traffic, cleared for takeoff. | `takeoff(rwy).then(turn(LEFT, by=10))` |
| F7 | At the flood basin, right downwind departure approved, remain at or below 2500 until north of the airport. | `depart(p, via=DOWNWIND, over=place("the flood basin")); remain(at_or_below(2500)).until(north_of(apt))` |
| F8 | Just offset left or right 20 degrees, I've got traffic. | `turn(EITHER, by=20)` (the agent picks the side) |
| F9 | At the shoreline, turn left heading 210, cleared for takeoff. | `takeoff(rwy).then(heading(RUNWAY).until(crossing(shoreline()))).then(turn(LEFT, to=210))` |
| F10 | Cleared for immediate takeoff. | `takeoff(rwy, immediate=True)` |

Gaps:

- **"Clear the ridge line" is vague.** Does it mean past the ridge, or above
  it with margin? Probably past, with a terrain-clearance check.
- **"The flood basin"** is a local name (the Sepulveda Basin). This is the
  sort of alias table a resolver needs per airport.
- **Picking a side.** F8 needs traffic awareness to choose which way to
  offset.

### Spacing and speed

| ID | Instruction | Translation |
|---|---|---|
| G1 | Make one left 360 for spacing, then look for traffic on a 6 mile final. | `orbit(LEFT, 360).then(p.follow(traffic("6 mile final")))` |
| G2 | Make a right 360, you're too close to that traffic. | `orbit(RIGHT, 360)` |
| G3 | A wide right 360. | `orbit(RIGHT, 360, size=WIDE)` |
| G4 | Make a right S-turn, you're a little close. | `s_turn(RIGHT).until(behind(t, SPACING))` |
| G5 | S-turns are approved to the west if you need them. | `permit(s_turns(side=WEST))` |
| G6 | Reduce to the slowest practical speed. | `speed(SLOWEST_PRACTICAL)` |
| G7 | Best forward speed until about 2 mile final. | `speed(BEST_FORWARD).until(on_final(nm(2)))` |
| G8 | Keep the speed up, cleared to land. | `speed(BEST_FORWARD)` |
| G9 | Extend downwind, reduce speed, turn base at the 118. | `p.extend("downwind", OPEN); speed(SLOWEST_PRACTICAL); p.turn("base", when=crossing(road("the 118")))` |
| G10 | Short approach approved, cleared for the option. | `permit(p.final_length(SHORT)); land(rwy, OPTION)` |
| G11 | Can you make a short approach? | `can(p.final_length(SHORT))` |
| G12 | Make a turn towards the Encino Reservoir for departing traffic. | `toward(place("Encino Reservoir"))` |

Gaps:

- **Speed words depend on phase.** SLOWEST_PRACTICAL and BEST_FORWARD
  differ between downwind and final.
- **Speed instructions imply a deceleration plan.** G7 has to slow from
  cruise-ish to Vapp by about 1 nm, which may be infeasible. That's a sim
  preview question.

### Sequencing

| ID | Instruction | Translation |
|---|---|---|
| H1 | Turn downwind, number two, follow the Falcon on a 6 mile final, report in sight. | `p.turn("downwind", when=NOW); p.follow(t); report("in sight", when=in_sight(t))` |
| H2 | Follow the Cessna ahead on base, report. | `p.follow(traffic("Cessna ahead on base"))` |
| H3 | Extend downwind, follow the Cherokee on a 3 mile final, report in sight. | `p.extend("downwind", OPEN); p.follow(t)` |
| H4 | Just behind the traffic, turn base, caution wake turbulence. | `p.turn("base", when=behind(t)); wake_caution(t)` (land beyond its touchdown point) |
| H5 | Pass behind that traffic, change to runway 16L. | `pass_behind(t); plan.change_runway(apt.runway("16L"))` |
| H6 | Use the upwind as much as needed to follow company ahead. | `p.extend("upwind", OPEN); p.turn("crosswind", when=behind(t))` |
| H7 | That Cherokee is going to the same place, just follow him. | `follow_traffic(t)` (fly its path, not a pattern) |
| H8 | Re-sequence number 2 behind the Cessna on short final. | `p.follow(t)` |

Gap:

- **This category depends on a traffic model,** meaning sensed positions of
  other aircraft. Without one, `follow`, `behind`, `passed` and `in_sight`
  fall back to ATC timing. Representing traffic intent with the same
  vocabulary (see K4) gives the deconfliction code something to work with.

### Landing, runway changes, exits

| ID | Instruction | Translation |
|---|---|---|
| I1 | Change to runway 16R, cleared to land. | `plan.change_runway(apt.runway("16R")); land(..., FULL_STOP)` |
| I2 | Turn base now, change to runway 16L. | `p.turn("base", when=NOW); plan.change_runway(apt.runway("16L"))` |
| I3 | Cleared for the option; on the go, straight-out departure approved. | `land(rwy, OPTION, on_go=depart(p, via=STRAIGHT_OUT))` |
| I4 | Cleared for the option, right closed traffic. | `land(rwy, OPTION, on_go=circuit(pattern(rwy, RIGHT)))` |
| I5 | Continue down the runway and exit at high-speed Hotel. | `rollout(exit=rwy.exit("H"))` |
| I6 | Can you expedite off the runway at this intersection? | `rollout(exit=NEXT, braking=FIRM)` (answer via `can()`) |
| I7 | Continue to the end, turn left at the end. | `rollout(exit=END, turn=LEFT)` |
| I8 | Cleared to land, exit Delta. | `land(rwy); rollout(exit=rwy.exit("D"))` |

### Go-arounds

| ID | Instruction | Translation |
|---|---|---|
| J1 | Go around, early turn approved to crosswind. | `go_around(); permit(p.turn("crosswind", when=ASAP))` |
| J2 | Start a climb, go around, continue right traffic for 34R. | `go_around().then(circuit(p))` |
| J3 | Go around, you can just make a left turn for the downwind now. | `go_around().then(join(p.downwind, when=NOW))` |

### Off-pattern maneuvering

| ID | Instruction | Translation |
|---|---|---|
| K1 | Cross midfield, hold over the business park. | `overfly(rwy.midfield).then(hold_over(place("the business park")))` |
| K2 | Go to the pier first and then eastbound. | `direct(place("Santa Monica Pier")).then(track(EAST))` |
| K3 | Overfly the Northridge Mall. | `overfly(place("Northridge Mall"))` |
| K4 | (traffic) He'll circle the pier a couple times, then head out towards the Hollywood sign. | `t.intent = circle(place("Santa Monica Pier"), times=2).then(direct(place("Hollywood Sign")))` |
| K5 | Follow the 101 westbound, at or below 2500 until leaving the Class D. | `follow(road("US-101"), direction=WEST); remain(at_or_below(2500)).until(outside(apt.class_d))` |

### Vectors

| ID | Instruction | Translation |
|---|---|---|
| L1 | Turn left heading 230, maintain at or above 4000. | `turn(LEFT, to=230); remain(at_or_above(4000))` |
| L2 | Turn right heading 170, reduce speed to 170 or less. | `turn(RIGHT, to=170); speed(at_most(kt(170)))` |
| L3 | Turn right 10 degrees. | `turn(RIGHT, by=10)` |
| L4 | Expedite climb through 5000. | `climb(rate=EXPEDITE).until(reaching(5000))` |

### Ground

| ID | Instruction | Translation |
|---|---|---|
| M1 | Turn right at Golf, hold short 16L. | `rollout(exit=rwy.exit("G"), turn=RIGHT); taxi(hold_short=apt.runway("16L"))` |
| M2 | Expedite, turn right at G, cross 16R, traffic 1 mile final. | `rollout(exit=rwy.exit("G"), turn=RIGHT); taxi(cross=[apt.runway("16R")], urgency=EXPEDITE)` |
| M3 | Cancel takeoff clearance, hold short 16R. | `plan.replace(hold_short(apt.runway("16R")))` (refused once the takeoff roll has started) |
| M4 | Taxi down the runway, turn left at Quebec, taxi back via Alpha. | `taxi(on=rwy, to=rwy.exit("Q")).then(taxi(via=["A"]))` |
| M5 | Runway 16R at Delta, line up and wait. | `line_up(rwy, at=rwy.intersection("D"))` |

### Instructor and handbook items

These mostly become standard library definitions, defaults and gates rather
than things the agent writes in the moment.

| ID | Rule | Translation |
|---|---|---|
| P1 | Flaps up after positive rate and at Vy. | `when(positive_rate() & at_speed(Vy), flaps(0))` |
| P2 | Takeoff power until at least 500 ft AGL. | `power(FULL).until(agl_above(500))` |
| P3 | Short-field: Vx until clear of obstacles or 50 ft, then Vy, flaps up in increments. | `climb(at=Vx).until(agl_above(50)).then(climb(at=Vy)); when(at_speed(Vy), flaps(0, incremental=True))` |
| P4 | Turn crosswind beyond the departure end, within 300 ft of pattern altitude. | default `p.turn("crosswind", when=past(rwy.departure_end) & within(ft(300), p.altitude))` |
| P5 | Departing: straight out, or 45° in the pattern direction after reaching pattern altitude. | `depart(p, via=FORTY_FIVE)` = `turn(p.side, by=45, when=past(rwy.departure_end) & reaching(p.altitude))` |
| P6 | Enter on the 45 toward abeam midfield, at pattern altitude. | `enter(p.downwind, via=FORTY_FIVE)` |
| P7 | From the upwind side: cross midfield 500 ft above, 2 nm out descend, turn back to the 45. | `enter(p.downwind, via=OVERHEAD)` = `overfly(rwy.midfield, at=p.altitude + 500).until(distance_from(apt) >= nm(2)).then(descend(p.altitude)).then(enter(p.downwind, via=FORTY_FIVE))` |
| P8 | Alternate: midfield crosswind at pattern altitude, then downwind. | `enter(p.downwind, via=MIDFIELD_CROSSWIND)` |
| P9 | Downwind 0.5–1 nm out. | `p.width(NORMAL)` default |
| P10 | Hold altitude until abeam the approach end, then power back and descend. | `p.downwind: maintain(p.altitude).until(abeam(rwy.threshold)).then(power(rpm=1500), flaps(10), descend(fpm=500))` |
| P11 | Turn base when the approach end is 45° behind. | default `p.turn("base", when=angle_behind_wing(rwy.threshold) >= 45)` |
| P12 | Base track perpendicular to centerline, crab as needed. | executor: the path follower already crabs |
| P13 | Strong wind or steep approach: base closer in. | planner rule: `p.final_length()` scales with headwind and flap setting |
| P14 | Final at the approach speed or 1.3 Vso, +10/−5. | `p.final: speed(Vapp, tolerance=(+10, -5))` |
| P15 | Full flaps not until established on final. | `flaps(FULL).earliest(established(p.final))` |
| P16 | Stabilized by 500 ft AGL or go around; unstabilized below 300 ft, go around immediately. | `gate(agl(500), require=stabilized(), otherwise=go_around()); remain(stabilized()).after(agl_below(300)).otherwise(go_around())` |
| P17 | Over 500 fpm, reduce before 300 ft AGL. | `remain(sink_rate(at_most(500))).after(agl_below(300))` |
| P18 | Round out at 10–20 ft. | executor reflex, not exposed |
| P19 | Go-around: power, attitude, configuration. | `go_around()` library definition |
| P20 | After a go-around, shift to the upwind side. | `go_around(offset=UPWIND_SIDE)` |
| P21 | Pattern altitude ±100 ft, airspeed ±10 kt. | default tolerances on `p.altitude`, `PATTERN_SPEED` |
| P22 | Touch down within 400 ft (200 ft short-field) of the point. | `land(rwy, touchdown=(rwy.aim_point, ft(400)))`, also an eval criterion |
| P23 | Rectangular course at constant distance from the field edges. | `circuit(pattern(rectangle=place("that field"), offset=nm(0.5)))` (the pattern generalised beyond runways) |
| P24 | Forward slip to steepen descent. | `slip(FORWARD)`: an executor capability E5 needs |
| P25 | Steep turn: 45° bank, ±100 ft, ±10 kt, roll out ±10°. | `orbit(LEFT, 360, bank=45)` with ACS tolerances |

---

## 4. How this maps onto what exists

| Concept | Today | Gap |
|---|---|---|
| Pattern as a value | `PatternGeometry` (runway frame, downwind offset, `extension_ft`, `crosswind_extension_ft`, `base_turn_x_ft`) | Upwind extension, width presets, `base_through(point)`, final length |
| Turn triggers | Geometric base trigger (`base_turn_ready`), `set_pattern_clearance` gates (≈ `atc("base")`), `execute_pattern_turn` (≈ `NOW`) | Triggers on places, crossings, traffic; earliest/latest bounds |
| Steps | Heading, altitude and speed holds; takeoff; pattern; dead-stick; taxi; line-up; park | `follow(line)`, `toward/direct(point)`, `orbit`, `s_turn`, `hold_over`, relative turns |
| Constraints | None; `SafetyMonitor` covers the envelope only | `remain(...).until(...)`, enforced and previewed |
| Landing options | Full stop, touch-and-go, go-around, runway exits | Stop-and-go, low approach, option, sidestep (`change_runway`) |
| Geography | apt.dat (runways, taxiways, comms) through DuckDB spatial | Roads, shoreline, water/land, landmarks, airspace volumes, local aliases |
| Traffic | None | Sensed traffic (X-Plane AI/multiplayer datarefs), intent model |
| Speech acts | Tools are imperative | `permit`, `prefer`, `can` (sim-preview feasibility) |
| Tolerances | Ad-hoc thresholds in code (e.g. downwind ±350 ft, final ±120 ft / 15°) | ACS tolerances as named defaults |

A plausible implementation path keeps the current stack and adds one profile:

- **`PlanProfile` owns all three axes and interprets a plan value each tick.**
  - Steps compile to guidance targets: a path for the L1 follower, and
    altitude and speed targets for TECS.
  - Conditions and constraints are evaluated in Rust.
  - Hand-off to `LandingRolloutProfile` at touchdown works as it does now.
- **The Python layer only builds plans.** That could be an embedded
  interpreter run once per clearance, or a restricted Python-like front end
  that emits JSON.
- **Plans go through a validator plus a simple-sim preview before they're
  committed.** The preview answers `can()`.

## 5. Open questions

1. **Geography source.** Should it be a local OSM extract (roads, coastline,
   water, landuse, named POIs) loaded into the existing DuckDB spatial views,
   with a per-airport alias table ("the flood basin" → Sepulveda Basin, "the
   VA" → the VA hospital nearest the airport)? How should the resolver show
   its confidence?
2. **Traffic.** Can X-Plane AI or multiplayer traffic positions be read
   reliably through the web API? Without them, a third of tower instructions
   have no sensor behind them.
3. **Limits on open-ended items.** How far can an open downwind extension go
   before the agent should query ATC? Should that limit be in config or
   decided by the agent?
4. **Who resolves ambiguity?** "At" vs abeam, which freeway is "the
   freeway", the unstated end of "at or above 4000". Should the agent decide
   explicitly in code, or should the resolver decide and the agent review?
5. **Executor fidelity.** "Square", slips, and steep turns need the
   low-level control to do more than smooth path following.
6. **Evaluation.** `instructions.yaml` can become an eval set: give the agent
   the instruction with a scenario, check the plan it writes, and fly it in
   the simple sim against the ACS tolerances.
