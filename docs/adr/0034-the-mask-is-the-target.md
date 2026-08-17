# ADR 0034: The mask is the target's shape, and the bump has two witnesses

- **Status:** Accepted
- **Date:** 2026-08-14
- **Supersedes:** the specifics of [ADR 0033](0033-arrival-is-contact.md) §4
  (the ±15° cone) and §5 (the encoders as the sole bumper). ADR 0033's
  decisions 1, 2, 3 and 6 stand unchanged, as does its principle that the
  governor is **informed, never bypassed** — this record changes the shape of
  what it is informed of, not the stance.

## Context

ADR 0033 made arrival mean contact and gave the governor a docking mode so the
contact could happen without disabling the safety filter. Nine Isaac runs
followed. **None of them delivered.** A closed on B, stalled around 0.35 m, and
sat there until the timeout.

Three separate defects were behind that, and a fourth was one second away from
turning a failure into a false success. All four are geometric or arithmetic
and all four were reachable offline; each cost roughly a 25-minute Isaac cycle
to find instead.

### The cone cannot admit a contact

A target of radius R subtends `asin(R/r)`. For B's 0.12 m that is more than the
cone's 15° everywhere inside **0.4636 m**, and **33.5°** at the 0.2175 m contact
range. Below about 0.41 m the target's own shoulders fall **outside** the mask
while lying **inside** the 0.35 m stop, so the filter brakes on the very object
it was told to drive into. The binding return is not the tangent but the beam
just outside the cone edge:

```
d(15°) = r·cos15° − sqrt(0.12² − r²·sin²15°)
at r = 0.3455  ->  0.254 m
crosses 0.35   ->  r ≈ 0.407–0.417, beam-phase dependent
```

Replay of session bag `20260814-003844` shows the governed duty cycle
collapsing on exactly this geometry: **98%** of ticks moving while B closes from
0.70 to 0.42 m, **28%** to 0.38 m, **12%** to 0.35 m, and **0%** below.

### The stub throttles the creep below its own stall threshold

`EastWallStub`'s south face lies 0.315 m off the approach line, entering the
±45° sector at `0.315/sin45° = 0.4455 m` for the **entire** aligned approach.
The slow-zone scale there is `(0.4455−0.35)/0.55 = 0.174`, so the 0.05 m/s creep
becomes **8.7 mm/s** — about 46 s to cover the remaining distance against a 25 s
budget. Even a perfect mask times out. Worse, 8.7 mm/s is *below* the docking
controller's own 10 mm/s stall threshold, so a perfectly healthy creep reads as
a contact.

### The wheels cannot witness the bump

The twin models slip deliberately: rear friction is authored at 0.1
(`build_corridor_arena.py:126`), `/odom_raw` integrates joint velocities with a
measured 92% wheel/world disagreement on the stand, and the EKF fuses wheel
twist only. At a real bump the wheels may keep turning and `measured_vx` never
falls.

### And a governor stop forges a bump

ADR 0033 §5 accrues stall whenever EKF `vx ≈ 0` while bearing-aligned. A stop
imposed by the safety filter satisfies that exactly — the machine asks for
motion, the filter zeroes it, the encoders read nothing. The cone leak pinned A
at 0.31–0.35 m, **inside** the 0.39 m sighting ceiling, so one uninterrupted
second would have reported `DELIVERED_CONFIRMED` between 0.12 and 0.17 m short
of ever touching B. It did not fire only because the throttled creep kept
jittering the debounce.

### Why the unit tests were green throughout

Every docking test in the fleet's `test_governor.py` modelled B as **one beam at
one bearing**. A single-beam target cannot leak outside an angular mask. The
in-process proof offered with the previous fix — "0 of 31 ticks moving without
the approach, 29 of 31 with it" — used the same fixture. It measured that the
mask was plumbed in and was reported as measuring that the mask worked.

## Decision

### 1. The mask is the target's silhouette, on its own topic

A return is masked **iff its point lies within `target_radius + margin` of the
declared target centre**. Not a cone: a disc, sized by the thing being
approached, in the place it was last seen.

- **It admits contact at every range.** Returns from the target's surface are
  within one radius of its centre by definition, at 0.62 m and at 0.22 m alike.
- **It self-sizes.** Far out it subtends almost nothing; up close it opens
  exactly as fast as the target does, and no faster.
- **It closes a hole the cone had.** The cone masked every on-cone return
  *nearer* than the target — the whole segment between sensor and target — so a
  foot planted on the approach line was invisible to the filter. The disc masks
  nothing nearer than `centre − radius − margin`.

The radius is the **authored** one. Measured across 38 runs the detector's
fitted radius spans 0.072–0.168 m, filling its acceptance band edge to edge, so
a disc sized from a fit would intermittently be smaller than B and unmask its
own target's nose.

It travels on a **new topic**, `~/docking_disc`, rather than a reinterpretation
of `~/docking_approach`: the third field means *margin* on one and *target
radius* on the other, and a sender that has not been updated must never have its
0.10 m margin read as a 0.10 m target. Both expire independently on silence; a
fresh disc wins; a caller still sending only the cone keeps its old behaviour.

Two numbers are **not** the caller's to set. The **margin** is the governor's own
parameter, because slack on a safety mask belongs to the filter enforcing it.
The **radius** is bounded by `docking_max_target_radius` (0.25 m) — it is the one
declared number that *widens* the masked region — and an out-of-range
declaration is **refused, not clamped**: a caller asking to mask a metre of
corridor has a bug, and quietly granting it the maximum hides the bug while
still moving the robot.

Clearances, verified: the east wall is 0.362 m from B's centre and the stub
0.568 m, so a 0.22 m disc keeps at least 0.06 m of slack including worst-case
staleness (~0.08 m).

### 2. The clamped creep is exempt from the slow zone, and from nothing else

While a declaration is live, a command **at or below** the 0.05 m/s creep clamp
skips the linear slow-down. The zone exists to bleed off speed before a stop,
and a command already at the clamp has nothing to bleed — braking distance at
0.05 m/s is millimetres, and the 0.35 m hard stop is untouched and still ahead
of it.

Untouched: the hard stop, the deadman, the stale-scan stop, the command
timeout, the empty-sector fail-closed, the yaw gate, and the off-object obstacle
stop at full strength. Anything faster than the clamp is slowed exactly as
before. The exemption is scoped to a live declaration **and** a command already
at creep speed; neither alone is enough.

### 3. The bump has two witnesses, and neither is the wheels

Contact is confirmed only when **both** hold across the debounce window:

- **The governor actually permitted forward motion** — `governed_vx` read back
  off `/cmd_vel`, not what the controller asked for. This is what separates
  "the filter stopped me" from "I hit something".
- **The laser says the robot did not move** — the scan matcher's verdict, taken
  as a **median over per-sample speeds** in a 2 s window. The matcher
  re-registers and jumps: the bag puts its stationary p95 at 374 mm against a
  median of 16.8 mm, so a mean (0.76 m/s on that data) or a maximum (3.74)
  reads a parked robot as moving and no contact is ever confirmed.

The encoders **corroborate and never suffice**. The laser witness is used as a
witness only, never fused into control, which is the limit `laser_odometry`'s
own docstring asks callers to respect: it "correctly reports no translation
while the wheels spin".

Either witness may **abstain** — a silent matcher, or a window too thin for a
median. Abstention falls back to the encoders, which can cost a delivery
(`ARRIVED_UNPROVEN`) but cannot forge one. That is the safe direction, and it is
deliberate.

### 4. Testing is cheapest-first, and the negative controls are permanent

Four tiers, and nothing reaches Isaac until the cheaper ones are green:

| tier | what | cost |
|---|---|---|
| T0 | Closed-loop bench, no ROS, no GPU. Real detector, machine and governor over a raycast corridor, **B as a 32-gon** | 13 s |
| T1 | The real governor node in-process, real topics, synthetic scans | ~10 s |
| T3 | Terminal micro-arena in Isaac, no Nav2 | ~3 min |
| T4 | Full acceptance run | ~8 min |

**No fixture may model the target as a single beam.** That constraint is the
direct cause of this record existing, and `_disc_scan()` — an exact ray-circle
trace of a real cylinder — replaces `_ahead()` in every docking test.

The cone scenarios are **kept and stay red**, permanently, as negative controls.
A bench whose failure cases stop failing has stopped modelling the geometry that
grounded nine Isaac runs, and its green verdicts are then worth nothing.
`DockingApproach` stays in the source for the same reason, documented as
superseded rather than deleted.

## Acceptance

Measured on the T0 bench, `nominal_m6_n3`, against a 0.2175 m contact range:

| scenario | mask | state | contact | final r |
|---|---|---|---|---|
| `disc` | silhouette | `DELIVERED_CONFIRMED` | yes | 0.2181 m |
| `slip` | silhouette | `DELIVERED_CONFIRMED` | yes | 0.2181 m |
| `misaligned` | silhouette | `DELIVERED_CONFIRMED` | yes | 0.2189 m |
| `cone_leak` | ±15° cone | `ARRIVED_UNPROVEN` | **no** | 0.4031 m |
| `slow_zone_false_stall` | ±15° cone | `DOCKING` | **no** | 0.4031 m |
| `forgery` | ±15° cone | `DOCKING` | **no** | 0.4031 m |

0.6 to 1.4 mm from contact, which is beam discretisation rather than tolerance
on truth. `slip` arrives with the pose pinned and the encoders reporting motion
throughout.

On T1, at the 0.34 m range where the cone pinned: with the declaration, duty
1.0 over 40 samples at the full 0.05 m/s; without it, duty 0.0 and "obstacle at
0.22 m".

The fleet governor suite carries the geometry directly: the cone's failure on a
ray-traced cylinder is **asserted**, the disc is verified to admit the target at
0.62, 0.46, 0.34 and 0.2175 m, an intruder between robot and target is verified
**visible** to the disc and hidden by the cone, and the wall behind the target
is verified never masked.

### Live, run `20260814-023306` — decisions 1 and 2 confirmed, decision 3 refuted

The first live exercise of all of this. **The mask and the exemption work, and
the witness threshold does not.**

| | |
|---|---|
| `/cmd_vel` during the 23.8 s creep | **476 of 476 ticks at the full 0.05 m/s** — zero braking |
| A's closest approach to B (truth) | **0.2252 m** against a 0.2175 m contact — 7.7 mm |
| Previous nine runs | pinned at 0.3455 m, 128 mm short, and never moved |
| Reported state | `ARRIVED_UNPROVEN` — creep timed out |

The governor is exonerated: it permitted every tick of the creep, at full speed,
for the whole window. That is decisions 1 and 2 doing on the robot exactly what
the bench said they would.

**Both witnesses over-report motion, so neither ever declared the stall.**

- The wheels read **0.0492 m/s median across the window and 0.0510 m/s in the
  final two seconds**, while A was pressed against B. Integrated, they claim
  1.19 m of travel against 0.393 m actually advanced — **A slipped two thirds
  of its commanded distance.** This is ADR 0033 §5's encoder bumper failing
  live, exactly as §3 above predicted, and it retires that clause on evidence
  rather than on argument.
- The laser matcher reads a **net displacement of 0.882 m** over the same
  window against 0.393 m of truth, and a per-pair median of 60 mm/s against a
  true average of 16.5 mm/s. Its final-2 s median was 0.0384 m/s against the
  0.030 m/s threshold — so it said "moving" and withheld the confirmation.

**The ε was derived from the wrong regime.** 0.030 m/s came from a *parked*
robot's 16.8 mm median. During a creep the matcher's own noise floor is roughly
four times the true speed, and no fixed threshold on this signal separates
"creeping" from "stopped against an obstacle". Decision 3's *structure* — two
independent witnesses, neither of them the wheels — survives; its laser
implementation does not, and is **open**.

## What is NOT established

- **The bench does not reproduce the session bag within its own bar.** The
  cone pin is a declared 0.4029 m against the bag's 0.3455 m — 0.057 m out
  against a ±0.03 m bar. The mechanism, duty collapse and leaked band all
  match; the radius does not. See
  [`docs/evidence/creep-bench/NOTES.md`](../evidence/creep-bench/NOTES.md).
  It is recorded rather than resolved because the real robot got *closer* than
  the bench and still never contacted.
- **The laser-stationarity threshold is measured WRONG, not merely
  unvalidated.** See the live section above: the matcher's noise floor during a
  creep is about four times the true speed, so 0.030 m/s never fires. A
  replacement has to come from a signal that degrades gracefully at millimetre
  scale — candidates are the *change* in the matcher's own residual, the
  detector's range trend across the last sightings before B goes blind, or a
  current/effort signal from the drive — and each needs its own measurement.
  **Until then no run can report `DELIVERED_CONFIRMED`,** which is the correct
  failure direction and the reason this is recorded rather than patched.
- **Whether A actually touched B on that run is unresolved.** Truth puts it
  7.7 mm short of a *modelled* contact range, which is inside the slop of that
  model, and the wheels were still turning. The bump may have happened and
  gone unwitnessed; that is precisely the ambiguity a working witness exists
  to remove, so it cannot be settled by asserting it.
- **The sighting ceiling is unchanged at 0.39 m.** The plan proposed tightening
  it to ~0.26 m now that a governor stop can no longer forge a bump. The dual
  witness removes the forgery this ceiling was guarding against, so the change
  is no longer urgent; it is left for a record that can measure it.

## Consequences

The safety argument gets **stronger**, not weaker: the disc unmasks the approach
segment the cone was hiding, the exemption is narrower than the mode it sits
inside, and contact now requires two independent witnesses where it previously
required one that a governor stop could satisfy on its own.

The cost is a second declaration topic to keep alive, one more parameter pair on
the governor, and a dependency on the scan matcher publishing during the creep.
The last is the real exposure: if `/odom_laser` is silent the witness abstains
and the delivery falls back to encoders that slip. That fails toward
`ARRIVED_UNPROVEN`, which is the outcome this project would rather have than a
confident wrong answer.
