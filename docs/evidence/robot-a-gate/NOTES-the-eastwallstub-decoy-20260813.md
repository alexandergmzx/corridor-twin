# The scene contains a decoy, and docking arms on it

**Status: RED. W2 does not close the delivery.** The map-frame deadlock it set
out to remove is removed and arming now fires on every recorded run — but on
four of seven it fires on the wrong object, and no variant tried today fixes
that. This is the finding, not a fix.

## What was measured

`tools/diagnostics/arming_replay.py` replays a bag's recorded `/scan` through
the real `LandmarkDetector` and the real `DockingMachine.armed`, integrating
`/odom` for travel exactly as the live gate does. Nothing in the arming path
sees truth; truth is read once, afterwards, to say **what the detection
actually was** — because "it arms" is not the claim that matters. The 5.754 m
failure armed too.

    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
    PYTHONNOUSERSITE=1 python tools/diagnostics/arming_replay.py \
      ~/Development/MicroROS/MicroROS-assets/bags/20260813-{000602,015025,020537,035954,102705,111002,113859}-isaac-d67

Seven bags, robot1, domain 67, all recorded 2026-08-13.

| bag | armed at travel | detected range | fitted r | detection really at | miss from B |
|---|---|---|---|---|---|
| 000602 | 6.371 m | 0.893 m | 0.1066 | (5.025, −2.388) | **0.018 m — B** |
| 015025 | 6.094 m | 0.269 m | 0.0940 | (4.487, −1.855) | 0.774 m |
| 020537 | 6.146 m | 0.651 m | 0.1110 | (5.033, −2.401) | **0.005 m — B** |
| 035954 | 6.369 m | 0.156 m | 0.1378 | (4.459, −1.926) | 0.748 m |
| 102705 | 6.326 m | 0.240 m | 0.1137 | (4.496, −1.925) | 0.721 m |
| 111002 | 6.152 m | 0.408 m | 0.1456 | (4.679, −1.933) | 0.589 m |
| 113859 | 5.699 m | 0.695 m | 0.1098 | (5.032, −2.402) | **0.007 m — B** |

Three on B to within 18 mm. Four on something else — and the four agree with
each other to within 0.11 m.

## The decoy is authored geometry

The four wrong detections cluster at roughly (4.5, −1.9). The manifest says
what is there:

    "EastWallStub": [[4.56534, -2.085], [5.4, -2.085],
                     [5.4,     -1.767], [4.56534, -1.767]]

Its **west end cap** — a free-standing face 0.318 m wide, centred at
(4.565, −1.926) — is the object. Every one of the four sits within 0.11 m of
that point.

It passes every test the detector has, and it passes them honestly:

* **Shape.** A 0.318 m cap fits a circle of 0.094–0.146 m against B's authored
  0.12, inside `MAX_RADIUS_ERROR_FRACTION`.
* **Chord.** 0.318 m against a permitted `0.24 × 1.4 = 0.336`.
* **Isolation** — the test written specifically to separate a post from a
  corner. It requires open space on both sides, and from the approach direction
  the cap *has* open space on both sides: beams passing north of the cap fly
  into the street, beams passing south fly past B. The stub is short enough to
  be genuinely free-standing from where A looks at it.

So this is not a detector bug in the sense of a missing check. **The scene
contains a second object that is, to a 2D lidar on A's approach, B.**

## Why the obvious fixes do not work

Two were tried, measured, and rejected. Both are recorded because the reasoning
that motivated them is more persuasive than the results.

**Rank candidates by radius error instead of fit residual.** The plan
identified this: `candidates()` sorts by residual, which asks "how circle-like"
rather than "how B-sized". Measured over four bags, per-frame:

| bag | by residual (B / stub / other) | by radius error (B / stub / other) |
|---|---|---|
| 035954 | 2090 / 62 / 176 | 2059 / 63 / 206 |
| 102705 | 689 / 21 / 98 | 670 / 21 / 117 |
| 111002 | 333 / 47 / 11 | 330 / 50 / 11 |
| 015025 | 758 / 18 / 86 | 748 / 21 / 93 |

Indistinguishable, and marginally worse. Rejected.

That table carries the more useful number, though: **the detector picks the real
B in 88–97% of frames and the stub in 2–12%.** Per-frame, B wins overwhelmingly.

**Require the arming frames to agree on a position.** Given the ratio above,
this looked decisive: make k-of-n mean "k scans that agree they are looking at
the same object" rather than "k scans that each saw something", so an
intermittent stub frame breaks a run instead of adding to one.

Measured, it was **worse — six of seven armed on the stub instead of four.**

The reason is the useful part. **The failure is ordering, not agreement.** The
stub's cap sits between A and B on the approach, so A resolves it *first*;
requiring consecutive agreement simply hands the decision to whichever object
accumulates a run earliest, and that is the decoy. Reverted by default, per
gate discipline. The weaker rule shipped.

**Tighten the chord ceiling.** A circle of radius *r* has a maximum chord of
exactly 2*r*, so `MAX_CHORD_FACTOR = 1.4` admits 2.8*r* — 40% beyond
geometrically possible — while the stub's face is 0.318 m against B's 0.240 m
ceiling. On paper the ceiling *is* the discriminator, and unlike a radius or
residual threshold it is a geometric impossibility rather than a tuned number.

**The prediction was recorded before the measurement: the stub tally should
collapse between 1.4 and 1.3, and B's should stay flat until 1.0.** Measured by
`tools/diagnostics/chord_sweep.py` over the same seven bags:

| chord factor | ceiling | B frames | stub frames |
|---|---|---|---|
| 1.4 (shipped) | 0.336 m | 6663 | **302** |
| 1.3 | 0.312 m | 6663 | 252 |
| 1.2 | 0.288 m | 6663 | 186 |
| 1.1 | 0.264 m | 6662 | 172 |
| 1.0 | 0.240 m | 6665 | **159** |

**Falsified.** No collapse anywhere — a gradual 47% decline that never
approaches zero, with 159 stub frames surviving even at the geometric floor.
The detector is fitting a PART of the 0.318 m face, not the whole of it, so its
chord is already inside B's ceiling and no ceiling can separate them. This was
named as the risk before the run and it is what happened.

Half of the result is still useful: **B's tally is flat across the whole
sweep** (6663 → 6665), so tightening is free. `1.4` permitting chords a real
cylinder cannot produce is a defect on its own terms, and `1.1` would cost
nothing and remove 43% of the decoy's frames. **Not applied** — a change whose
own measurement says it does not fix the thing it was made for does not get
shipped on the strength of a side benefit. Recorded as a recommendation.

## The live run of 2026-08-13 16:02 docked on B

One run, `--allow-contract-fail`, so not gate evidence — but it is the first
direct evidence of the arming change in the field, and it disagrees with the
replay's pessimism.

    world_frame_delivery: standoff (4.438, -2.400)
                          final    (4.4381, -2.1528)
                          closest_approach_m 0.2472   walked_away_m 0.0
    docking: DOCKED at 0.6655 m, refinements 0
             rejections: {"not yet persistent across scans": 97}

| from A's final position | distance |
|---|---|
| to **B** | **0.6489 m** — against a detected 0.6655 m |
| to the stub's west face | 0.2601 m |

It locked onto B, and the laser agreed with truth to **17 mm**. `walked_away_m`
is **0.0**: A stopped and stayed, where every previous run either drove to the
street's far wall or docked and then walked 3.43 m off chasing a stale map
goal. The rejection log contains no containment refusal of any kind, against
2812 map-frame refusals before.

So the decoy is real but it is not deterministic, and it did not decide this
run. Four of seven in replay, zero of one live. That is not a contradiction and
neither number is a rate.

## Five live runs: the rate, and the signature

`tools/diagnostics/run_batch.sh 5 nominal_m6_n3`, sequential, robot1, domain
67, all `--allow-contract-fail`. Summarised by
`tools/diagnostics/batch_summary.py`, which attributes each dock by matching the
detected range against the true distance to B and to the stub.

| run | ref | closest | walked | docked on |
|---|---|---|---|---|
| 155926 | 0 | 0.2472 | 0.000 | B |
| 161855 | 0 | 0.4482 | 0.000 | **stub** |
| 162234 | 2 | 0.1147 | 0.072 | B |
| 162737 | 3 | 0.1007 | 0.012 | B |
| 163112 | 2 | **0.0354** | 0.008 | B |
| 163617 | 0 | 0.6257 | 0.000 | **stub** |

**The decoy takes 2 of the 5 post-fix runs — 40%.** The replay's 4 of 7 was not
pessimistic; it was right, and the single good live run that preceded this batch
was the outlier. One run is not a rate, which is what the batch exists to say.

Two things are now clearly separate. **Arrival semantics are fixed**: the three
runs that acquired B refined 2-3 times and closed to 0.1147, 0.1007 and
0.0354 m, all staying put — three consecutive results meeting both halves of
ADR 0033's proposed criterion. **Arming target is not fixed**: two runs parked
0.45-0.63 m away at the wrong object, and 161855 proves the two faults are
independent, since correct arrival semantics on the wrong object is still a bad
delivery.

### The signature separates perfectly

Full docking histories, detected range at each event:

    B    : refine@0.854 -> 0.712 -> DOCK@0.617
           refine@0.907 -> 0.834 -> 0.745 -> DOCK@0.618
           refine@0.906 -> 0.620 -> DOCK@0.613
    stub : DOCK@0.353
           DOCK@0.468

**Every B acquisition begins outside the 0.620 m arrival band; every decoy
acquisition begins inside it, and docks instantly with no refinement.** The gap
between the two populations is 0.386 m with no overlap.

The mechanism is geometric rather than statistical. A *drives past* the stub --
lane centre x = 4.083 against a face at x = 4.565, so 0.48 m of lateral
clearance -- and *approaches* B head-on, stopping short of it. Something you
pass is first seen close; something you approach is first seen far.

### Why the travel gate cannot help

> **CORRECTED. The first version of this section said the arming lead was
> 0.900 m against a 0.806 m separation — a ratio of 1.12 — and concluded that
> shrinking the window by a few centimetres would put the decoy behind A. That
> was wrong, it was published in `ef296b0`, and the real ratio is 3.1. The
> conclusion it supported is dead; the section below is the measured version.**

    route station abeam the stub : 6.575 m
    route station at B           : 7.380 m
    along-route separation       : 0.806 m
    arming unlocks at travel     : 4.850 m
    ACTUAL lead before B         : 2.531 m   (3.1x the separation)

The 0.900 m figure was `window_m`, which is a tolerance *on
`route_to_delivery_m`* — and **`route_to_delivery_m` is wrong**
(`tools/corridor_nav_gate.py:83-98`). Its docstring says the departure leg
"runs PAST B" and excludes it from the sum. It does not: the five segments run
approach → corner arc → departure → delivery arc → delivery run, and the route
*ends at B* — measured, the station closest to B is 7.3804 m at a distance of
0.0000 m, which is the full trajectory length. `departure_length_m` (1.6307 m)
lies **before** B, so omitting it understates the route by exactly that, and
arming unlocks 2.531 m before the delivery point rather than 0.900 m.

So the window is 3.1x the decoy separation, not 1.12x, and no small adjustment
closes it. Worse, the two requirements are contradictory: the good runs armed
with 1.681 m or more of lead, and excluding the decoy needs 0.520 m or less.
**Window-shrinking is dead**, and it was never alive — it rested on an
arithmetic error of mine on top of a pre-existing one in the codebase.

The pre-existing bug is not simply "correct the sum", either. Corrected,
`min_travel_m` becomes 6.480 m, and bag 113859 armed **on the real B** at
5.699 m of odometry travel — so the correction would have refused a good
arming. The reason is that A does not drive the authored route: Nav2 plans its
own path, and the measured odometry distance to B is around 5.7 m against an
authored 7.38 m. `route_to_delivery_m` is wrong in derivation and accidentally
close in value. Correcting the derivation without re-basing
`ARM_WINDOW_ROUTE_FRACTION` on measured travel would break arming outright.

What survives from the section is only the qualitative half, and it still
holds: every guard so far governs *whether* a detection is believable — the
stub genuinely is — and none governs *when* the question is first asked.

This also explains why every guard added so far failed. They all govern
*whether* a detection is believable, and the stub genuinely is. None of them
govern *when* the question is first asked.

## The fix: a wall's end is not convex

Found by putting the "must approach from outside the arrival band" rule through
adversarial review, which **rejected** it — it is exactly `refinements >= 1`, it
encodes A's route rather than any property of the objects, and its margin was
31 mm — and proposed this instead.

**A cylinder of radius r puts its centre exactly r beyond the nearest point of
its own surface.** The closest return lies on the segment from the sensor to
the centre; that is what convex means. A circle fitted across part of a flat
face has no such constraint, and routinely places its centre level with — or in
front of — the measured surface, which is impossible for the object it claims
to be.

    centre_depth = |centre| - min(|point|)   over the fitted cluster

Isolation could never have caught this. It separates a post from a CORNER,
something attached to a wall that continues; the stub's free end genuinely has
open space on both sides. Convexity is a property of the OBJECT, so it survives
re-authoring, the other two profiles, and any change to A's route.

### The marginal distribution nearly talked me out of it

`tools/diagnostics/centre_depth.py`, seven bags, every accepted fit:

| | median | p05 | depth ≤ 0 |
|---|---|---|---|
| B | +0.1020 (≈ r) | +0.0719 | **1.1%** |
| stub | +0.0849 | −0.0772 | **21.9%** |
| other | −0.0236 | −0.1093 | 54.5% |

Right direction, but 78% of decoy frames survive a sign test — and I wrote that
this "doesn't obviously support" the predicted 4/7 → 1/7. **That was the wrong
statistic.** Arming needs 3-of-5 persistence, so the question is not what
fraction of frames survive but whether the survivors still form a chain. End to
end:

| filter | first arming on B | on the decoy | never armed |
|---|---|---|---|
| baseline | 3 | **4** | 0 |
| `depth > 0` | 6 | 1 | 0 |
| `depth > 0` and `abs(d − r) ≤ 0.40r` | 6 | 1 | 0 |
| **`depth > 0` and `abs(d − r) ≤ 0.25r`** | **7** | **0** | **0** |
| `depth > 0.5r` | 6 | 1 | 0 |

Both pre-registered predictions landed exactly. Removing 22% of the decoy's
frames does break its chain.

Confirmed against the shipped detector: **7 of 7 bags arm on the real B**,
misses 0.005–0.024 m, first arming at 0.651–0.893 m — all outside the 0.620 m
arrival band, so refinement runs rather than instant-docking. Runner-up counts
collapse as well (204 → 33, 51 → 9): the same decoy leaving the
radius-ambiguity path, which was eating B's arming window.

**How much of this is geometry and how much is a chosen number.** The sign test
has no threshold at all and does most of the work, 3/7 → 6/7. The magnitude
band closes the seventh, its marginal contribution rests on one bag, and at
0.40 it buys nothing. Both figures are in the constant's own comment so the
next reader does not have to take the 7/7 at face value.

### Live confirmation: nine runs, three profiles, zero decoy docks

`tools/diagnostics/run_batch.sh 3 nominal_m6_n3 wide_corner_m6_n4_5
uniform_m6_n6`, sequential, all `--allow-contract-fail`.

| | before convexity | after |
|---|---|---|
| docked on B | 5 | **4** |
| docked on the decoy | **5 of 10 (50%)** | **0** |

Deliveries, closest approach to the standoff: **0.0675, 0.0909, 0.1457, 0.150,
0.1703 m**, every one of them staying put. Four docks is a small sample and is
not on its own a rate; it agrees with the 7-of-7 replay, which is why it is
being read as confirmation rather than as proof.

Of the nine attempts, four are excluded and none of the exclusions is about
docking: two `bt_navigator never reached ACTIVE`, one lost goal-acceptance where
the transit ran but the dock loop never did, and one where the EVALUATOR's
ground-truth stream died.

### The run that was scored as the worst of the day and was one of the best

`20260813-174631` was recorded as `closest_approach_m 5.0072`, docked on an
unattributable object. Truth says otherwise:

    start (0.016, 0.000)   end (4.456, -2.251)
    path 5.653 m   displacement 4.978 m   -> 0.150 m from the standoff
    docked range 0.616 m   true distance to B 0.601 m   -> a 15 mm match, on B

The evaluator's own ground-truth subscription died after 0.81 s, so
`world_frame_delivery` scored the run against A's SPAWN.
`ground_truth_distance_m` reads 0.000 where every other run in the population
reads 5.4-10.7 m.

It cost two wrong diagnoses before the bag was opened -- first that A never
moved, then that the EKF was integrating position noise into phantom travel.
Measured, odometry noise while genuinely stationary is **0.014 mm per sample**
and sums to 0.017 m across a whole run; it cannot accumulate metres, and the
4.925 m of recorded travel was real motion throughout. `batch_summary.py` now
refuses to score a run whose evaluation plane went dark, because a sensor that
failed on the EVALUATION side is not evidence about the robot.

## What this means

- **Docking cannot be closed by tightening the detector's thresholds.** Every
  threshold that separates B from the cap also sits inside the spread of B's
  own fitted radii (0.107–0.124 measured on true-B armings, against the cap's
  0.094–0.146 — overlapping ranges).
- **Per-frame accuracy is not the problem.** B wins 88–97% of frames. The
  problem is that arming is a *first-past-the-post* decision taken at the one
  moment the decoy is best placed to win it.
- **Two directions are open, and choosing between them is a decision, not a
  measurement.** Either the scene loses the decoy — `EastWallStub`'s free west
  end is arguably a modelling artefact rather than an intended feature — or
  arming stops being first-past-the-post and instead accumulates evidence over
  the whole approach, where B's 9:1 frame advantage would decide it.
  **Parked for the operator.**

## What W2 did close

Not nothing, and it is independently justified:

- The **map-frame proximity test is deleted.** It refused 2812 times on a run
  where A's own laser was measuring B correctly at 0.63 m. Arming now fires on
  all seven bags; before, on the docked run, it never fired at all.
- **`armed()` cannot read a robot pose** — passing one is a `TypeError`, pinned
  by `test_arming_reads_no_robot_pose_at_all`. The property ADR 0029 wanted is
  now structural rather than asserted.
- The bearing test is **body-frame**, with a floor that is geometry rather than
  a tuned number: at closest approach B is abeam by definition, so 90° is the
  minimum any honest cone can use. Measured across seven bags: 85.2–91.6°.
- **Persistence counts scans, not calls** — the docking loop spins at 10 Hz
  regardless of scans and over-counted 2.7× (8119 calls / 3031 frames).

## Scope and limits

- Seven bags, one profile (`nominal_m6_n3`), one robot (robot1), all from
  2026-08-13. The other two profiles are not measured here; the stub is present
  in all three, so there is no reason to expect them to differ, but that is an
  expectation and not a measurement.
- The replay reproduces the arming *decision* only. It does not simulate what
  Nav2 would then have done, so "armed on the stub" is not the same claim as
  "would have delivered 0.6 m from B" — the refinement loop and the sensor-based
  DOCKED test both still run afterwards, and neither was replayed.
- No `slam_toolbox` or Nav2 parameter was touched.
