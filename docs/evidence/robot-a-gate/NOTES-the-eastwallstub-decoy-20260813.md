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
