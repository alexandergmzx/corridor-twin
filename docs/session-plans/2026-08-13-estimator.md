# Session plan — bank correction 2, build correction 3's estimator

**Branch** `estimator-2026-08-13`, from `b915f81` (tip of
`delivery-close-2026-08-13`, pushed).
**Started** 2026-08-13 21:52 CST · **Budget** 6 h → 03:52 ·
**Nothing new started after 03:22.** `date` between units.

Unattended hard rules bind: append-only history, green-checkpoint commits,
Isaac single-occupancy via `/tmp/fleet-isaac.lock`, park-don't-decide, lens up
on every live run, **nothing pushed**.

**Naming.** The brief's handback line names the task author. That is the name D8
removed from this repository, so the artefact is **"the scoreboard against the
2026-08-04 interview corrections"**. Flagged rather than silently substituted.

---

## Why this session exists

Correction 2 (autonomous navigation) is **functionally done and evidentially
unbanked**. Correction 3 (active AI/ML) has a trained detector and no estimator.
This session banks the first and produces the first honest number for the
second.

---

## Ratified before planning, not re-opened

- **The decoy question is CLOSED** by `2a4e706`. `EastWallStub` stays
  permanently — it is the task author's topology, ADR 0010 lineage — and is
  recorded in ADR 0033 as the scenario's negative control.
- **Enforcement footage must be of REAL deliveries.** The narrow fleet edit for
  concurrent camera capture is granted: single commit, listed separately.
- No `slam_toolbox`/Nav2 tuning outside the capped matcher A/B. Gates run
  dock-ON under ADR 0033 semantics once pinned.

---

## What the investigation found

Ten findings. Three move work, one threatens the demo, and three correct the
estimator design before a line of it is written.

### 1. Defect 1 is already closed

The brief lists refined-goal orientation at `corridor_nav_gate.py:457` as open.
It was fixed last session in `e351c0c` — `facing_yaw` at
`tools/corridor_nav_gate.py:466-468`, with two tests. The brief is reading the
handback's stale block (finding 2). **One unit drops out of the queue.**

### 2. The handback contradicts itself, and the brief inherited it

`docs/session-plans/2026-08-13-delivery-close.md` carries **two** "Morning
decisions" blocks:

| line | content |
|---|---|
| **346** | decoy is an open scene-vs-method decision that "unblocks W3 and W4" |
| **426** | "The decoy is closed — the scene-vs-method question is withdrawn" |

346 predates the afternoon; 426 is current. The stale block is marked superseded
**in place**, not deleted — it is the record of what was believed at the time.

### 3. Nothing is banked, because `out/*` is gitignored

`git ls-files out/evidence` → **0**. Every run artifact from last session lives
on this disk only. Banking correction 2 is a **promotion** job into
`docs/evidence/robot-a-gate/`, which already holds `gate-*.json`, `nav-*.json`,
`acceptance-*.json` and `NOTES-*.md` in exactly that pattern.

### 4. The current code's record is better than the brief credits

The brief's "three consecutive greens" (0.1147, 0.1007, 0.0354) are
**pre-convexity**. Post-convexity there are **four consecutive** nominal passes:

    0.0592, 0.1332, 0.0458, 0.1335 m   — all DOCKED on B, all walked_away < 0.08

Post-convexity docked runs across all three profiles: **8 of 8 on B, 0 on the
decoy.**

### 5. ADR 0033 must supersede TWO clauses, not one

The brief names ADR 0029:129. There is a second, independent restatement:

- `docs/adr/0029-map-divergence-at-the-corner.md:127-130` — Nav2 `SUCCEEDED`
  within 0.15 m map-frame; the demonstration must pass with the detector disabled
- `docs/adr/0031-b-is-the-cylinder.md:104-106` — restates both independently

**ADR 0032 has never been allocated** — zero references repo-wide. Its promise is
unnumbered at `docs/adr/0024-learned-enforcement-perception.md:51-54` (the family
pin "lands as its own short record") and `:65-69` (resolution as a measured
parameter). **ADR 0033's number is already in circulation at 7 sites**, including
shipped code at `tools/diagnostics/batch_summary.py:13`.

Ledger is otherwise clean: 31 ADRs, index and mermaid map consistent,
`test_repository_contract.py:96` green. A new ADR needs a table row **and** a map
node in the same commit.

### 6. **A cannot speed. The enforcement demo has nothing to enforce.**

Measured, not inferred:

    governor absolute cap    0.35  m/s   yahboomcar_safety/governor.py:46
    Nav2 planning cap        0.22  m/s   config/robot1/nav2_robot1_corridor.yaml:88
    measured cmd_vel peak    0.220 m/s   every run, sitting on the cap
    measured odom peak       0.272-0.325 m/s
    strictest scene limit    0.80  m/s   corridor-robot-scale.yaml:120-121

Widths scaled by 0.30; `limit_mps` deliberately did not — it is a policy, not a
length. **Even the safety governor's ceiling is 2.3× below the strictest limit**,
so no Nav2 configuration produces a violation without bypassing the governor,
which `0031:91-94` forbids outright.

This does **not** block the estimator — measuring speed accurately is worth doing
whether or not a limit is crossed. It blocks the *demo narrative*, which requires
"a compliant stretch **and** a speeding episode". ADR 0023 defers the width→limit
table with `[to pin after first profile run]`; those runs now exist. **Decision
D3.**

### 7. W6 splits in two, and only half is reachable

The edit is small — ~25-30 lines of API calls in `sim_runner.py:RosBridge.__init__`
(`:546`, inside the existing `og.Controller.edit` at `:570-643`) — and a
**complete reference implementation already exists in this repo** at
`tools/isaac_5_1_ros_camera.py` (node types `:139-150`, wiring `:637-707`,
OpenCV pinhole intrinsics `:587-607`). `PCam` is present in all three delivery
arenas.

Two risks the line count hides, and one hard block:

- **Rate.** `sim_runner` renders only when a scan is due, floor 12 Hz. The camera
  runs ~12 Hz **and variable**, not the declared 15.0.
- **`/scan` coupling.** That rate is *calibrated, not derived*; the file warns it
  "will drift if anything changes render timing", and `/scan` feeds slam_toolbox
  and Nav2's costmaps. A prior ~21 s `/scan` blackout is documented. **This can
  break the delivery this session is banking.**
- **The certificate cannot be green on domain 67.** D-20 allocates 42/43 as the
  corridor planes; the runner *refuses* 42/43 at `corridor_profile_run.sh:167-173`;
  and there is **no `/clock` on 67**, so `clock_advancing` fails. Publishing
  `/clock` from `sim_runner` would break a fleet-wide invariant stated three
  times in that file.

ADR 0026's crossing numbers were taken under **camera-only load** — its own text
names a full A-plane as "the unrun stronger test" — and image crossing already
measures 0.954 / 0.9745 / **0.9265** against a 0.95 floor.

**Footage of a real delivery is reachable. An isolation certificate over a real
delivery is not, this session.** The estimator needs only the first.

### 8. The dataset cannot produce a speed — but it can pre-qualify the estimator

3000 labelled frames are **static poses with no timestamps**. So a *speed*
requires footage. But `out/datasets/p_cam_v1/dataset.json` carries ground-truth
pose **and** the tight box for all 3000 frames, with a 619-frame held-out val
split — a complete, **zero-simulator, zero-GPU-lock bench** for the geometry.

Missing entirely: a detector **inference** script (nothing loads the 80 MB
checkpoints), track association, pixel→world.

Reusable as-is: `GateSpeedEstimator`, `ViolationDetector`, `MarkerMap`, and
`tools/synthetic_observer_report.py`. Intrinsics cross-check two independent ways
to **fx = 417.032**.

### 9. The camera path and the Nav2 path are different runs, and neither is both

| | `/p_cam/image_raw` | stamps | `/clock` | A's motion | what A looks like |
|---|---|---|---|---|---|
| `run_demo.sh` → `isaac_5_1_ros_camera.py` | yes | sim time | yes | scripted, constant speed | **v1 stand-in cube** |
| `corridor_profile_run.sh` → `sim_runner.py` | **no camera** | wall clock | none | Nav2 + physics | **the yahboom twin** |

The detector was trained *exclusively* on the twin —
`tools/replicator_p_cam_dataset.py:30-34`: *"A detector trained on the cube would
learn a box."* So the existing camera path is **out of distribution**, and the
ratified "footage of REAL deliveries" is also the only in-distribution path. The
fleet edit is justified on both grounds.

A one-line de-risk exists: `isaac_5_1_ros_camera.py:80` hardcodes
`ROBOT_PRIM = "/World/Actors/A"`. Making it `--robot-prim` and pointing the
adapter at the **arena** USD (which holds both `PCam` and the twin at
`/World/Robot`) gives in-distribution frames, sim-time stamps, `/clock` and exact
truth — scripted, not Nav2. **That is the U8 abort fallback, not the plan of
record.**

### 10. Three estimator design corrections, measured before implementation

- **Do NOT project onto the route polyline.** It buys nothing — bearing is
  already the well-constrained axis — it injects the truth generator's own
  trajectory, and it double-counts `path_axis_fraction` for **+0.78% bias**. Use
  **world X**, exactly what `ArucoStationEstimator` already returns
  (`estimator.py:354`).
- **Bottom-centre is the right pixel**, with a **+65 mm bias flat across range**
  that therefore cancels in a speed difference. Do not correct it. But the
  **box-edge convention is worth 6%**: `y_max` as-is drifts 38 mm across the
  enforcement window, `y_max + 1` drifts 4 mm. **Pin it first, by experiment.**
- **`GateSpeedEstimator` is biased HIGH in this regime.** At 0.22 m/s and 15 Hz,
  A advances 14.6 mm/frame against 15.8 mm of station noise — noise exceeds
  displacement, and its re-crossing behaviour shortens the interval at both ends:
  **+1.0 to +3.5%, biased high**, the worst direction for enforcement. A ±0.30 m
  least-squares window fit is unbiased at 1.4% sd, yields **5 gates instead of
  4**, survives the measured 7% frame loss, and is correct under acceleration.
  **Add `WindowSpeedEstimator` beside it; do not modify `GateSpeedEstimator`** —
  four test files assert its semantics.

Sensitivity, for the record: `dstation/dpy` = 0.048 m/px at the corridor entry,
0.011 m/px at the last gate. A leaves frame at X = 3.31 m, so only the straight
leg is estimable.

---

## Queue

| # | unit | box | skip-edge |
|---|---|---|---|
| **U0** | This plan | 15 m | never |
| **U1** | **Bank correction 2** — supersede the stale block; promote the four post-convexity greens + NOTES | 45 m | never; first deliverable |
| **U2** | Defect: `route_to_delivery_m` + arming-window re-base, one commit, **test first** | 60 m | if the re-base cannot keep bag 113859's 5.699 m arming, STOP and record — do not ship half |
| **U3** | Defect: `ARRIVED_UNPROVEN` cancel path at `:445` | 45 m | drop if U2 overruns |
| **U4** | **ADR 0033**, superseding 0029 and 0031's arrival clauses | 60 m | never — U5 is meaningless without it |
| **U5** | Acceptance runs, three profiles, dock-ON, lens up | 90 m | 3 consecutive nominal, then wide_corner gated, uniform reported; on ≥2 infra failures bank what exists |
| **U6** | Governor-vs-stub bag check | 15 m | drop freely |
| **U7** | **Detector inference script + val-split bench** — no simulator, no GPU lock | 60 m | never — free, and it pins a 6% error before any run |
| **U8** | **Fleet edit** + footage of two real deliveries | 90 m | **abort on any `/scan` regression** — revert the fleet commit, keep the delivery, fall back to `--robot-prim` and label the footage scripted |
| **U9** | **Estimator v0**, offline on U8's footage | 90 m | the headline; protect it |
| **U10** | ArUco baseline A/B *skeleton* only | 30 m | only if U9's number exists |
| **U11** | Matcher A/B, revert-by-default | 90 m cap | **first dropped**; only after U5 is banked |

**Critical path:** U7 → U8 → U9. U7 sits *ahead* of the footage because it costs
no simulator and no GPU lock, and a 1 px box-edge convention error is worth 6% of
the speed number — cheaper to pin before the run than to discover inside it.
Under time pressure U3, U6, U10 and U11 go first.

---

## Evidence bar per unit

| unit | bar |
|---|---|
| U1 | Four promoted run summaries with sha256; one `NOTES-acceptance-*.md` stating the `--allow-contract-fail` caveat and that these predate ADR 0033 |
| U2 | A test that fails on the current sum and passes after; bag 113859's 5.699 m arming still admitted, asserted **by name** |
| U3 | Dock tests green; a test that a refused DOCK still cancels and A stops |
| U4 | Index row **and** mermaid node in the same commit, or `test_repository_contract.py:96` fails |
| U5 | Per-run JSON; three consecutive nominal meeting ADR 0033; map-frame reported **ungated**; reds committed as findings, in bold |
| U6 | JSON of `/cmd_vel_raw` vs `/cmd_vel` divergence with A's distance to the stub |
| U7 | Bottom-**edge** error vs range on the 619-frame val split — *not* centre error, which `localisation.json` reports and the estimator does not consume; the edge convention pinned by experiment; end-to-end station error vs GT pose |
| U8 | `/scan` rate before and after the camera, same profile — the regression check, not a formality; two delivery bags carrying `/p_cam/image_raw` |
| U9 | **Per-gate speed error against evaluation-plane truth from a ±0.30 m window fit with zero fitted parameters, and no target this session** — the number exists before any number is judged. Reported with: gate coverage (of 5), per-frame station σ, the truth source named, and `GateSpeedEstimator` as an A/B so its measured high bias is visible rather than inherited. A number with no coverage figure is not a result |
| U10 | Harness runs end to end with the learned arm; baseline arm asserts NotImplemented |
| U11 | `travel_registered.py` before and after, both recorded in `degeneracy-study.md` whatever the outcome |

---

## Decisions

| | question | default taken |
|---|---|---|
| **D1** | Write ADR 0032 this session? | **No — park.** Writing it before U9 would pin a resolution on detection rate, which `docs/evidence/detector/NOTES.md:111-125` explicitly argues is the wrong number. U9's speed error is what 0032 should turn on |
| **D2** | Track association | **Single highest-confidence detection per frame + a plausibility gate** — back-projected point inside the drivable polygon and within 4σ of the extrapolated station. One robot, 0.9927 detection, so a tracker is unjustified; but the dataset has **no distractors** (`replicator_p_cam_dataset.py:71`), so false positives are plausible and untested. ~10 lines, and it turns a silent wrong answer into a logged rejection |
| **D3** | The unreachable speed policy | **Record now, pin next session.** Add the measured envelope to evidence and name it in ADR 0033's context; do not write the width→limit ADR — the brief scoped 0032/0033 only. Top morning decision |
| **D4** | If U8's camera regresses `/scan` | **Revert the fleet commit immediately**, keep the delivery, take footage from `--robot-prim` on the arena stage instead, labelled *not a Nav2 delivery* |

---

## Delegated vs not

**Delegated, under the granted exception:** one fleet commit to
`robot-fleet/src/yahboomcar-ros2/tools/sim_runner.py` adding an env-gated camera
render product plus ROS2 camera/camera-info helpers. Single commit, listed
separately in the handback, reverted on any `/scan` regression.

**NOT delegated:** everything else fleet-side. No `slam_toolbox`/Nav2 parameter
changes outside U11's capped A/B. No new ADRs beyond 0033 (0032 parked per D1).
No scene topology change. Park-don't-decide for the rest.

---

## Log

*(updated after every unit)*

| unit | status |
|---|---|
| U0 | **DONE** — this document |

---

## Handback

*(written at session end, or on early failure — whichever comes first)*
