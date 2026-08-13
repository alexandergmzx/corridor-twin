# Close the autonomy gate, open Phase 3

> **APPROVED 2026-08-12 17:18 CST. UNATTENDED** — Alexander is away, so the hard
> rules bind in full: git is local-only and `git push` does not exist tonight;
> history is append-only; a commit is a green checkpoint or it does not happen;
> Isaac is single-occupancy under `/tmp/fleet-isaac.lock`; judgment calls are
> **parked, not decided**.
>
> **Budget 5 h — started 17:18, ends 22:18. No new unit starts after 21:48.**
> `date` between units. Branch `gate-green-2026-08-12` from
> `corridor-stabilization-2026-08-12` (`9867360`). The handback is this
> document's final section and is written even if the session fails early.

## Live status

| Unit | State | Notes |
|---|---|---|
| P0 session plan | **DONE** 17:20 | this file |
| P1 the startup circle, fixed | **DONE** 17:56, inside the box | culprit was the CONTRACT CHECK driving; 3/3 runs green on all three criteria |
| P2 landmark containment | **DONE** `e17f83d` | window derived = 0.900 m (= 3.0 x 0.30, two derivations agreeing); spawn control + fail-closed |
| P3 scan path, wide_corner + uniform | folded into P4 | a P4 run on a profile IS its smoke; acceptance read from each run's relay log |
| P4 acceptance runs | **DONE, RED** `ea85a9e` | both gated profiles red on ADR 0029's two; **uniform's transit gate PASSES**; 13 runs, 6 results, 6 reruns |
| P5 P-camera candidates | **DONE** `3bce1f3` | **P cannot see the corridor from where P stands**; memo + geometry; frames outstanding |
| P6 paper debt | **DONE** `21ba0af` | ADR 0030 pins scale + mask decision; 0029 renamed, acquittal row scoped; loop-closing verified `65b7c0b` |
| P5 3-D check (added) | **DONE** | the corner mast clears 5/5 with `scene.occlusion`'s raycaster; memo updated |

## Corrections to the plan, made while executing

1. **P1's leading mechanism was wrong.** The plan expected A to be chasing a
   moving map frame. `map→odom` is identity for the first 88 s, so it was not.
   The culprit is `check_isaac_contract.py`, which drives a 0.4 m-radius arc on
   `/cmd_vel` -- bypassing the governor -- to prove the twin responds, 70 s
   before Nav2 says anything. Recorded in `NOTES-startup-fixed.md`.
2. **P3 is folded into P4.** A gated run on a profile exercises that profile's
   scan path, so its acceptance (accepts from scan 1, zero fail-open) is read
   from each run's own relay log rather than from two extra Isaac sessions.
   Fewer sessions, same evidence.
3. **A defect I introduced cost a run.** P2 read `target["ekf_topic"]` from a
   table that never carried it; the first gated run died in the nav gate's
   constructor, sent no goal, and produced no delivery. Fixed and pinned
   (`f49ab35`). The crash machinery worked -- `traceback in runner.log` is in
   that run's manifest -- and it exposed two more defects worth more than the
   run cost: the session-bag lookup used `find -newer run.json`, which is
   rewritten during the run and so never matched, and the containment's bearing
   test was written to pass by construction. Both fixed and measured.
4. **One session was orphaned by my own error** at 17:44 -- a tool timeout above
   its ceiling killed the runner mid-run, leaving Isaac holding 2987 MiB and the
   lock held by a dead PID. Cleaned per the rules: `simctl stop`, verified, lock
   released. Runs are backgrounded one per call since.

## Session start state

Host clean at 17:18: GPU 607 MiB of 16303, RAM 19 G free, disk 120 G free,
domain 67 carries no un-namespaced ROS nodes. A **stale** Isaac lock from my own
16:47 run was removed — PID 2001914 dead, owner `corridor-profile-run
nominal_m6_n3`, the run the watchdog killed.

## What the stabilization session hands over

Green: ruff clean, 387 passed / 1 skipped, colcon 140 tests 0 failures.

Working that was not before: the arena and the plan are the same scenario
(hash-checked before every run), the gate divides by the observed window, the
lens runs, evidence is session-scoped with a result/rerun/**crash**
classification, the domain is preflighted and teardown escalates, the scan
filter accepts the corridor from scan 1, and the wheel radius is calibrated —
midpoint drift **0.159 → 0.0097**.

Two things it deliberately left open, and both are P1/P2's inputs:

1. **The startup circle is measured, named and NOT fixed.** All three prior
   hypotheses are falsified by log: nothing commands A before the goal
   (`commands_before_goal: 0`, three runs of three), and the first recovery
   `Spin` fires at t = 173 s when A is already parked at the far end. **The
   recovery-Spin brief is dead and is not to be implemented.** What ground truth
   shows is 253° of turning over 1.06 m in the first 60 s, ending 0.2 m *behind*
   spawn, then **56 s stationary**, then a transit that works and reaches
   0.0625 m from the delivery standoff.
2. **A drives 3.4 m past B and stops**, because with docking off the arrival
   gate is a map-frame goal the drifting map never satisfies (ADR 0029).

## P1 — the startup circle, fixed. **90 min hard timebox.**

The culprit is the local controller, commanding rotation *after* the goal while
the map is still nearly empty. The leading mechanism, and the first thing to
measure, is that **A is chasing a map frame that is still moving**: SLAM's
`map→odom` correction jumps as its first scans arrive, A's *believed* heading
swings without the robot moving, and the controller physically rotates to
correct an error that only exists in the frame.

**Diagnosis first, offline, no GPU.** Both of this afternoon's bags carry `/tf`,
`/odom` and `/sim/ground_truth`. Compare map-frame yaw against truth yaw over
the first 60 s and plot the `map→odom` correction beside the commanded rotation.
If the correction moves and truth follows it, the mechanism is confirmed and the
fix is a readiness gate, not a controller parameter.

**Fix, if confirmed:** hold the goal until the map frame has settled — a bounded
wait on `map→odom` being stable within a threshold over a window — rather than
sending it the instant `bt_navigator` reports ACTIVE. Runner-side, no Nav2
parameter touched.

**Acceptance** (ground truth, three consecutive runs): cumulative pre-transit
rotation **< 45°**, forward progress begins **< 30 s after goal-send**, and
transit accuracy does not regress — closest approach **≤ 0.15 m** of the
delivery standoff.

**Skip-edge at 90 min:** ship the measured mitigation that gets tonight's runs
green, and write the proper fix up as a parked unit with its evidence. Do not
spend P4's budget here.

## P2 — landmark containment, re-derived at the committed scale

Per Alexander's 14:52 decision, U3's numbers are re-derived rather than carried:
the arming window is **derived from the scaled manifest**, not the briefed 3.0 m.
The as-run route is **7.380 m** at nominal (pinned in
`test_scenario_as_run.py`), against 24.601 m authored — so a 3.0 m window
authored at the old scale is 0.9 m here. Compute it from the manifest at run
time; do not write a literal.

Arming requires **all** of: travel-integrated distance (EKF, A's own estimate,
never truth) ≥ route − window; map-frame goal proximity ≤ window; detection
bearing within **±60°** of the goal bearing. Spawn-region negative control in
the tests, paired with a positive guard so it cannot pass vacuously. **Docking
stays OFF in every gate run.**

## P3 — scan path for wide_corner and uniform

Nominal is green: 349 passed / 10 dropped, no fail-open. The two constants are
already manifest-driven and per-profile, so this is a verification unit: one
smoke each, acceptance **accepts from scan 1, zero fail-open**.

## P4 — the acceptance runs

nominal + wide_corner **gated**; uniform runs and is **REPORTED, never gated**
(ADR 0022). Masked-oracle map ≤ 0.20 m, gate JSON pass at pinned thresholds,
P1's startup criterion, zero landmark events, dock off, lens up, session-scoped
artifacts with arena and manifest hashes.

Reds are committed findings, in bold, with their artifacts. **No tuning past
this list.** If green, one additional nominal pass with `--dock` enabled as a
separate, clearly-labelled demo-candidate run.

## P5 — Phase 3 opens: P-camera candidates

**Runs even if P4 is red.** 2–3 poses computed from the SCALED manifest, each
with per-gate line-of-sight, distance and incidence angle to all four
enforcement stations; one 640×360 frame each through the ADR 0009 adapter in a
single Isaac session; saved under `docs/evidence/p_cam_candidates/` with a
one-page memo: the table, and what each pose costs the detector.

**CHOOSE NOTHING.** The memo is for Alexander.

## P6 — paper debt (no Isaac, parallel-safe, separate commits)

- **ADR 0030** pinning the committed scale constant and the robot-scale build
  default, its derivation, the oracle-mask decision, and the 0.12 m screen
  margin — declaring every prior scale value superseded.
- **ADR 0029**: rename to the house register; scope the "odometry calibration
  acquitted" row to its bench-sweep conditions.
- Verify `do_loop_closing` in the committed slam config matches the
  falsified-hypothesis revert.

## Rules for tonight

Two attempts at any failing command for the same reason, then record and move
on. Every gate run writes JSON; thresholds printed and enforced from one
constant; infrastructure failures are reruns, twice at most, classified
explicitly. Fleet touches only if a P1 fix genuinely requires one — single
commit, listed separately. **Nothing pushed.**

## Session close

Ended 19:25, **2 h 07 m of a 5 h budget** — the queue finished early rather than
running out of clock. `bash tools/check_workspace.sh` green at close: ruff clean,
**400 passed / 1 skipped**, colcon build 4 packages, colcon test 140 tests,
0 errors, 0 failures. Machine left clean: no residents on domain 67, Isaac lock
free, GPU at idle.

**Nothing pushed.** 14 commits on `gate-green-2026-08-12`; one fleet commit
(`aae2617` in `yahboomcar-ros2`), listed separately below.

### Four defects of mine, each caught by an instrument built earlier the same day

| defect | caught by | cost |
|---|---|---|
| `ekf_topic` read from a table that never had it | traceback → run manifest | 1 run |
| session-bag lookup used `-newer run.json`, which is rewritten during the run | "startup criterion unmeasured" | a measurement |
| recorder capped shorter than the nav window it must outlive | reading the run's own log | 1 run |
| "goal not accepted" classified as infrastructure regardless of whether the robot drove | the 7.865 m transit it discarded | 1 run |

The fourth is the one to remember: a rule I wrote at 18:42 threw away the best
transit of the night at 18:46. The gate cannot tell a refused goal from a lost
acceptance response — from where it stands they are identical — so the runner
now asks the recorder how far the robot actually went, using the transit gate's
own 1.0 m threshold. Both branches were verified against the two real artifacts
before the fix was committed.

### One fleet commit, for separate review

`yahboomcar-ros2` `aae2617` — `check_isaac_contract.py` no longer asserts the
robot moved when the caller passed `--speed 0 --turn 0`. Required by P1: the
corridor now runs that check without motion, and the assertion had become a
permanently-false failure. Every other caller is unchanged.

## Handback — the scoreboard against the 2026-08-04 interview corrections

### Correction 1 — communication-domain isolation: **DONE, unchanged tonight**

A on 42, P on 43, one-way gateway with a declared allowlist. Certificate green
with its mutation control red; producer 0.9995, image crossing 0.954 at the
pinned 640×360. ADR 0020 (decision), ADR 0026 (verified live).
Evidence: [`docs/evidence/crossing/NOTES.md`](../evidence/crossing/NOTES.md).

Nothing in this session touched it, and nothing needed to.

### Correction 2 — autonomous navigation: **works; the GATE is red on the map**

Governed Nav2 on a live SLAM map, no authored route, docking off.

| profile | closest approach | drift | yaw scale | duplicate wall | transit gate |
|---|---|---|---|---|---|
| `nominal_m6_n3` | **0.110 m** | 0.0137 | 1.166 ❌ | 0.840 ❌ | FAIL |
| `wide_corner_m6_n4_5` | **0.020 m** | 0.0606 ❌ | 1.108 ❌ | 0.780 ❌ | FAIL |
| `uniform_m6_n6` *(reported)* | 0.083 m | **0.0041** | **1.060** ✅ | *(save failed)* | **PASS** |

Artifacts: `out/evidence/robot-a-gate/20260812-184944-robot1-nominal_m6_n3/`,
`…-185907-robot1-wide_corner_m6_n4_5/`, `…-191347-robot1-uniform_m6_n6/`, each
with `run.json` carrying the git SHA and the arena/manifest hashes. Whole
session: [`session-runs-20260812.json`](../evidence/robot-a-gate/session-runs-20260812.json).
Write-up: [`NOTES-acceptance-20260812.md`](../evidence/robot-a-gate/NOTES-acceptance-20260812.md).

**A delivers.** It leaves cleanly, drives its seven metres, and arrives 2–18 cm
from the standoff. This morning the same measurement read 5.754 m, because the
plan and the arena were different scenes.

**The gate is red on the map**, and on the yaw scale that feeds it — ADR 0029's
open blocker, untouched and untuned. `uniform`'s fully green transit gate is the
useful datum: on the one profile where the matcher has an easy time, every
number falls into place, yaw included.

### Correction 3 — active AI/ML: **the camera pose is the blocker, and it now has a memo**

Phase 3 could not start because nobody had placed P's camera, and placing it
turned out to be a decision rather than a task: **ADR 0019's corner screen, which
hides P from A, also hides the corridor from P.** From a camera at P's own
height, 0/5 enforcement stations are visible.

Measured with `scene.occlusion`'s own 3-D raycaster (72 opaque triangles):

| candidate | 3-D line of sight | in frustum | usable | range to A |
|---|---|---|---|---|
| at P, 0.21 m | 0/5 | 5/5 | **0/5** | — |
| at P, 0.63 m | 0/5 | 5/5 | **0/5** | — |
| **at P, 1.50 m mast** | **5/5** | 5/5 | **5/5** | 2.29–4.68 m |
| north wall, west of the screen | 5/5 | 4/5 | 4/5 | 1.05–2.80 m |
| north wall, midpoint | 5/5 | 1/5 | 1/5 | 1.26 m |

**Nothing chosen.** [Decision memo](../evidence/p_cam_candidates/NOTES.md).

#### What the detector pipeline needs next, in order

1. **The pose decision.** Everything below keys off it.
2. **Move the render product.** `tools/isaac_5_1_ros_camera.py` still mounts the
   single camera at `/World/Actors/A/CameraMount/FrontCamera` — A's v1 camera.
   The chosen pose becomes a P-owned prim, and the one-render-product budget is
   preserved by moving it, never by adding a second.
3. **Replicator dataset spec.** Labels are A's pose and extent in P's frame,
   derived from simulator truth *on the evaluation plane only*. Domain
   randomisation over lighting, A's yaw, and the corridor variant. The three
   profiles give three geometries for free. Frame budget and the train/val split
   by profile are the two numbers to pin.
4. **Training harness.** Off-GPU-session: the dataset is rendered once, training
   never holds the Isaac lock. Report per-station detection rate and metric
   error against truth.
5. **ArUco-plate-on-A baseline.** The classical arm. At the mast's 4.68 m, A
   spans ~27 px, so the plate spans a fraction of that — the baseline may need
   its own closer pose, a larger plate, or a longer lens, and that is the first
   argument the chosen pose forces.

### Tree state at handback

One untracked file, **not mine and not touched**:
`corridor-v2-adr-pack.md` (24 KB, mtime Aug 11 11:11, never committed on any
branch). It is the working draft that ADRs 0020–0024 were split out of, and its
own header says so. It has survived three sessions untracked. Either it is spent
and can go, or it wants committing as a historical draft — a call for you, not
for an unattended session.

### Parked for Alexander

1. **The P-camera pose** (above). Nothing proceeds without it.
2. **The map divergence** remains the open blocker: duplicate-wall 1.00–1.56 m
   against 0.20, on a masked map whose oracle is 0.000. The linear channel is
   acquitted by calibration; the yaw fusion anomaly (0.14×–23.4×) is fleet
   territory and now carries fleet OI-23.
3. **A drives 3.3–3.5 m past B** with docking off, because the arrival gate is a
   map-frame goal the drifting map never satisfies. Containment now exists to
   make the docking path safe; whether the demo runs docked is a scenario call.
