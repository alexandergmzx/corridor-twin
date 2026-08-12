# Making A actually find B — autonomy method

> **APPROVED 2026-08-11 21:44 CST.** Budget 5 h — ends 02:44, no new unit
> starts after 02:14. Binding; status updated after every unit; first thing
> re-read after any context compaction; the handback is its final section and
> is written even if the session fails early.

## Live status

| Unit | State | Notes |
|---|---|---|
| U0 session setup | **DONE** 21:46 | `26b3387` — branch, plan, 1/3 scale adopted with the factor derived (B was 0.21 m from a wall, robot needs 0.27) |
| U1 standoff goal + scale fix | **DONE** 21:52 | `a3034f9` — goal was inside B (lidar sees render geometry); standoff 0.6 m validated against is_clear; actors now scale (B 0.45 -> 0.15 m) |
| U2 TF staleness | IN PROGRESS | started 21:52 |
| U3 DWB vs MPPI measured | pending | |
| U4 three profile runs (TRANSIT only) | pending | |
| U5 ADR 0028 | pending | |
| U6 terminal docking | stretch, after Phase 3 | |


## Context

The corridor twin has no goal-seeking. `corridor_sim_gate.py:285` drives a
hardcoded open-loop pattern ("straight passes with brief settles"), the governor
only brakes at walls, and `corridor_nav_gate.py` sends **one** `NavigateToPose`
at B that has never succeeded. That is a mapping rig plus a wall-avoider — the
diagnosis that prompted this plan is correct.

**Correction to what I reported last session.** I said the 1/3-scale run failed
because the planner "couldn't create a plan through the 0.6 m corner". That was
the *0.2*-scale run. At 1/3 scale the planner **succeeded**:
`nav-launch-robot1-nominal_m6_n3.log:136-142` shows `Begin navigating … to
(5.22, -3.34)` then `Passing new path to controller` three times — Nav2 planned
through unmapped space and the global side worked. The abort at `:143-148` is
`Transform data too old when converting from odom to map` (~312 ms lag against
`transform_tolerance: 0.3`, `nav2_robot1_corridor.yaml:83`) →
`Unable to transform robot pose into global plan's frame` → `follow_path`
aborted. **The blocker is controller TF staleness, not planning or geometry.**

## Two structural blockers, found while planning

1. **The goal is inside B.** `corridor_nav_gate.py:79` aims at
   `manifest.actors.b_xyz_m` — B's *centre*. B is a 0.45 m box
   (`usd_authoring.py:276`) and, while it carries no `PhysicsCollisionAPI`, the
   RTX lidar sees **render** geometry, so B is an obstacle in the costmap. The
   goal sits inside its own inflated footprint and is unreachable at a 0.15 m
   tolerance, independent of TF. B being an obstacle is correct — a person is
   one. Aiming at its centre is the bug.
2. **B, A's stand-in and the corner screen never scaled.** Hardcoded at
   `usd_authoring.py:276`, `:299` and `geometry.py:565`, so in a 1 m corridor B
   is still 0.45 m — and that footprint sets the standoff, so it must be fixed
   first.

## What the constraints permit

- **ADR 0022:15-17** — *"v1's authored line and waypoints were read, correctly,
  as a level indicator."* The route must stay **emergent** (0022:87-90).
- **ADR 0023:44** — *"Live slam_toolbox map, no prior map, no AMCL."*
- **Neither constrains the goal source.** ADR 0023 never mentions B or the
  destination; the A-side contract test (`test_repository_contract.py:253-265`)
  forbids only *police* tokens — `b_xyz` is P-side only (`:289-293`) — and
  `corridor_nav_gate.py:79` already reads B from the manifest as precedent.

**A may know the destination; it may not be given the route or the map.**

## The method

**Goal-directed navigation in a partially-known environment.** Three layers, none
of which authors a path:

1. **Optimistic global planning** — NavFn with `allow_unknown: true`
   (`nav2_robot1_corridor.yaml:109`) plans through unknown cells; the rolling
   global costmap (`:216-222`) keeps the goal in bounds. Proven working on disk.
2. **Continuous replanning** — Nav2's stock
   `navigate_to_pose_w_replanning_and_recovery.xml` replans as SLAM fills the
   map. No BT xml is overridden in either repo, so this is already live. **The
   route emerges from replanning**, which is what ADR 0022:87-90 pins.
3. **Governed local control** — unchanged (ADR 0023:39-42).

### Binding scope constraints (ratified)

- **The arrival gate is UNCHANGED: Nav2 `SUCCEEDED` and ≤ 0.15 m map-frame
  error.** The landmark is terminal-docking refinement, **never** the arrival
  mechanism, and **the demo must pass with the detector disabled.**
- **There is NO search behaviour.** A's motion is 100 % governed Nav2
  `NavigateToPose`. Docking adds a **perception node and a small state machine —
  never a motion primitive, never raw `cmd_vel`, never patrol or exploration.**
  `sim_patrol` is bench tooling and does not appear in the mission.
- **The MS200 is 360°, so acquisition requires zero motion.** A detects while
  driving or standing; it never moves in order to acquire.
- **Detection is GEOMETRIC** — cluster scan points, fit a circle of the
  landmark's authored radius, single source of truth = the manifest. **Never
  intensity-based**: sim/real intensity fidelity is an unowned contract question.
- **Docking is a stretch unit after the three profile runs and does not precede
  Phase 3** (the learned detector — the one open interview correction). Parkable
  without breaking anything. **The profile runs execute TRANSIT only.**
- **The evaluation plane measures world-frame delivery error** from simulator
  truth (evaluation-only, CLAUDE.md invariant 1).
- **B remains a visible prop** for P's camera and the viewer either way.

## Terminal docking — bounded state machine (binds U6)

| State | Behaviour |
|---|---|
| **TRANSIT** | One `NavigateToPose` to B's nominal address (start-relative, per ADR 0028). Nav2 owns everything, including recoveries. |
| **ACQUIRE** | Detector runs on `/scan` throughout but is **ARMED only within `R_arm = 3.0 m`** of the goal in map frame — this is what kills false positives from corner geometry. Accept on fit residual below threshold **AND** k-of-n frame agreement (**3-of-5**), so a single phantom frame cannot trigger. A does not move to acquire. |
| **REFINE** | On confirmed detection: refined goal = landmark centre − standoff along the approach bearing, **standoff ≥ 0.6 m** (robot_radius + inflation — the landmark is lidar-visible, so the costmap treats it as an obstacle and the refined goal must sit outside its inflation). Issue **exactly one** further `NavigateToPose`. **One refinement maximum, ever. No re-refine loop.** |
| **DELIVERED / DELIVERED_UNREFINED** | If the detector never confirms within **10 s** of Nav2 `SUCCEEDED` at the nominal goal: stop, state `DELIVERED_UNREFINED`, **arrival gate still green**. |

The same standoff construction applies to the **nominal** goal in TRANSIT — B is
an obstacle for exactly the reason the landmark is, so aiming at either centre
fails. One constant, one rule, validated with `is_clear`.

**Per-run evidence (JSON, gate discipline):** detected y/n, fit residual,
frames-to-confirm, refinement distance, and the evaluation plane's world-frame
delivery error **with and without refinement** — that pair is ADR 0028's
validation data.

## Inventory — everything needed is installed and unused

| Asset | Where | Status |
|---|---|---|
| `allow_unknown` | `navfn.hpp:128-131`; set at `nav2_robot1_corridor.yaml:109` | already on |
| Rolling global costmap | `nav2_robot1_corridor.yaml:216-222` | already applied |
| `nav2_mppi_controller` | installed | unused; config uses DWB |
| `nav2_simple_commander` | `robot_navigator.py:161,185,221` | installed, never used |
| `nav2_route` (route graph) | installed | **rejected** — a route graph is an authored route (0022) |
| B's position | `manifest.actors.b_xyz_m`; used at `corridor_nav_gate.py:79` | shipped |
| `is_clear` free-space oracle | `scene/geometry.py:610` | standoff validation + detector tests |
| Frontier exploration | **absent**, none apt-available for Jazzy | **not wanted** — there is no search behaviour |

## Unit queue

| # | Unit | Box | Gate |
|---|---|---|---|
| U1 | **Standoff goal + scale fix.** Drive B / A-stand-in / corner-screen dimensions from config so they scale; derive the nominal goal as a standoff pose beside B, validated with `is_clear`. Unit-tested, no GPU. | 60 m | Goal provably in free space on all three profiles. |
| U2 | **TF staleness.** Measure the real map→odom lag under load; raise `transform_tolerance` on controller and both costmaps. | 45 m | An A→B attempt gets past `follow_path`. If the lag far exceeds 312 ms, root-cause it rather than inflating tolerance without limit. |
| U3 | **DWB vs MPPI, measured.** Jazzy ships MPPI, reported better in tight spaces but prone to jitter in narrow corridors. Both, in the 1 m corridor, JSON each. | 60 m | Decide on numbers. If MPPI is not clearly better, DWB stays. |
| U4 | **Three profile runs — TRANSIT only.** Fresh Isaac session each. Report action status, map-frame error, **world-frame delivery error from truth**, route length and duration as emergent. | 90 m | Arrival gate unchanged. Red runs are committed artifacts; no mid-gate tuning. |
| U5 | **ADR 0028.** Records the adoption of this method, its validation by the U4 runs, the start-relative nominal address, and **honestly that adoption preceded measurement**. Index row + decision map same commit. | 45 m | Accepted on either outcome; negative result in bold. |
| — | *Phase 3 (learned enforcement detector) takes priority here* | — | — |
| U6 | *(stretch, parkable)* **Terminal docking** exactly as specified above: perception node + bounded state machine, no motion primitive. Disabled by default. | 90 m | Detector unit-tested on synthetic scans incl. a wall and a corner at the same range. **U4 must still pass with it disabled.** |

U1 and U2 are the two blockers between here and the first successful autonomous
delivery. Everything after is measurement and record-keeping.

## Verification

- **U1**: `pytest` — standoff pose is `is_clear` on all three profiles and sits
  outside B's inflated footprint; scaled B is plausibly sized in a 1 m corridor.
- **U2/U3/U4**: `bash tools/corridor_profile_run.sh --robot robot1 --profile <p>
  --gated --allow-contract-fail --domain 67`, with `CORRIDOR_ARENA_DIR` /
  `CORRIDOR_MANIFEST` pointed at the robot-scale assets. Success is
  `action_status: SUCCEEDED` **and** map-frame error ≤ 0.15 m — ADR 0023:89-92
  names the status-unchecked anti-pattern so it cannot recur.
- **U6**: `pytest` on synthetic `LaserScan` fixtures — landmark present, absent,
  wall at the same range, corner at the same range, partial occlusion, and a
  single phantom frame (must **not** confirm under 3-of-5).
- Every unit: `ruff check` + touched tests green before commit; full
  `bash tools/check_workspace.sh` green before handback. Isaac lock held for
  every GPU run; scratch domain 67/69; one session at a time.

## Not delegated — parks to the handback

- Re-pinning the speed policy (ADR 0023:52-59); raising the governor or Nav2
  velocity caps (0023:116-121).
- `git push`; any write outside this repo.
- `robot_radius: 0.12` remains inherited from robot2 and unverified for robot1.

---

# Night session, 2026-08-11 22:25 onward — the motion-policy correction

The plan above was interrupted mid-U2 by a motion-policy violation and an RViz
observation of a diverged map. What follows is the ordered sequence that was
worked instead, and what each step measured.

## Step 0 — the mission had three motion sources (fixed, `e99c9fa`)

`simctl` step 7 launches `sim_patrol` unless told otherwise: *"patrolling: 1.0 m
legs at 0.18 m/s, publishing /cmd_vel_raw"*. It is present in `simctl-patrol.log`
of **every session directory this project has ever produced**. Alongside it,
`corridor_sim_gate.py` drove a straight-pass warm-up before the nav stack even
launched, and then Nav2's controller joined on the same topic.

That is the square-patrol behaviour observed in the viewport, and it makes every
odometry, drift and map number taken before tonight a measurement of three
controllers fighting over one robot.

Fixed: `--no-patrol`; warm-up drive deleted; the gate keeps its instrument and
loses its publisher (`--observe-only` creates no publisher **object**, so the
mode cannot regress into commanding motion) and now records *concurrently* with
the transit. Artifacts gained `motion_source` and `observed_s`.

**slam_toolbox needs no warm-up.** It maps during transit — confirmed this
session: *"map publishing: 5 updates, 8.22 m across"* with the robot stationary.

## Step 1 — the odometry chain is ACQUITTED (`c874129`, `2c2ac59`)

Configuration first: `ekf_filter_node` is the sole publisher of
`odom -> base_footprint` (`ekf_sim_pnfix.yaml:93-99`); it takes vx from the
wheels (`odom0_config` index 6) and yaw rate from the IMU (`imu0_config` index
11); `laser_odometry` publishes only the `/odom_laser` **topic**, not TF; the
corridor nav launch adds no second EKF and no static transform. `session.json`
records `"ekf": "pn-fix"`. Wheel yaw is structurally absent from the chain.

Then measured, because that is a claim about a config file — a rate sweep,
each rate driven as its own leg in one direction:

| commanded rad/s | truth deg | EKF deg | EKF/truth | wheel/truth |
|---|---|---|---|---|
| +0.3 | 93.9 | 97.7 | 1.0408 | 4.0311 |
| −0.3 | −97.4 | −98.8 | 1.0142 | 3.6665 |
| +0.6 | 182.9 | 184.9 | 1.0109 | 3.1197 |
| −0.6 | −181.0 | −173.4 | 0.9577 | 3.3055 |
| +1.0 | 305.6 | 311.3 | 1.0185 | 2.7951 |
| −1.0 | −304.8 | −307.8 | 1.0099 | 2.9451 |
| +1.5 | 454.9 | 441.6 | 0.9707 | 2.4597 |
| −1.5 | −468.2 | −457.1 | 0.9763 | 2.4311 |

EKF yaw within **±4%** at every rate and both directions, while the wheel
negative control fires at **2.4–4.0×** throughout — so the pivots genuinely
slipped and the EKF was genuinely under test. Artifact:
`docs/evidence/robot-a-gate/yaw-sweep.json`.

**No launch-assembly fix is needed.** Raw odom is not in the SLAM input chain.

## Step 2 — NOT MET. The map diverges, and it is measured, not eyeballed

`score_slam_map.py` is now run on every profile run (`0553d1d`), with its
`--self-test` negative controls re-exercised each time rather than once at
authoring time. `--reference` is deliberately not passed: that tool's own
docstring (`:28-30`) says its span rows are invalid for an L-shaped space, and
saying so here is what it asks for instead of quietly scoring the wrong thing.

The metric needed calibrating before it could convict anything, because a
tapered corridor's converging walls and its corner could plausibly read as "the
same wall twice". So the **authored** geometry is rendered as the map a perfect
SLAM would produce, from the scene's own `is_clear` oracle, and scored by the
same instrument (`3f05c7e`):

| map | median wall thickness | duplicate wall extent |
|---|---|---|
| **authored (perfect SLAM)** | 0.020 m | **0.000 m** |
| run 22:40 (stationary, 0.42 m) | 0.020 m | 0.300 m |
| run 22:55 (transit, 5.78 m) | 0.020 m | **1.920 m** |
| run 23:15 (transit, no RViz) | 0.020 m | **0.800 m** |

**The authored floor is zero**, so a run's reading is its error in full. The
23:15 map spans 9.78 × 10.28 m for a corridor 5.04 m long and 2.52 m wide.

`Failed to create plan` × 23 is downstream of this. No planner parameter was
touched.

## What the divergence is NOT

Ruled out by measurement, each with its artifact:

1. **Not competing motion sources.** The patrol was gone from 22:40 onward.
2. **Not the odometry chain's calibration.** ±4% across the rate sweep.
3. **Not a rate-dependent gyro fault.** Spread 0.083 across 0.3–1.5 rad/s.
4. **Not an inverted yaw channel.** Signed rotation agrees in sign everywhere.
5. **Not simulator slowdown.** Real-time factor **1.001** on the 23:15 bag.

What remains open is a transit-only yaw scale error. On the 22:55 bag
(`transit-audit-225511.json`): truth rotated **+810.4°**, the EKF believed
**+982.7°** — ratio **1.213**, final heading error **172.4°**, final position
error 1.974 m. Integrating the raw gyro over the same bag gives +884.2°, so
~9% originates in `/imu` itself and the rest is added downstream. The pivots
say this is neither a calibration constant nor rate dependence. **It is not yet
explained.**

## Two of my own instruments were wrong, and both were load-bearing

Recorded because each produced a confident number that was false.

**The gate timed itself on the wall clock** (`2727c0c`). Every "EKF output gap"
— 0.483 s, 0.996 s, 3.052 s, 4.138 s — measured the *recorder*, not the EKF:
this node spins rclpy in a Python loop that also deserializes a growing `/map`
OccupancyGrid, and the messages it missed while blocked were reported as the
EKF failing to publish. From the 23:15 bag, receiver-independent: worst `/odom`
gap **0.398 s** wall, 0.367 s by header, and **zero** gaps over the 0.4 s
threshold. The EKF never stalled. The RViz-starvation hypothesis in `2f58530`
was reasoning built on those numbers and **is withdrawn**; `--no-rviz` stands
on its own merits for an unattended run. Gaps and tracks are now stamp-timed,
which is what CLAUDE.md requires under simulation regardless.

**The yaw metric summed absolute deltas** (`2c2ac59`). That is blind to an
inverted channel — it scores a perfectly reversed signal as a flawless 1.0 —
and it accumulates per-sample noise rather than cancelling it: it scored
**5496° (fifteen revolutions)** for a robot that turned 810° over 4982 samples,
and that artifact was briefly mistaken for runaway recovery spins. The nav log
records **two**. Rotation is now summed signed.

## Where this leaves the plan

U2 is **not** closed: there is still no successful autonomous delivery, and the
blocker is a diverged map rather than anything in the planner. Step 3 of the
sequence (costmap dump, free width at the corner, cost class of the goal cell,
footprint provenance) is deliberately **not** started, because every quantity
it would measure is read off a map that is known bad.

The `1.0 m` corner figure quoted in an earlier session is **retracted**: it came
from the abandoned 0.2-scale iteration. At the committed factor 0.42 the corner
is authored at **1.26 m** and the entry at 2.52 m (`corridor-small.manifest.json`).

## Step 2 continued — what was tried after the divergence was measured

Each of these is a committed change with its rationale, and each was measured.

| change | commit | effect on `duplicate wall extent` |
|---|---|---|
| baseline, three motion sources | — | not measurable; every number void |
| one motion source, patrol gone | `e99c9fa` | 1.920 m |
| `--no-rviz` (hypothesis, since withdrawn) | `2f58530` | 0.800 m |
| gate re-timed on message stamps | `2727c0c` | 1.480 m |
| `max_vel_theta` 1.0 → 0.4 rad/s | `e972c31` | **2.680 m** |

**No change has brought the map near the 0.20 m threshold, and the authored
floor is 0.000 m.** The spread across nominally identical runs (0.8 → 2.7 m)
is itself a finding: whatever this is, it is not a deterministic function of
the parameters touched so far.

### The velocity cap, and why it was right to try even though it failed

The DWB block's own comment described its caps as `[estimate]`. The governor's
real limits are `max_speed` 0.35 m/s, `max_yaw` 1.5 rad/s, and **`max_yaw_near`
0.4 rad/s inside `stop_distance` 0.35 m** (`governor.py:43-51`). In a corridor
1.26 m wide at the corner, a robot of radius 0.12 is within 0.35 m of a wall
for most of its transit, so `max_yaw_near` is the binding cap — and Nav2 was
commanding exactly 1.000 rad/s (max over 9617 `/cmd_vel` samples). The governor
clipped it, so **DWB was predicting trajectories the robot would never execute,
its self-model wrong by 2.5× exactly where the corridor is tightest.**

That was worth correcting on its own terms and remains corrected. It did not
fix the map.

### The yaw chain, measured stage by stage

`corridor_sim_gate` now taps the chain at every stage (`9500260`). On the one
transit where the robot turned enough to compute a ratio:

| tap | rotation | ratio vs truth |
|---|---|---|
| `/sim/ground_truth` | +227.45° | — |
| `/imu` (raw from the twin) | +262.21° | **1.153** |
| `/imu/data` (after madgwick) | — | — |
| `/odom` (after robot_localization) | +365.37° | **1.606** |

A later run caught the missing row: raw `/imu` −48.42° against filtered
`/imu/data` −48.43°. **Madgwick is not the amplifier — it passes yaw rate
through unchanged.** So ~15 points enter at the sensor and ~40 more enter at
`robot_localization`, which has only that one yaw input.

That last step is the open anomaly: a filter that over-reports rotation
relative to its own only source of rotation.

### A weakness in the new gate criterion, stated rather than papered over

`yaw_scale` uses SIGNED net rotation, which is right for a pivot sweep and
wrong for a transit: a robot that turns left and right in equal measure nets
near zero, so the criterion reports UNAVAILABLE on most transits (`the robot
turned only -15.5 deg` on a run of 2.205 m). It has not yet failed a run.
The quantity that did catch every bad transit is the transit audit's
**`max_yaw_error_deg`** — the instantaneous estimate-vs-truth heading error,
137.9° and 148.6° on the two audited runs. That is the criterion the gate
should carry; it is not yet wired in.
