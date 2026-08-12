# Making A actually find B — autonomy method

> **APPROVED 2026-08-11 21:44 CST.** Budget 5 h — ends 02:44, no new unit
> starts after 02:14. Binding; status updated after every unit; first thing
> re-read after any context compaction; the handback is its final section and
> is written even if the session fails early.

## Live status

| Unit | State | Notes |
|---|---|---|
| U0 session setup | IN PROGRESS | started 21:44 |
| U1 standoff goal + scale fix | pending | |
| U2 TF staleness | pending | |
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
