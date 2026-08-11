# robot1 evidence night — 2026-08-11

> **APPROVED 2026-08-11 17:33 CST.** All eight decision points (C1–C8) ratified
> at their recommended defaults; no overrides given. Budget 5 h — ends 22:33,
> no new unit starts after 22:03. Operator away: park, do not decide.
>
> This file is binding. Status is updated after every unit. It is the first
> thing re-read after any context compaction. The handback is its final section
> and is written even if the session fails early.

## Live status

| Unit | State | Notes |
|---|---|---|
| X0 plan + branch + Isaac lock | IN PROGRESS | started 17:33 |
| X1 end-wall correlation + study draft | pending | |
| X2 composer robot1 mode | pending | |
| X3 robot1 governed Nav2 | pending | 90 m hard cap |
| X4 gate parameterization | pending | |
| X6 three profile runs | pending | X5 folded in |
| X7 P-camera candidates | stretch | |
| X8 study contrast | stretch | |

## Context

ADR 0027 closed selection: **robot A = robot1**, because robot2's encoder-less
odometry published nothing until ~5 m into every corridor profile. Tonight
produces **v2 evidence for robot1 against the pinned bar** — not a second
selection. Selection is closed and stays closed.

Robot2's three-profile runs are already committed and become the degeneracy
study's data. Tonight adds robot1's contrast and the first P-camera candidates.

**The night's central hypothesis, from inventory:** robot1 should not reproduce
robot2's failure at all, because its EKF does not consume the scan matcher.
`ekf_sim_pnfix.yaml:138-146` removed the laser-pose input ("measured HARMFUL"),
so `/odom` is encoders (`odom0: /odom_raw`, vx only, `:117-122`) + IMU yaw-rate
(`imu0: /imu/data`, `:150-155`). Meanwhile `simctl:665` still launches
`yahboomcar_localization laser_odometry`, so `/odom_laser` is published and
measurable **beside** a healthy EKF. That is the cleanest possible contrast:
same corridor, same instrument, matcher degeneracy visible but no longer
load-bearing.

---

## Preflight — run read-only during planning

| Item | Result |
|---|---|
| Branch / HEAD | `feat/v2-corrections` @ `3055455` |
| Tree | **`M CLAUDE.md`** (Alexander's own in-progress edit) + untracked retired pack file |
| Fleet commits intact | `7ae79fb`, `40aa744`, `99b0310`, `aaeeabd` all present |
| Three robot2 arenas | present |
| Hardened pose gate on all three | **PASS** — yaw err ≤1.4e-7°, pos err 0.0 (re-verified via `pxr`, no GPU) |
| Contract + gate tests | 45 passed |
| Isaac / GPU | clear; 650 MiB; **`/tmp/fleet-isaac.lock` absent (free)** |
| Disk / RAM | 124 G free (87% used); 36 G RAM available |
| `docs/session-plans/` | does not exist — created in X0 |

---

## Inventory (file:line)

### robot1's own contract — robot2's numbers do not transfer
- `yahboomcar-ros2/tools/check_isaac_contract.py:51`
  `WANT_HZ = {'scan': 12.0, 'odom_raw': 11.0, 'imu': 25.0, 'battery': 1.0}`
- Tolerance is inline, **no `TOL` constant**: `:97` `max(0.3, want * 0.10)` (±10%, floor ±0.3 Hz)
- Topics are **root**, no prefix: `:71-74` `/scan`, `/odom_raw`, `/imu`, `/battery`; drives via `/cmd_vel` `:79`
- CLI: `:54-58` `--seconds` (30), `--speed` (0.12), `--turn` (0.3), `--domain` (66)
- Hardware-measured: `architecture.md:447` scan 12.40→12.39, imu 24.30→24.27, odom_raw 11.06→11.01;
  `yahboomcar-ros2/docs/bench-report-20260808.md:22` scan 12.18 / imu 23.85 / odom_raw 10.93 — **imu runs ~24 against a declared 25**, inside the ±10% band but consistently low.

### robot1 lidar = MS200, *not* C1
- `yahboomcar-ros2/tools/build_arena.py:51-56` — beams 360, hz 12, range **0.12–8.0 m**, xyz `(-0.0046, 0, 0.094)`
- `author_lidar` `:59-61`; every sensor kwarg defaults `None` and falls back to those constants `:65-69`
- **Calling it with no kwargs is robot1's contract.** Default prim name `laser_frame_lidar` `:61`
- `minDistBetweenEchosM` hardcoded 0.05 `:128` (not parameterized)

### robot1 namespace / frames / odom
- **Root namespace, unprefixed** — `architecture.md:46-48` (A2.1), `:49-51` (A2.2: "robot1's root topics are consumed directly by the sim gates")
- Frames: `odom`, `base_footprint`, `laser_frame`, `imu_frame` (`slam_preflight.py:66-68`; `ekf_sim_pnfix.yaml:97-99`)
- `/odom_raw` = firmware/twin encoders 11 Hz; `/odom` = EKF output (`bringup_corrected_launch.py:82` remaps `odometry/filtered`→`/odom`), 10 Hz (`ekf_sim_pnfix.yaml:86`)
- Sim stamps `odom` correctly (`sim_runner.py:588`); the hardware `odom_frame` landmine (`slam_preflight.py:32-36`) **does not apply tonight** — sim only

### Governed Nav2 for robot1 — **does not exist; buildable without leaving this repo**
- No robot1 nav launch and no `nav2_robot1.yaml` anywhere (exhaustive greps; `fleet_bringup/launch/` has 6 files, `param/` has 3, all robot2/robot3)
- Vendor `navigation_dwb_launch.py` has **no cmd_vel remapping at all**; `nav2_smoke.py:162-164` says so explicitly
- **The enabler:** robot1's governor hardcodes its topics — `yahboomcar_safety/.../cmd_vel_governor.py:82,84,86` = `/scan`, `/cmd_vel_raw`, `/cmd_vel`. At **root namespace** the robot2 pattern `remappings=[('cmd_vel','cmd_vel_raw')]` with no `namespace=` resolves to `/cmd_vel_raw` — the governor's input. So a root-namespace robot1 nav launch is governed by construction.
- Template: `robot2_nav_sim_launch.py:37-38` `governed = [('cmd_vel','cmd_vel_raw')]`, applied to controller `:43` and behavior `:53`; lifecycle `bond_timeout: 0.0` `:60-69`
- Substitutions needed vs `nav2_robot2.yaml`: `odom_topic` `:22,:116` → `/odom`; scan `:142,:181` `scan_filtered` → `/scan` (**robot1 has no scan_filtered producer**); frames `:102-104,:115,:126-127,:162-163` → unprefixed; `map_topic` `:175` → `/map`
- `simctl:436-437` — **simctl never starts Nav2 for either robot**, so the wrapper launches it, exactly as it already does for robot2

### simctl robot1 Isaac path
- `--robot robot1` is the **default** (`simctl:1209-1210`); robot1 is a 7-step path (`:618-772`) vs robot2's 3
- `[3/7]` `:665` launches `yahboomcar_localization laser_odometry` → **`/odom_laser` exists on robot1**
- `[4/7]` `:666-669` SLAM via `yahboomcar_config slam_launch.py`, bond disabled (`slam_launch.py:65-76`)
- `[5/7]` Isaac via `sim_runner.py`, waits 420 s for `/scan`+`/odom_raw` (`:704-711`)
- EKF default is **pn-fix** (`simctl:1224-1226`)
- `YAHBOOM_ARENA_USD` (`sim_runner.py:41-46`): absolute → verbatim; relative → joined under `USD_DIR`; empty → `arena.usd`. Env inheritance reaches the child (simctl only sets it for `--fun`) — same pattern my robot2 wrapper already relies on.

### robot1 twin asset + physics
- `build_arena.py:38` `ROBOT_USD = USD_DIR/micro4/micro4.usd`, referenced at `/World/Robot` `:333-334` — exact parallel to my composer's rasptank path. Asset verified present.
- `USD_DIR` resolves to `src/MicroROS/yahboomcar_ws/src/yahboomcar_twin/usd` (`_layout.py:60-63`)
- **Wheel colliders are measured, robot1-specific, and inline in `main()`** — `build_arena.py:396-420`: de-instance, disable mesh colliders, analytic cylinders (r 0.024, h 0.0215, axis Y, purpose `guide`), per-wheel friction with **rear deliberately low** (`--friction` 0.6 `:182`, `--rear-wheel-friction` 0.1 `:189`) for the skid-steer model
- Spawn: `WHEEL_DROP 0.045 + SPAWN_CLEARANCE 0.01` = 0.055 (`:43-44,:335`) — coincidentally identical to rasptank's, but derived per-robot rather than assumed

### X1 inputs confirmed present
- Acquisition stations in committed JSON: nominal 5.83, wide_corner 5.41, uniform 4.77 m
- Covariance traces: 466 / 476 / 599 rows
- Per-profile wall polygons in the manifest (`CornerBuilding` west face x = 11.50 on all three)
- **Computed during planning:** A→end-wall is 11.50 m on every profile. C1 (12 m) sees it from station 0; **MS200 (8 m) only from station 3.50 m.** Range is therefore a *confound* in the end-wall hypothesis, not a clean variable — X1 must say so.

---

## A. Unit queue

Wall-clock budget **5 h**. `date` between units. **No new unit starts after 4 h 30 m.**
X5 is folded into X6's first run (the contract precondition *is* the smoke) to buy back time.
**X7/X8 are explicitly stretch** — the queue realistically ends at X6.

| # | Unit | Box | Skip-edge |
|---|---|---|---|
| X0 | Write plan to `docs/session-plans/2026-08-11-robot1-evidence.md`; create session branch; implement the Isaac lock helper (**F12: lock is doc-only, zero code today**) | 15 m | none — mandatory |
| X1 | End-wall correlation from existing robot2 JSONs + first draft `docs/degeneracy-study.md` | 40 m | none — no Isaac, guaranteed win, always first |
| X2 | Composer `--robot robot1` mode; recompose all three profiles | 50 m | if micro4 fails to compose → **park X2/X5/X6, keep X1/X7**, report in bold |
| X3 | robot1 governed Nav2 launch + param file, in this repo | **90 m HARD CAP** | on cap: X6 runs **drive-and-map only**, stated in bold in every artifact and the handback. **Never Nav2 around the governor as a fallback.** |
| X4 | Parameterize gate tools for robot1 (no third fork) | 35 m | none |
| X6 | Three profile runs (X5 folded in as the precondition) | 75 m | per-profile: infra failure → rerun ×2 max, then park that profile and continue |
| X7 | *(stretch)* P-camera candidates: 2–3 poses, LOS/distance/incidence table, one 640×360 frame each | 30 m | drop if past 4 h 30 m |
| X8 | *(stretch)* Fold robot1 numbers into the study as the encoder contrast | 20 m | drop if past 4 h 30 m; X1's draft already stands alone |

### Unit detail

**X0 — Isaac lock.** CLAUDE.md now mandates `/tmp/fleet-isaac.lock`, and F12 records it as
doc-only with zero code. Implement `tools/isaac_lock.sh` (acquire writes PID + session name;
a lock whose PID is dead is stale and removable; release on every exit path) and wire it into
`corridor_profile_run.sh` and the composer wrapper. Poll 5 min × max 45 min, then park
GPU units and continue with non-GPU work.

**X1 — the guaranteed win.** Correlate acquisition station against per-profile geometry
(end-wall distance, local width, taper rate). State plainly that **n=3 cannot establish a
mechanism** — ADR 0027 already refuses that claim and the study must not quietly acquire it.
Include the two instrument defects as a methods note: the gaps-between-messages blindness
(a 5.9 m silence scoring "1") and the same-instant truncation fix.

**X2 — composer robot1 mode.** `--robot {rasptank,robot1}`; robot1 selects
`USD_DIR/micro4/micro4.usd`, calls `author_lidar` with **no kwargs** (MS200), and replicates
the wheel-collider block. Keep the spawn-clearance gate (≥0.5 m) and the hardened pose gate
including the ±180° seam. Re-run all three profiles.

**X3 — two new files in this repo**, launched by absolute path (`ros2 launch <path>` accepts
one), so nothing is written outside corridor-twin and no colcon packaging is needed.

**X4 — parameterize, don't fork.** The surface is small and already isolated:
`corridor_sim_gate.py:51` `NS`, `:186-190` topics, `:278-280,:328-330` frames;
`corridor_nav_gate.py:52,91,94,109`. Add `--namespace`, `--odom-source`, `--ekf-topic`,
`--base-frame`, `--odom-frame`, defaulting to today's robot2 values so committed artifacts
stay reproducible. **`GOAL_TOLERANCE_M = 0.15` stays one constant, printed and enforced.**
Record `/odom_laser` *and* `/odom` for robot1 — that pair is the study's contrast.

---

## B. The robot1 evidence bar

Carried unchanged from ADR 0022 where the quantity still means the same thing:

1. **Nav2 SUCCEEDED and map-frame goal error ≤ 0.15 m** — one constant, printed and enforced.
2. **Longitudinal midpoint drift ≤ 5%** of distance travelled, same-instant truncation.
3. **Covariance vs station recorded** for the study contrast — for robot1, from *both*
   `/odom_laser` (matcher) and `/odom` (EKF).

### Replacement for the matcher-withholding criterion

The robot2 criterion measured the matcher because on robot2 **the matcher *was* the
odometry**. On robot1 it is not an EKF input at all (`ekf_sim_pnfix.yaml:138-146`), so
withholding cannot starve localization and the criterion does not transfer.

**Recommended replacement — EKF output continuity, derived by ADR 0022's own logic**
(blind travel < goal tolerance):

> Maximum publication gap on `/odom` ≤ **0.4 s** (≤ **4 consecutive** missed updates at the
> EKF's 10 Hz, `ekf_sim_pnfix.yaml:86`), **measured from drive start** so initial silence
> counts.

Derivation: 0.35 m/s governor cap (`governor.py:41-60`) × 0.4 s = 0.14 m blind travel,
under the 0.15 m tolerance. The governor cap is used rather than the gate's drive speed
because it is the true worst case.

**Stated limit:** this is a *liveness* check. Encoder odometry cannot go silent, so it will
almost certainly pass — and that is the point of recording it. The criterion that actually
detects corridor degeneracy on robot1 is **drift (#2), unchanged**: encoder odometry cannot
go quiet, but it can go wrong. Matcher withholding is still *recorded* on robot1 as study
data, and explicitly **not gated**.

---

## C. Decision points — recommended defaults to ratify or override

| # | Decision | **Recommended default** |
|---|---|---|
| C1 | Withholding-replacement criterion and value | **`/odom` max gap ≤ 0.4 s (≤4 consecutive), from drive start; drift stays the real degeneracy gate; matcher withholding recorded, not gated** |
| C2 | X3 assembly approach (no ready launch exists) | **Mirror `robot2_nav_sim_launch.py` at root namespace into this repo's `tools/`, launched by absolute path. Governed by construction via `[('cmd_vel','cmd_vel_raw')]`. No writes to yahboom.** |
| C3 | Nav2 `max_vel_x` for robot1 | **0.22 m/s**, matching robot2's so the two robots' runs are comparable, and safely under robot1's 0.35 governor cap |
| C4 | Nav2 `robot_radius` for robot1 | **0.12 m inherited from `nav2_robot2.yaml:132`, flagged in the artifact as unverified for robot1's chassis.** Measuring it is a morning item — it matters most where the corridor narrows |
| C5 | Wheel-collider physics (`build_arena.py:396-420`) | **Replicate in this repo with a provenance header citing the source lines; park the OI-19 extraction to the morning list.** OI-19 (`architecture.md:452`) already reads "sim geometry helpers have three" — this is the fourth, and extracting it means writing to yahboom, which is not delegated |
| C6 | robot1 contract precondition | **`check_isaac_contract.py` with its own `WANT_HZ` and `--domain <scratch>`, unmodified.** Its declared imu 25 Hz vs measured ~24 is recorded, never re-declared |
| C7 | Alexander's uncommitted `CLAUDE.md` edit | **Leave it untouched and uncommitted; document the dirty tree in the handback.** Committing another author's in-progress edit unattended is not mine to do |
| C8 | Session branch name | **`robot1-evidence-2026-08-11`**, per the append-only new-branch-per-session rule |

---

## D. Delegated vs NOT delegated

**Delegated tonight:** everything inside this repo — composer robot1 mode, the robot1 nav
launch and param file, gate parameterization, the Isaac lock helper, all runs, all JSON
artifacts, `docs/degeneracy-study.md`, evidence promotion, local commits on the session
branch.

**NOT delegated — parks to the handback's morning list, no exceptions:**
- gate-parameter tuning to reach green
- new or edited ADRs
- `git push` (does not exist tonight)
- any write outside this repo (incl. the OI-19 extraction and the fleet ledger)
- choosing P's camera pose
- re-declaring any contract number (incl. robot1's imu 25 vs measured ~24)

---

## Execution contract

On approval, **first act**: write this plan verbatim — with ratified decisions — to
`docs/session-plans/2026-08-11-robot1-evidence.md`. Binding; status updated after every
unit; re-read as the first action after any context compaction; the handback is its final
section, written even if the session fails early.

CLAUDE.md's unattended hard rules bind in full: Isaac single-occupancy lock, domain
deny-list **20/42/43/44/66/68** (scratch 67/69), local-only git, append-only history,
`colcon build --packages-select` + touched tests green before every commit, bounded retries
(×2), park-don't-decide, `date` between units, no new unit after 4 h 30 m.

## Verification

- Per unit: `ruff check` + `pytest` on touched tests before any commit; full
  `bash tools/check_workspace.sh` green before the final handback commit.
- X2: arena report shows MS200 params read back, spawn clearance ≥0.5 m, pose gate pass on all three.
- X3: launch inspected for the governed remapping on **both** controller and behavior servers before any run.
- X6: per-profile JSON with contract precondition, drive-and-map gate, and nav result; every
  non-green run classified explicitly as **infrastructure rerun** or **committed red result**.
