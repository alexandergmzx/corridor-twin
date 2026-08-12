# Stabilizing the instruments, and the world they measure

> **APPROVED 2026-08-12 14:53 CST.** Attended session. Binding; status updated
> after every unit; first thing re-read after any context compaction; the
> handback is its final section and is written even if the session fails early.
> Branch `corridor-stabilization-2026-08-12`. Nothing pushed.

## Live status

| Unit | State | Notes |
|---|---|---|
| U0-0 reap orphans | **DONE** 14:53 | 21 processes, all SIGTERM-clean. Inventory below |
| U0a rate basis | **DONE** `0d9693e` | per-stream observed span; 5.35 → 11.50 Hz; floors named and printed; exactly one artifact was misreported |
| U0c unfreeze the lens | **DONE** `e24e596` | `lag_s` KeyError killed the sampler on tick 1; the lens had never worked |
| U0b session-scoped evidence | **DONE** `51e3e75` | run directory + `run.json`; result/rerun/**crash** with crash as the default; 74 files quarantined |
| U0d preflight + verified teardown | **DONE** `0bc6429` | `residents()` catches un-namespaced orphans; teardown polls; both tested against decoys |
| U0e arena/plan coherence | **DONE** `ff428e4` | robot-scale is the build default; arenas rebuilt; route margin 0.3 → 0.128; **corner screen did not scale**; cross-check before simctl |
| U4a offline scale distribution (no GPU) | pending | |
| U6 paper debt | pending | |
| U1 corridor-aware scan path | pending | fleet delegation #2 |
| U2 the startup circle | pending | |
| U3 landmark containment | pending | re-derive, then fix |
| U4b/c/d odometry | pending | |
| U5 acceptance re-run | pending | **blocked on the parked threshold below** |

Gates after U0: `bash tools/check_workspace.sh` green — ruff clean, **372 passed /
1 skipped**, colcon build 4 packages, colcon test **140 tests, 0 errors, 0
failures**.

## Morning decisions — parked, not taken

1. **The map-score threshold cannot be met by a perfect map of this scene.**
   The authored reference map — the oracle ADR 0029's whole divergence argument
   rests on — now scores **0.340 m** of duplicate wall against a **0.20 m**
   limit. Nothing about the map got worse: fixing the corner screen's scale
   lengthened a partition that runs parallel to the east wall 0.33 m away, and
   `duplicate wall extent` cannot tell that pair from one wall drawn twice at
   0.02 m resolution.

   | corner screen north margin | oracle floor | occlusion certificate |
   |---|---|---|
   | 0.40 m (the unscaled constant) | 0.060 m | **FAILS** — P visible on the whole approach |
   | 0.12 m (correctly scaled) | **0.340 m** | passes |

   0.20 m is an absolute number from the fleet's 4 × 4 m room; the corridor was
   scaled by 0.30 and the threshold was not, so it is now 3.33× stricter in
   relative terms than where it was measured. Either the limit is re-derived at
   the corridor's scale, or a run is scored as (reading − floor). **Both are
   threshold decisions, and a threshold is pinned by an ADR, not by the session
   that tripped over it.** Until then `map score ≤ 0.20` is not a criterion this
   scenario can meet and no run should be failed on it. The test pins the
   contradiction rather than choosing a value to go green.

   Consequence for U5: its stated acceptance needs this decision first.

2. **ADR 0029's duplicate-wall table describes the 12 m arena.** Its runs
   (0.740–2.680 m against a 0.000 m reference) were measured in the unscaled
   arena, which is internally consistent and no longer the scene that runs.

## Why this session exists

The 2026-08-12 Codex audit and ADR 0029's open anomaly. Four things were
verified first-hand while planning, and the fourth reorders everything.

### 1. The arena is not the scenario — **the finding of the session**

`out/arena_corridor_robot1_*.usd`, built **Aug 11 17:43**, are the **unscaled
12 m scene**. Read straight off the stage with `pxr`:

```
/World/Environment/Corridor   min [-2.0, -10.0, 0.0]   max [18.5, 3.5, 4.0]
/World/Actors/B               [16.568, -8.225, 0.0] .. [17.018, -7.775, 1.7]
/World/Actors/P               [17.15, 2.1, 0.0] .. [17.75, 2.7, 1.8]
```

There is **no landmark post in the stage at all**.

Meanwhile `corridor_nav_gate` plans from `out/corridor-small.manifest.json`
(factor 0.30): `corridor_length_m 3.6`, B at `[5.038, -2.4]`, landmark at
`[5.038, -3.2]` radius 0.12. `session.json` for `20260812-130918-isaac-d67`
confirms `arena=arena_corridor_robot1_nominal_m6_n3.usd`.

**Every corridor run since the rescale drove a 0.30-scale plan inside a
1.0-scale world.** Without any further hypothesis this accounts for:

- `world_frame_delivery.final_error_m 5.754`, `closest_approach_m 4.882` — A
  stopped where it was told, roughly 12 m short of where B actually stands;
- a "landmark" confirmed at 1.06 m in a stage containing no post, which then
  re-aimed the mission backwards (`nav-robot1-nominal_m6_n3.json`);
- the scan conditioner's geometry mismatch (a 20.5 × 13.5 m world against a
  4 × 4 m room model);
- ADR 0029's corner reasoning, which quotes a 0.42 factor that is not committed
  and an inflation radius that is no longer configured.

The composer defaults `--stage out/corridor.usda --manifest
out/corridor.manifest.json` (`build_corridor_arena.py:391-392`), and both of
those are the **unscaled** pair on disk (Aug 11 14:19, `corridor_length_m 12.0`).
The runner then defaults `MANIFEST` to the same unscaled file
(`corridor_profile_run.sh:127`) — today's runs only planned sanely because
`CORRIDOR_MANIFEST` was exported by hand, which is precisely how the arena and
the plan came apart.

**Decision (Alexander, 14:52):** rebuild, and **re-derive U3 and U4 from
scratch**. Every pre-rebuild number is measured in the wrong world and is not
carried forward as a premise.

### 2. The gate divides by the requested duration

`gate-robot1-nominal_m6_n3.json` records `seconds = 551.0` beside `observed_s =
256.11`, and reports `odom_laser_hz = 5.35` (2946/551). Against the observed
window: **11.50 Hz**. `ekf_hz` 4.71 → **10.13 Hz**. Two of that run's three
failures — "odom_laser too slow or absent", "EKF output too slow or absent" —
are artifacts of the instrument. Only the drift row was real, and it is now
re-premised by finding 1 anyway.

`--seconds` is not a measurement window: `corridor_profile_run.sh:456` derives
it from whatever the watchdog cap has left after bring-up.

### 3. The mandated lens is frozen from its first tick

`corridor_lens.py:401` reads `m['lag_s']`; `build_state()` stopped emitting that
key when the content-lag tile was removed. `KeyError('lag_s')` on iteration one
— in the run's own lens log. `latest['state']` is written once and never again,
`history` stays empty, and the `--dump` writes nothing because `history` is
falsy. Broken since the lens landed (`4e0f903`, the only commit that ever
touched the file). `test_the_content_lag_tile_is_gone` checked the tile was gone
from the page, not that nothing still read its key.

The lens rule has therefore been unenforceable for as long as the lens has
existed.

### 4. `/scan` is silent for ~21 s, then unfiltered for the rest of the run

`_scan_frame_relay.py` (fleet-side, spawned bare by `sim_runner.py:909`, no
params) validates each scan against `segments_room()` — a hardcoded 4.0 × 4.0 m
room, `yahboomcar_sim/arena.py:23-24`, called with no arguments. On a corridor
arena essentially every beam "sees through the wall", so it drops everything for
its 300-sample fail-open window (~21 s at 12–14 Hz) and then disables filtering
for the remainder. Measured in **56 of 62** `-isaac-d67` sessions on this box.

The phase-corrupted revolutions the relay exists to drop then reach
slam_toolbox, both costmaps and the governor for the whole transit.

## U0-0 — orphans reaped, 2026-08-12 14:53

21 processes, all of which died on SIGTERM; none needed SIGKILL. Recorded here
because the next run shares their domain and one of them is the node that
executes `Spin`.

| PID | started | what |
|---|---|---|
| 1660555 | Aug 12 13:11:30 | `behavior_server`, un-namespaced, corridor config, `cmd_vel:=cmd_vel_raw` — **the 13:16 run's own, still alive 1 h 24 m later** |
| 673292 | Aug 12 02:42:31 | `planner_server`, un-namespaced, corridor config — 11 h 53 m |
| 3904949, 3911144, 3911145 | Aug 11 16:41–16:44 | `/robot2` `behavior_server` ×2, `planner_server` |
| 9 pairs | Aug 11 14:37 – 16:42 | `ros2 launch fleet_bringup robot2_sim_bringup_launch.py` + its `scan_to_scan_filter_chain` |

Two consequences, both now units: teardown does not verify (U0d), and a stale
un-namespaced `behavior_server` sharing node and action names with a new run is
the first candidate for the startup circle (U2).

## Standing corrections carried into this session

- `pn-fix` (wheel vx at index 6, IMU yaw rate at index 11) is the **designed**
  EKF configuration, not a defect. No wheel yaw is to be added.
- No `slam_toolbox` parameter tuning (ADR 0029's law). The named defects
  upstream of SLAM are fixed and re-measured first.
- No search behaviour. The startup circle is diagnosed as unspecified motion and
  U2 proves or disproves that with a log, not an argument.
- Profile gate runs execute with **docking disabled**; docking runs are separate.

## Handback

*(written at session end; see the final section)*
