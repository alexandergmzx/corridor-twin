# Stabilizing the instruments, and the world they measure

> **APPROVED 2026-08-12 14:53 CST.** Attended session. Binding; status updated
> after every unit; first thing re-read after any context compaction; the
> handback is its final section and is written even if the session fails early.
> Branch `corridor-stabilization-2026-08-12`. Nothing pushed.

## Live status

| Unit | State | Notes |
|---|---|---|
| U0-0 reap orphans | **DONE** 14:53 | 21 processes, all SIGTERM-clean. Inventory below |
| U0a rate basis | pending | |
| U0c unfreeze the lens | pending | |
| U0b session-scoped evidence + run manifest | pending | |
| U0d preflight + verified teardown | pending | |
| U0e arena/plan coherence (blocking) | pending | |
| U4a offline scale distribution (no GPU) | pending | |
| U6 paper debt | pending | |
| U1 corridor-aware scan path | pending | fleet delegation #2 |
| U2 the startup circle | pending | |
| U3 landmark containment | pending | re-derive, then fix |
| U4b/c/d odometry | pending | |
| U5 acceptance re-run | pending | |

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
