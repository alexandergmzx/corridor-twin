# Corridor arena composition and smoke — v2 plan T1.2 / T1.4

First live run of the RaspTank fleet twin inside a corridor arena, and the
first corridor Isaac session on a non-default ROS domain. Both are v2-plan
Day-1 tasks; neither says anything about the enforcement pipeline, which is
still the v1 one.

## Environment

| Field | Value |
|---|---|
| Date | 2026-08-11 |
| Isaac Sim | 5.1.0.0 (`~/isaac/env_isaaclab`, Python 3.11) |
| IsaacLab | 2.3.2 |
| GPU | NVIDIA GeForce RTX 5070 Ti, driver 580.173.02, 16303 MiB |
| ROS | Jazzy, fleet `ground_station/install` |
| Domain | 67 (scratch; 20/42/43/44/66/68/70 refused structurally) |
| Arena | `out/arena_corridor_nominal_m6_n3.usd` |

## Commands

```bash
# T1.2 — one arena per profile (repeat for the other two profiles)
~/isaac/env_isaaclab/bin/python tools/build_corridor_arena.py --profile nominal_m6_n3

# T1.4 — smoke
bash tools/corridor_smoke.sh --domain 67
```

Run the composer from a shell with **no system ROS sourced**: `pxr` there is
Isaac's bundled USD (CLAUDE.md environment discipline).

## T1.2 — arena composition: PASS on all three profiles

`arena-nominal_m6_n3-report.txt` is the nominal profile's report, promoted as
representative; the other two are byte-similar apart from the profile-specific
values below.

| Profile | Robot yaw | Forward-sign gate (along heading) | Result |
|---|---|---|---|
| `nominal_m6_n3` | +7.13° | +0.797 m | PASS |
| `wide_corner_m6_n4_5` | +3.58° | +0.797 m | PASS |
| `uniform_m6_n6` | +0.00° | +0.798 m | PASS |

The yaw tracks the taper, and the untapered profile's heading is exactly +x —
a coherence check the composer gets for free by reading the pose out of the
manifest rather than hardcoding it.

The lidar authors **15** parameters, not 14. The v2 plan's Day-1 brief says 14,
which is what `rasptank_twin/usd/arena_report.txt` still records; that report
predates `minDistBetweenEchosM`, added to `author_lidar` on 2026-08-09 after
the near-wall measurement. 15 is the current, correct count and the extra
parameter is exactly the one the brief separately asks for at 0.05.

Independently read back off the prim, beyond `author_lidar`'s own check:

```
scanRateBaseHz = 10          reportRateBaseHz = 5000
nearRangeM     = 0.05        farRangeM        = 12.0
minDistBetweenEchosM = 0.05  numberOfChannels = 1   numberOfEmitters = 1
emitterState:s001:elevationDeg = [0]   emitterState:s001:channelId = [1]
```

The last two are the traps: a nonzero elevation makes `IsaacComputeRTXLidarFlatScan`
refuse to run, and `channelId` 0 leaves the plugin on its previous 3D profile.
Both failure modes look like a healthy sensor publishing nothing.

## T1.4 — smoke: PASS

`contract-nominal-domain67.json` is the checker's output, unedited.

| Topic | Measured | Declared | Verdict |
|---|---|---|---|
| `/robot2/scan` | 10.04 Hz | 10 | pass |
| `/robot2/scan_filtered` | 10.11 Hz | 10 | pass |
| `/robot2/imu` | **52.35 Hz** | 60 | pass, but 12.8% low |
| `/robot2/odom_laser` | 10.30 Hz | — | alive |

Frames all carry the `robot2/` prefix; `scan` and `imu` are BEST_EFFORT,
`scan_filtered` RELIABLE (the 2026-08-08 contract amendment); no robot-side
odometry topic, per fleet D-05.

**The IMU figure is worth keeping in view.** The gate is `--imu-hz 60` with the
checker's ±25% tolerance, so 52.35 Hz passes inside a 45–75 Hz band — a real
pass, not a marginal one, but it sits below the declared rate rather than
scattered around it. It was not tuned to pass, and 60 was used rather than the
checker's 100 Hz default because the Isaac twin does not reach 100.

`/robot2/scan_filtered` and `/robot2/odom_laser` both alive matters more than it
looks: without the conditioner and the ground-station matcher in the loop, SLAM
silently never builds a map and the run appears healthy throughout.

Session stopped and verified dead; 12 processes terminated, 0 remaining, 71
stale DDS shared-memory segments cleared.

## What this does not show

- Nothing about enforcement, P's camera, or the domain crossing. That is T2.
- No map quality claim. SLAM ran, but a smoke is not a mapping gate.
- No statement about the other two profiles under simctl — only `nominal_m6_n3`
  was smoked. T3.3 runs the per-profile gates.
- The composed arena references the RaspTank asset by absolute path, so it is
  not portable off this machine as-is.
