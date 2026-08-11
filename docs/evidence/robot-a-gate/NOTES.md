# Robot-A corridor gate and degeneracy study — v2 plan T3.3

The three-profile measurement ADR 0027 is decided on.

**Both gated profiles FAILED.** The cause is the same on all three profiles and
is not a flake: the scan matcher publishes nothing for the first ~5 m of
corridor travel, so localization never tracks, and Nav2 aborts.

## Environment

| Field | Value |
|---|---|
| Date | 2026-08-11 |
| Isaac Sim | 5.1.0.0; RTX 5070 Ti, driver 580.173.02 |
| ROS | Jazzy, fleet `ground_station/install` |
| Domain | 67 (scratch), fresh Isaac session per profile, verified dead between |
| Stack | `robot2_sim_bringup_launch.py` via simctl, then `robot2_nav_sim_launch.py` |
| Drive | straight passes, 0.15 m/s forward with 1.5 s settles, 90 s |

```bash
bash tools/corridor_profile_run.sh --profile nominal_m6_n3       --gated --domain 67
bash tools/corridor_profile_run.sh --profile wide_corner_m6_n4_5 --gated --domain 67
bash tools/corridor_profile_run.sh --profile uniform_m6_n6               --domain 67
```

## Results

| Profile | Gated | Moved | First `odom_laser` at | Max consecutive withheld | Midpoint drift | Nav2 | Verdict |
|---|---|---|---|---|---|---|---|
| `nominal_m6_n3` | yes | 8.17 m | **5.83 m** | **506** (limit 5) | **1.000** (limit 0.05) | ABORTED, 18.37 m error | **FAIL** |
| `wide_corner_m6_n4_5` | yes | 8.30 m | **5.41 m** | **459** | **1.000** | ABORTED, 17.59 m error | **FAIL** |
| `uniform_m6_n6` | no (reported) | 8.38 m | **4.77 m** | **391** | **1.000** | not measured | finding |

The robot moved ~8.3 m on every profile and the map built (1361–1682 occupied
cells), so the twin, the governor and SLAM all work. What does not work is
odometry.

`uniform_m6_n6`'s Nav2 leg is **NOT MEASURED**, not failed: the action server
had not come up within the runner's 25 s wait. Its drive-and-map leg — the
degeneracy trace, which is what this profile exists for — completed normally.

## The finding: the matcher does not acquire for the first ~5 m

`odom_laser` produces nothing at all until the robot has already travelled
4.8–5.8 m. Until then the EKF has no laser input, its pose stays at the origin,
and estimated travel at the midpoint (~4.1 m) is **0.0 m** — hence a drift
fraction of exactly 1.000 on all three profiles.

That number was invisible at first. The withholding metric measured only gaps
*between* messages, so a run where the matcher had produced nothing whatsoever
for 5.9 m scored "1 consecutive withheld update". Counting the initial silence
turns the same run into 506. **A gate that measured only the gaps it could see
would have reported this stack as healthy.**

Once acquisition happens the matcher behaves: it publishes at ~5 Hz and the
covariance is plausible.

## The degeneracy study: covariance against station

The primary artifact is the full `covariance_trace_station_xx_yy_yawyaw` array
in each `gate-*.json` (466–599 rows per profile). At the midpoint:

| Profile | `cov_xx` (along) | `cov_yy` (across) | Anisotropy |
|---|---|---|---|
| `nominal_m6_n3` | 4.58e-04 | 2.61e-05 | **17.6×** |
| `wide_corner_m6_n4_5` | 4.73e-04 | 2.52e-05 | **18.8×** |
| `uniform_m6_n6` | 4.63e-04 | 2.51e-05 | **18.4×** |

The along-corridor axis is ~18× less constrained than the cross-corridor axis,
on every profile. That is the textbook corridor degeneracy, measured rather than
asserted, and it is remarkably stable across a 6→3 m taper, a 6→4.5 m taper and
a uniform 6 m corridor.

Acquisition station orders monotonically with taper — 5.83 m (most tapered),
5.41 m, 4.77 m (untapered) — and the withheld count orders the same way. Three
points is not enough to claim a mechanism, and no causal explanation is offered
here.

## Twin sensor rates are variable and sometimes miss the contract

The `--imu-hz 60` precondition failed on two of five attempted sessions:

| Attempt | IMU | `scan` | Precondition |
|---|---|---|---|
| nominal, first | 41.01 Hz | — | **FAIL** (band 45–75) |
| nominal, retry | pass | pass | pass |
| wide_corner | 55.34 Hz | pass | pass |
| uniform, first | 37.46 Hz | 7.06 Hz | **FAIL** (both) |
| uniform, retry | pass | pass | pass |

Both failures were treated as **infrastructure and rerun**, never as results —
the runner exits 3 for these and 1 for a red gate. But the variance is a real
finding: the corridor arenas load the twin closer to its limits than the
RaspTank's own 4×4 room does, and nothing was tuned to make a run pass.

## What this does not show

- **Not a chassis verdict.** The scan matcher is the fleet's in-house one, tuned
  and measured in a 4×4 m room. Whether the corridor behaviour is a platform
  limit or a tuning limit is not resolved by this measurement, and ADR 0027 says
  so explicitly.
- Nav2 aborted with the goal ~18 m away in largely unmapped space. The abort is
  consistent with broken localization, but this run does not separate "aborted
  because localization failed" from "aborted because the goal was unreachable in
  the map built so far".
- No camera, enforcement, or detector claim.
- `robot_radius` 0.12 was left at its pinned value against a measured
  circumscribed 0.128 m; nothing here was re-tuned mid-gate.
