# The fusion anomaly, characterised

**2026-08-12 03:35 CST.** Offline analysis of four session bags, robot1,
`nominal_m6_n3`. No GPU; reproduce with `tools/corridor_transit_audit.py` and
the snippet recorded in this session's handback.

## It is not a reset. It is continuous over-integration.

| run | yaw scale | EKF peak yaw rate | truth peak | EKF steps > 0.3 rad | EKF p95 rate |
|---|---|---|---|---|---|
| 00:28 | **23.434** | 3.15 rad/s | 1.25 | 1 | 0.80 |
| 03:02 | 0.594 | 4.27 | 1.42 | 2 | 0.92 |
| 03:12 | −1.518 | 2.87 | 1.35 | 0 | 0.83 |
| 03:20 | 0.140 | 1.74 | 0.94 | 0 | 0.48 |

At 10 Hz a robot capped at 0.4 rad/s cannot move more than ~0.06 rad between
consecutive `/odom` samples. **Zero to two samples per run exceed 0.3 rad**, so
the estimate is not jumping and the filter is not relocalising or resetting. The
error accumulates smoothly, at rates 2–3× truth's own.

## The twin does NOT over-rotate — a correction

Truth peaks at 0.94–1.42 rad/s against Nav2's 0.4 rad/s cap, and this document
first read that as the twin physically over-rotating by up to 3.5×. **That
reading was wrong.** Peak-versus-cap conflates a transient with a scale factor.

Measured properly, over samples with a steady yaw command (>0.15 rad/s) and
real rotation (>0.05 rad/s), so a curved path's geometry cannot contaminate it:

| bag | n | truth/commanded yaw, median | mean | max |
|---|---|---|---|---|
| `20260812-002857` | 159 | **0.506** | 0.653 | 1.87 |
| `20260812-031251` | 184 | **0.565** | 0.675 | 1.87 |
| `20260812-032055` | 49 | **0.724** | 0.810 | 1.74 |

The twin turns at roughly **half to three-quarters** of the rate it is told to,
which is under-rotation — the ordinary signature of wheel slip and of the
governor clamping near walls. The 1.42 rad/s peaks are transients.

So the drive conversion is **not** implicated, and the composer's effective
wheel radius (0.0458 m against a geometric 0.0245) is not a suspect on this
evidence. **There is one problem, not two**: the fusion over-reports relative to
truth, from an input measuring 0.987–0.993 of truth, and nothing in this
repository can reach it.

## What is already excluded

Dumped from the **running** node, not read from a file:
`odom0_config` enables index 6 only (wheel vx); `imu0_config` index 11 only
(IMU yaw-rate). Wheel yaw cannot enter. `laser_odometry` holds only a `Buffer`
and a `TransformListener` and never broadcasts, so exactly one node publishes
`odom → base_footprint`.

Loop closure is falsified separately (`NOTES-loop-closure.md`).

## The morning's starting point

Two candidates remain, both outside this repo:

- the twin's IMU covariances — `orientation_covariance[0] = -1` ("not
  provided"), and whether `angular_velocity_covariance` is populated at all;
- `robot_localization`'s handling of a 25 Hz IMU at a 10 Hz filter rate with
  `two_d_mode: true`.

Neither is guesswork this repository can settle, and neither should be changed
without the same-bag A/B protocol the fleet's near-wall study established.
