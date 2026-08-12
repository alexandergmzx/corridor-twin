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

## And truth is ALSO too fast

Nav2's `max_vel_theta` is 0.4 rad/s and the governor's ceiling is 1.5 (0.4 near
walls). **Truth peaks at 0.94–1.42 rad/s** — the robot physically rotates up to
3.5× faster than anything commands it to.

So there are two stacked problems, and they are separable:

1. **The twin over-rotates relative to command.** Consistent with wheel slip on
   a differential chassis during a turn, or with the composer's drive
   conversion — the effective wheel radius used to drive robot1 is 0.0458 m
   against a geometric 0.0245, a factor established in an earlier session.
2. **The fusion over-reports relative to truth**, on top of that, from an input
   (`/imu`, `/imu/data`) that measures 0.987–0.993 of truth.

Layer 2 is the one that destroys the map, and it is the one nothing in this
repository can reach.

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
