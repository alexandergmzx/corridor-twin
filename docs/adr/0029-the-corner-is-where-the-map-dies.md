# ADR 0029: The corner is where the map dies, and B carries a landmark that does not care

- Status: Accepted
- Date: 2026-08-12
- Source: robot1 corridor runs 2026-08-11 22:40 → 2026-08-12 02:15, artifacts
  under [`docs/evidence/robot-a-gate/`](../evidence/robot-a-gate/):
  `corner-probe-*.json`, `transit-audit-225511.json`, `yaw-sweep.json`,
  `lens-history.json`, `NOTES-landmark.md`.
- Relates to [ADR 0028](0028-goal-directed-navigation-on-a-live-map.md), which
  records that the navigation method works and that the arrival gate stays red.
  **This record says why it stays red**, and adopts the one measurement of B
  that is immune to the cause.

## Decision

**Two things, and the second exists because of the first.**

1. **The delivery cannot be gated on a map-frame number until the map is
   trustworthy.** The evaluation plane measures world-frame delivery error from
   simulator truth, and that is the number this project quotes.
2. **B carries a lidar-detectable landmark**, detected geometrically, whose
   measurement is taken in the laser frame and is therefore true whatever the
   map believes.

## What was measured, and what it rules out

The map diverges. Scored by the fleet's own instrument against an authored
"perfect SLAM" reference rendered from the scene's free-space oracle — a
reference which itself scores **0.000 m of duplicate wall**, so a run's reading
is its error in full:

| run | duplicate wall extent | limit |
|---|---|---|
| stationary, 0.42 m travelled | 0.300 m | 0.20 m |
| transit | 1.920 m | 0.20 m |
| transit | 0.800 m | 0.20 m |
| transit | 2.680 m | 0.20 m |

Seven candidate causes were tested and **each is ruled out by a measurement**,
not by argument:

| candidate | verdict | evidence |
|---|---|---|
| competing motion sources | **fixed, not the cause** | `sim_patrol` + a warm-up drive + Nav2 all commanded `/cmd_vel_raw`; removing them did not fix the map |
| odometry calibration | **acquitted** | EKF yaw within ±4% across 0.3–1.5 rad/s, both directions |
| rate-dependent gyro fault | **acquitted** | ratio spread 0.083 across that sweep |
| inverted yaw channel | **acquitted** | signed rotation agrees in sign everywhere |
| simulator slowdown | **acquitted** | real-time factor 1.001 |
| system load | **acquitted** | 0.936–1.094 with the full nav stack running and no goal sent |
| corridor shape faking the metric | **acquitted** | the authored corridor scores a 0.000 m floor |

## Where it dies

The corridor is clean. `slam_lens`, attached from the first transform, measured
over its window a scan-to-map fit of **0.752–1.000** (last 0.993, against a 0.5
"bad" line) and a SLAM-pose-vs-truth divergence of **0.000–0.022 m**. Two
centimetres.

The failure is at the far end. A reaches the delivery standoff — 0.244 m on the
best run — and the map degrades around and after that point.

## The anomaly, stated because it is not solved

`robot_localization` reports rotation its own input does not contain. On the
run measured stage by stage:

| tap | rotation | ratio vs truth |
|---|---|---|
| `/sim/ground_truth` | −155.57° | — |
| `/imu` (raw from the twin) | −153.52° | **0.987** |
| `/imu/data` (after madgwick) | −154.46° | **0.993** |
| `/odom` (after robot_localization) | **−3645.69°** | **23.4** |

The sensor is accurate to 1.3%. The orientation filter passes it through
unchanged. **The fusion output claims ten revolutions the robot did not make.**

Every structural explanation was checked and eliminated:

- The running EKF's parameters were dumped live: `odom0_config` enables index 6
  only (wheel **vx**), `imu0_config` index 11 only (IMU yaw-rate). Wheel yaw is
  structurally excluded at runtime, not merely in a file.
- `laser_odometry` holds only a `Buffer` and a `TransformListener`; it never
  broadcasts. There is exactly one publisher of `odom → base_footprint`.
- 23.4× sits inside the 6–26× band that the fleet's
  `docs/slam-research/near-wall-stability.md` measured for the wheel-yaw channel
  with the body blocked — but that study's mechanism requires the wheel channel
  to reach the filter, and here it cannot.

**This is unexplained and it is the top open item.** It is also fleet territory:
the fix, if it is a fix, lives in `robot_localization`'s configuration or in the
twin's IMU covariances (`orientation_covariance[0] = -1`, "not provided"), both
outside this repository.

## What was corrected here, and what it bought

`robot_radius` was 0.12, inherited from robot2 and carrying an UNVERIFIED flag.
robot1's chassis is 0.20 × 0.16 m, so its circumscribed radius is **0.128 m** —
Nav2 believed the robot was 8 mm smaller than it is, in the one place where
8 mm decides whether a path fits. `inflation_radius` was 0.16, justified against
a 0.6 m corner from an abandoned scale iteration; at the committed 0.42 factor
the corner is 1.26 m, so 0.30 leaves a 0.66 m uninflated band.

Measured effect, world frame from truth: closest approach **0.769 → 0.244 m**,
`walked_away` **4.21 → 0.85 m**.

## The landmark

A cylindrical post beside B, authored from config so it scales, its radius
reaching the detector through the manifest so the prop and the expectation
cannot drift into two literals. Detection is **geometric** — cluster, fit a
circle, require both a small residual and the authored radius — and confirmed
3-of-5.

**Intensity was considered and rejected**: sim-vs-real intensity fidelity is an
unowned contract question here, and a detector that works only in Isaac proves
nothing about the robot.

Measured live: first detection at **2.763 m**, confirmed in **3 frames**,
tracked to **0.309 m**, fitted radius **0.0665 m** against an authored 0.063
(+5.5%), mean residual 9.3 mm over 207 confirmed frames.

Tested against what would produce a phantom: a flat wall at the same range, a
convex corner at the same range, a cylinder of the wrong radius, a single lucky
frame, and frames that disagree on position. None confirm.

## Consequences

- **The arrival gate is unchanged** — Nav2 `SUCCEEDED` within 0.15 m map-frame —
  and the demonstration must pass with the detector disabled. The landmark is
  terminal-docking refinement, never the arrival mechanism.
- Nothing consumes the detection yet. Terminal docking (one refinement, ever) is
  specified and unbuilt.
- Parameter tuning of `slam_toolbox` is **not** the next move. The near-wall
  study falsified it on three bags at verdict level and says so explicitly; this
  record does not reopen it without new evidence.
- The speed policy stays `[to pin after first profile run]`. A speed derived
  from a diverged map is not a speed.
