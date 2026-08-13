# The corner is not where the yaw error is made

**2026-08-13, offline, no GPU.** Three session bags from the 2026-08-12
acceptance runs, read with `tools/corridor_yaw_stage_audit.py`.

```bash
source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
BAGS=~/Development/MicroROS/MicroROS-assets/bags
PYTHONNOUSERSITE=1 python tools/corridor_yaw_stage_audit.py \
  --bag $BAGS/20260812-184959-isaac-d67 --profile nominal_m6_n3 \
  --gate-json out/evidence/robot-a-gate/20260812-184944-robot1-nominal_m6_n3/gate.json \
  --out out/evidence/robot-a-gate/yaw-stage-nominal_m6_n3.json
```

Artifacts: [`yaw-stage-nominal_m6_n3.json`](yaw-stage-nominal_m6_n3.json),
[`yaw-stage-wide_corner_m6_n4_5.json`](yaw-stage-wide_corner_m6_n4_5.json),
[`yaw-stage-uniform_m6_n6.json`](yaw-stage-uniform_m6_n6.json).
No simulator, no GPU, no ROS graph — bags only.

## The hypothesis, and its answer

The transit gate failed on yaw scale at 1.166 (`nominal_m6_n3`) and 1.108
(`wide_corner_m6_n4_5`) while passing at 1.060 on `uniform_m6_n6`, the profile
with no tight corner. That pattern suggested the excess was made **at the corner
arc**, under angular acceleration.

**It is not.** Measured on the arc alone, with both channels taken from the same
samples over the same span:

| profile | corner sweep | truth on the arc | `/imu`÷truth | `/odom`÷truth |
|---|---|---|---|---|
| `nominal_m6_n3` | 97.1° | −94.34° | 1.0182 | **1.0430** |
| `wide_corner_m6_n4_5` | 93.6° | −81.27° | 0.9966 | **1.0186** |
| `uniform_m6_n6` | 90.0° | −67.94° | 1.0548 | **1.0993** |

The EKF tracks truth around the corner to within 2–10%. Nothing here produces a
1.17. **The corner-arc hypothesis is refuted**, and the two-labelling
cross-check (geometric ring around the manifest's own arc centre versus truth's
own yaw rate) agrees on 88–91% of samples, so the frames are registered and the
label is not the reason.

## What the number actually was

Chasing the discrepancy between this audit and the gate produced the real
finding, and it is a defect in the instrument.

For `wide_corner_m6_n4_5` the gate reported `truth_deg = −68.54`,
`estimated_deg = −75.95`, ratio **1.1081**. From the bag:

| quantity | whole bag | transit only |
|---|---|---|
| truth rotation | −74.03° | **−69.51°** |
| EKF rotation | **−75.95°** | −69.4° |

The gate's **numerator reproduces the whole-bag EKF rotation exactly**, to the
centidegree, while its **denominator matches the transit-only truth**. Those are
two different intervals of the same run.

`yaw_scale` summed each track over its own full extent. The two series arrive on
two independent subscriptions which neither start nor stop together, so the
window was whatever each happened to receive. Clipped to the span they share,
the same samples read **1.0013**.

`nominal_m6_n3` is the control: there the two windows happened to agree (gate
1.1663; bag 1.1725 whole-bag, 1.1616 transit-only), and **its red is real**.

## The fix, and why it is not tuning

`corridor_sim_gate.yaw_scale` now clips both tracks to their shared span before
summing, and reports the window, the sample counts, and how many samples each
side dropped outside it — so the window is part of the measurement rather than
an accident of it.

This is not mid-gate tuning to reach green. It does not move a threshold and it
does not touch the robot: `nominal`'s 1.17 survives it unchanged, and the
synthetic negative control in `test/test_yaw_scale_window.py` computes the OLD
quantity on the same samples and asserts it would have failed. A gate whose
value depends on which subscription happened to be listening is not measuring
the robot.

## The madgwick step, settled

`wide_corner`'s live tap reported `/imu` → `/imu/data` as **×1.0953**.
`imu_filter_madgwick` republishes `angular_velocity` unchanged, so an identity
step cannot scale anything, and `uniform`'s tap reports the two as
bit-identical — that is the control.

From the bag, `/imu` integrates to −75.34° over the whole bag against the live
tap's −68.69°, while the live `/imu/data` reads −75.07°. **The bag agrees with
`/imu/data`, not with the live `/imu`.** So the ×1.095 was never a filter
effect: the live `/imu` subscription was short, exactly as the truth
subscription was. Same class of defect, same run.

On `nominal_m6_n3` the bag's `/imu` integral is −77.5° against the live tap's
−77.52° — a match to two decimal places, which is what a healthy subscription
looks like and is why that profile's chain reads clean end to end.

## What is left, and where it lives

**`nominal_m6_n3` still reads 1.16–1.17 with the windows matched**, and that is
now the whole of the open yaw question. It is not the corner: the arc reads
1.043. It is distributed over the run, at a stage below `/imu` — the sensor is
within 2% of truth throughout — which puts it in `robot_localization`'s fusion.

That is **fleet territory** (`yahboomcar_config/param/ekf_sim_pnfix.yaml`), and
ADR 0029's named suspects are unchanged: the twin's IMU covariances
(`orientation_covariance[0] = -1`) and a 25 Hz IMU into a 10 Hz filter with
`two_d_mode: true`. No `slam_toolbox` or Nav2 parameter was touched, per
ADR 0029's standing law.

**Gate consequence, stated plainly:** two of the three profiles' yaw reds were
partly an artifact of the instrument and one was not. The re-runs are what say
which reds survive; this document does not claim them in advance.

## Scope and limits

- `/imu/data` is in **no bag**. The session recorder's topic list
  (`yahboomcar-ros2/tools/_session_record.py:55-57`) carries `/imu` and not the
  madgwick output, so the filter stage cannot be measured offline at all. Every
  statement about it here is inference from an identity passthrough plus the
  live tap, and it is labelled as such.
- The arc label is a 0.35 m corridor around the manifest's authored arc centre.
  A robot that cut the corner wide enough to leave that band would be labelled
  as being on a straight; the kinematic cross-check is what would show it, and
  it does not.
- `uniform_m6_n6` turns 429° over its transit against ~70° for the other two,
  because it drives a much longer effective path. Its ratios are the most
  statistically comfortable of the three and its arc reads the *highest*
  (1.0993) — another reason the corner is not the discriminator.
