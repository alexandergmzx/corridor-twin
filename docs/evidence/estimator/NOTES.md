# Speed from P's camera: correction 3's first measured result

**Date** 2026-08-14. **ADR 0024's pipeline, measured end to end for the first
time.** The 2026-08-04 interview asked for active AI/ML use; ADR 0024 answered
with a learned detector on P's own roadside camera. What was missing was the
number. This is the number, and the two defects that measuring it uncovered.

## Headline

| | |
|---|---|
| Gate coverage | **5 of 5**, from pixels alone |
| Violations | **exactly one**, confirmed at gate 3.0, three compliant gates before it |
| Speed error, as measured | **−10.59%** mean over the five gates |
| ...of which the recording path's timing defect | **−11.20%**, independently measured |
| **...leaving the estimator's own contribution** | **+0.62%** |
| Station accuracy, projection only, 412 labelled frames | bias **+0.023 m**, sd 0.056 m |
| Detector gross-failure rate | **19.5%** of frames land >0.5 m out, and **confidence cannot tell you which** |

The attribution is **reported, never applied**. Nothing in the table is
corrected by the lag: a correction derived from the run it corrects is a
fitted parameter wearing a mechanism's clothes.

## Commands

```bash
# 1. where P's camera is, out of the stage that was rendered (venv, pxr)
.venv/bin/python tools/export_camera_pose.py \
  --stage out/arena_corridor_robot1_nominal_m6_n3.usd --out out/p_cam_pose.json

# 2. frames out of the bag (system ROS 3.12; rosbag2_py will not load in Isaac's 3.11)
source /opt/ros/jazzy/setup.bash
PYTHONNOUSERSITE=1 python3 tools/export_p_cam_frames.py \
  --bag out/evidence/ship-day/f3.1-violation/rosbag --out .../png

# 3. detector + geometry (Isaac's 3.11 for torch; no Isaac Sim, no lock)
~/isaac/env_isaaclab/bin/python tools/p_cam_infer.py \
  --frames .../png --camera-pose out/p_cam_pose.json --out .../stations.json

# 4. the timing control, then the table
.venv/bin/python tools/p_cam_render_lag.py --stations .../stations.json \
  --schedule .../commanded-pose-schedule.json --out .../render-lag.json
.venv/bin/python tools/p_cam_speed_table.py --stations .../stations.json \
  --schedule .../commanded-pose-schedule.json --lag .../render-lag.json \
  --out .../speed-table.json

# controls, GPU-free
.venv/bin/python tools/p_cam_station_bench.py --dataset out/datasets/p_cam_v1 \
  --resolution lo --intrinsics .../png/index.json --out .../station-bench-lo.json
```

| | |
|---|---|
| Isaac | 5.1, RTX 5070 Ti, 3313 MiB, headless |
| Scene | `arena_corridor_robot1_nominal_m6_n3.usd`, scale 0.30, physics deactivated |
| Camera | P's mast at `[5.235, 0.72, 1.5]`, 640×360 at 15 Hz, one render product |
| Detector | RT-DETR r18vd fine-tune, `rtdetr-lo/checkpoint`, score threshold 0.3 — the training run's own, not retuned |
| Truth | the adapter's commanded pose schedule (`--drive-out`); evaluation input, never an observer input |
| Domains | rendered on 42, **recorded on 43** — every frame measured here crossed the gateway |
| Result | **reported, not passed.** There was no target and no threshold to meet |

## The table — F3.1, one pass at A's measured cruise (0.22 m/s)

| gate | width | limit | n | measured | truth | error | error % | σ | secant % | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.6 | 1.65 | 0.30 | 38 | 0.1879 | 0.2200 | −0.0321 | −14.61 | 0.0015 | −14.55 | compliant |
| 1.2 | 1.50 | 0.25 | 32 | 0.1962 | 0.2200 | −0.0238 | −10.80 | 0.0010 | −11.36 | compliant |
| 1.8 | 1.35 | 0.25 | 34 | 0.1994 | 0.2200 | −0.0206 | −9.38 | 0.0016 | −7.99 | compliant |
| 2.4 | 1.20 | 0.04 | 37 | 0.2020 | 0.2200 | −0.0180 | −8.20 | 0.0011 | −8.86 | OVER |
| 3.0 | 1.05 | 0.04 | 33 | 0.1981 | 0.2200 | −0.0219 | −9.95 | 0.0011 | −8.46 | OVER **CONFIRMED** |

208 of 376 frames produced a station; the other 168 are after A rounds the
corner and leaves the frustum, which is correct behaviour rather than a miss.

σ is the standard error of the fitted slope, ~0.0013 m/s — **1000× smaller than
the error.** The estimator is precise and biased, so the confirmation rule's
2σ discount does nothing at all against this kind of error. Worth stating,
because a pipeline that reported only σ would look excellent.

The `secant` column is what `GateSpeedEstimator` computes — two crossings, one
difference — on the same stations. It tracks the window fit within ~1.5
percentage points and is never better, which is what 2 samples against 32–38
should do.

## Defect 1: the image does not show the scene its timestamp claims

`CLAUDE.md` has carried "the pose-to-render latency is uncharacterised" as an
open limit since v1, bounded there at one camera period — 0.066 m at 1.0 m/s.
**It is three orders of magnitude larger than that bound, and it is not a
latency but a rate deficit.**

`tools/p_cam_render_lag.py` subtracts two times that share a clock: when the
schedule put A at station x, and when the pixels first show A there.

| station | schedule | pixels | lag | lag in metres |
|---|---|---|---|---|
| 0.5 | 2.53 s | 2.91 s | +0.38 s | +0.083 |
| 1.0 | 4.82 s | 5.49 s | +0.67 s | +0.148 |
| 1.5 | 7.12 s | 8.10 s | +0.98 s | +0.215 |
| 2.0 | 9.40 s | 10.63 s | +1.24 s | +0.272 |
| 2.5 | 11.70 s | 13.10 s | +1.40 s | +0.307 |
| 3.0 | 13.98 s | 15.67 s | +1.69 s | +0.372 |

**The lag grows +0.112 s per second of sim time**, so the content advances at
0.888× the clock and any speed read off those stamps is low by 11.2%. Measured
mean error: −10.59%. **Residual: +0.62%.**

The mechanism is consistent with the adapter stepping sim time faster than the
render product delivers frames, with each delivered frame stamped at publish
rather than at capture. The 0.08 m/s pass, which needs 2.8× more sim time for
the same route, degrades exactly as that predicts — lag growth +0.287 s/s,
content at 0.713× the clock.

This is why F3.2 is published as a **timing control rather than a compliant
result.** Its gates are all reported compliant, but at 0.713× the clock and
with the outlier population below dominating a run that is three times longer,
its speed figures do not measure the estimator and are not quoted as if they do.

## Defect 2: the detector is accurate when right and unbounded when wrong

Three measurements over one geometry, isolating a stage at a time.

| what | station bias | station sd | n |
|---|---|---|---|
| **Projection only** — truth boxes, labelled val split | **+0.023 m** | 0.056 m | 412 |
| Detector, same frames, good detections | +0.032 m | 0.094 m | 330 |
| Detector, same frames, **all** detections | +0.131 m | **0.850 m** | 410 |

The projection is sound: bottom-centre pixel → ray through the published K →
ground plane at z = 0 recovers station to 2.3 cm, and regressing estimate on
truth gives **0.9895 × truth + 0.043** — unity to about 1%.

The detector matches it when it is right: median error +0.040 m, MAD 0.021 m.
But **19.5% of frames (80 of 410) land more than 0.5 m from the median, and the
worst is 3.5 m out.**

**Confidence does not discriminate them.** Gross failures score a median 0.945;
good detections score 0.936. So the 99.3% detection rate in `../detector/`
measures a different thing from what enforcement needs: it asks whether a box
overlaps the robot, not whether the box puts the robot in the right metre.

An ordinary least-squares fit is not robust to 20% contamination. A robust fit
is the obvious next step and is **named, not applied** — swapping the method
after seeing the result is tuning to the answer.

## What is NOT claimed

- **This is not the autonomous run.** F3.1 is a scripted constant-speed pass at
  the speed A was *measured* driving (ADR 0038). Autonomy is evidenced
  separately on A's own plane. The two cannot yet be one run: the v1 estimator
  returns the *camera's* station and the camera is now a static mast, and the
  fleet's `sim_runner.py` carries no camera.
- **One profile, one pass per speed.** `nominal_m6_n3` only.
- **No ArUco baseline A/B.** Cut for delivery; ADR 0024's classical control
  remains unmeasured against the learned path.
- **+0.62% is a residual, not a certified accuracy.** It is one run, and it is
  the difference of two measurements each with their own error.

## A mistake caught here, because it would otherwise have shipped

The first version of this table used a camera pose exported from
`out/corridor.usda`. The mast sits at `[5.235, 2.82, 1.5]` there and at
`[5.235, 0.72, 1.5]` in the composed arena the frames were rendered from —
**2.1 m apart and aimed differently.**

Nothing failed. All 208 frames produced a station, all five gates were covered,
the violation was confirmed at the same gate, and the table read *better* than
the correct one — 7.5% mean error instead of 10.6%.

`p_cam_speed_table.py` now refuses when the pose's stage and the schedule's
stage disagree; `p_cam_station_bench.py` refuses when the pose disagrees with
the eye the dataset was rendered from. Two guards, because the failure is
silent in both directions and the wrong answer is plausible.

## Artifacts

| file | what |
|---|---|
| `speed-table-f3.1-violation.json` | the table above, per gate, both estimators, with the attribution |
| `render-lag-f3.1-violation.json` | defect 1, per station |
| `render-lag-f3.2-compliant.json` | the same at 0.08 m/s, where it is 2.6× worse |
| `station-bench-lo.json` | projection-only control, 412 labelled val frames |
| `diagnose-478.png` | the frame that exposed defect 1: detector box on A, yellow cross where the schedule says A should be |
