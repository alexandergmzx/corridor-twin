# Live camera-only enforcement run

The first end-to-end demonstration: A drives the authored route in Isaac Sim,
its single RGB camera is the only sensor, and the police observer recovers
station, speed and one violation from those pixels alone.

## Command

```bash
bash tools/run_demo.sh --headless --record
```

One command starts both halves in the two environments they require. The
system-Jazzy consumers come up first, then Isaac starts in a shell with
`AMENT_PREFIX_PATH`, `PYTHONPATH`, `ROS_DISTRO` and `CMAKE_PREFIX_PATH` unset.

## Configuration

| Field | Value |
|---|---|
| Commit | `3cadbd3` (second run; geometry unchanged by the band clamp in `7a5980a`) |
| Date | 2026-07-27 |
| Isaac Sim | 5.1.0, bundled Jazzy bridge |
| GPU | NVIDIA GeForce RTX 5070 Ti, 16303 MiB, driver 580.173.02 |
| Host | Linux Mint (still reported unsupported by the checker; Ubuntu 24.04 remains the fallback) |
| Stage / profile | `out/corridor.usda`, `nominal_m6_n3` (m = 6.0 m, n = 3.0 m) |
| Geometry | East-face reference relocated to `along_m 0.75`, 0.60 m (R17). No accepted marker is occluded in this run |
| Path speed | constant 1.0 m/s |
| Render mode | `RaytracedLighting`, **read back** from `/rtx/rendermode` and `/rtx-defaults/rendermode` |
| Anti-aliasing | enum 3 on both trees |
| Render products | 1 |

## Measured result — PASS

### Motion

```
ISAAC_ROS_CAMERA_DRIVE speed_mps=1.0 route_s_m=23.851 route_length_m=23.851
                       reached_end=True updates=1433 sim_span_s=23.867
```

A completed the whole 23.851 m line-arc-line route in 23.867 s of **simulation**
time. Commanded path speed is therefore 0.99933 m/s against a requested 1.0,
a 0.07 % deviation. Station comes from the simulation clock, never wall time.

### Camera

| Property | Value |
|---|---|
| Encoding / size | `rgb8`, 640x360 |
| Images recorded | 328 over a 23.467 s stamp span |
| Delivered rate | mean 13.93 Hz, max 15.00 Hz, min 5.00 Hz |
| Intrinsics | fx = fy = 417.032, cx = 320.0, cy = 180.0 |
| Distortion | `plumb_bob`, all coefficients zero |

The mean sits below the 15 Hz contract because some frames stretch during
recording; the maximum reaches exactly 15.00 Hz in both runs.

**Across two runs of the same command** the minimum was 3.75 Hz and 5.00 Hz, so
**3.75 Hz is the worst observed** rather than a characterised bound — two
samples is not a distribution. Both are `ros2 bag record` artifacts rather than
renderer results: the recorder competes for the same host while the graph runs.

Neither reaches the speed measurement. The observer differentiates message
stamps, so a stretched frame changes *when* a gate crossing is interpolated,
not how far apart the surveyed gates are. The two runs bear that out — their
worst delivered rates differ by a third while their maximum speed errors, 0.0331
and 0.0317 m/s, differ by 0.0014.

### Camera-derived enforcement

Truth speed 1.0 m/s. The observer never subscribed to pose, odometry, TF or the
configured speed; every row below came from ArUco correspondences in the frames
above.

| Gate | Measured speed | sigma | Local width | Limit | State |
|---:|---:|---:|---:|---:|---|
| 4.0 m | 0.999 m/s | 0.016 | 5.00 m | 1.20 m/s | compliant |
| 6.0 m | 1.003 m/s | 0.016 | 4.50 m | 1.20 m/s | compliant |
| 8.0 m | 0.968 m/s | 0.013 | 4.00 m | 0.80 m/s | over |
| 10.0 m | 1.012 m/s | 0.011 | 3.50 m | 0.80 m/s | over |

- **Maximum speed error 0.0317 m/s** at a 1.0 m/s truth speed.
- **Every enforcement gate after the first produced a measurement.** Gate 2.0 is
  the first crossing and cannot carry a speed, which needs two.
- **Exactly one violation**: event 1 at station 10.0 m, exceedance 0.189 m/s
  against the 0.80 m/s corner limit.

That is the demonstration in one run: an unchanged 1.0 m/s is legal on the wide
approach and becomes an offense once the corridor narrows and the rule tightens.
Both gates in the strict zone were measured, which is what lets the conservative
two-estimate confirmation fire at the corner at all.

### Resources

```
ISAAC_ROS_CAMERA_GPU name=NVIDIA GeForce RTX 5070 Ti used_mib=3546 total_mib=16303
ISAAC_ROS_CAMERA_PASS updates=1433 profile=nominal_m6_n3 static_probe=False drive=True render_products=1
```

3546 MiB of 16303 MiB, with one render product and one camera.

## Frames

| File | What it shows |
|---|---|
| [`corridor-approach.png`](corridor-approach.png) | t = 6.0 s. Corridor wall gates on both tapering faces plus the far-field references, all decoding together |
| [`corner-references.png`](corner-references.png) | t = 9.8 s, inside the strict zone. The corridor gates have left the frustum and the height-staggered references on the north-wall extension and the east face carry the pose. This is the coverage the reference plates were added for, working in a real render |

[`summary.json`](summary.json) carries the full per-message record.

## Runtime subscription audit

Source and AST tests prove what the observer *can* subscribe to. This is what it
actually did subscribe to, captured live while it was consuming Isaac pixels:

```bash
ros2 node info /police_observer
```

[`runtime-node-info.txt`](runtime-node-info.txt) records the result. The
observer's subscriber list is exactly:

```
/clock:                          rosgraph_msgs/msg/Clock
/robot/front_camera/camera_info: sensor_msgs/msg/CameraInfo
/robot/front_camera/image_raw:   sensor_msgs/msg/Image
```

No pose, no odometry, no TF, no configured speed, no truth topic. `/clock` is
subscribed by rclpy's own `TimeSource` under `use_sim_time`, which carries
simulation time and says nothing about where A is. The display node is captured
in the same file and reads only the two topics the observer publishes.

## What this run does and does not establish

**Does:**

- Continuous authored motion driven from simulation time.
- Cross-ABI delivery: Isaac's bundled Jazzy publisher to a system-Jazzy
  observer, over the default Fast DDS.
- Camera-only station and speed recovery from RTX-rendered pixels, with the
  reference fiducials providing corner coverage in practice and not only in the
  synthetic model.
- One render product, one camera, renderer mode read back rather than requested.

**Does not:**

- Replace the static qualification. That is a separate paired capture with its
  own dwell schedule and mirror control, and it is still outstanding.
- Measure the pose-to-render latency. Whether a pose written before
  `app.update()` lands in that frame or the next is unmeasured, and no offset
  was applied to compensate for it. At 1.0 m/s one camera period is 0.066 m,
  which bounds the effect on a single station but has not been characterised.
- Produce a full GUI run. The viewport path was confirmed separately (see
  below) but the recorded enforcement result above is from `--headless`.
- Say anything about profiles other than `nominal_m6_n3`. At this speed neither
  alternate can produce a violation at all: `wide_corner_m6_n4_5`'s narrowest
  gate is 4.75 m (limit 1.2 m/s) and `uniform_m6_n6` is 1.5 m/s throughout, so
  neither has any gate in the 0.8 m/s zone.

## Known risk: the violation has no redundancy

The strict 0.8 m/s zone holds exactly two gates, 8.0 and 10.0, and the policy
requires two consecutive over-limit measurements to confirm. Both gates must
therefore be measured **and** over-limit, or this run produces no violation at
all — a silent absence rather than a wrong answer.

What makes that acceptable today is that the margin is comfortable rather than
marginal: gate 8 measured 0.968 m/s against a 0.80 m/s limit, and the
conservative 2σ lower bound is 0.942 m/s. The risk is a *missed* measurement,
not a borderline one, and `e0bea0c` reduced exactly that failure mode by
stopping noise-level backward station steps from discarding gate history.

Mitigation is to rehearse rather than to widen the zone a second time. The
simulator-free fallback produces the same single event if the live run fails.

## Viewport path

The presentation uses the GUI, so it was run separately:

```bash
EVIDENCE_DIR=$PWD/out/evidence/live-demo-gui UPDATES=600 bash tools/run_demo.sh
```

```
ISAAC_ROS_CAMERA_RENDER_READY active_render_mode='RaytracedLighting' default_render_mode='RaytracedLighting'
                              active_anti_aliasing=3 default_anti_aliasing=3
ISAAC_ROS_CAMERA_DRIVE speed_mps=1.0 route_s_m=9.983 route_length_m=23.851 reached_end=False updates=600
ISAAC_ROS_CAMERA_GPU name=NVIDIA GeForce RTX 5070 Ti used_mib=3547 total_mib=16303
ISAAC_ROS_CAMERA_PASS updates=600 profile=nominal_m6_n3 static_probe=False drive=True render_products=1
```

> **Measured before the R17 geometry correction**, against a headless run that
> used 3,411 MiB. The plate relocation moved headless to 3,486 MiB; the viewport
> run was not repeated, so treat 3,547 MiB as the pre-correction figure and the
> 136 MiB viewport delta as the transferable result.

The viewport cost 136 MiB over the headless run of the same build, and the
renderer state, render-product count and drive loop are unchanged. `reached_end=False`
is the `UPDATES=600` cap used to keep this check short, not a failure: 600
updates is 9.983 s of simulation time, so A covered the corridor approach and
stopped short of the corner. The default `UPDATES=3000` is 50 s of simulation
time against a 23.9 s route, which is what the headless run completed.

Simulation advances at 60 Hz in both modes, matching the adapter contract's
`simulation_hz`.

## Defect this rehearsal found

The first attempt failed with `malformed launch argument 'corridor_profile:='`.
`ros2 launch` rejects an empty value outright, so with no profile requested the
entire ROS side refused to start while Isaac ran on regardless — a failure that
reads like a DDS problem and is not. `tools/run_demo.sh` now omits the argument
instead of passing it empty. Rehearsing is what surfaced it.
