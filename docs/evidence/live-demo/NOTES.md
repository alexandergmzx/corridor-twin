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
| Commit | `5b2bc6c` plus the launch-argument fix recorded below |
| Date | 2026-07-27 |
| Isaac Sim | 5.1.0, bundled Jazzy bridge |
| GPU | NVIDIA GeForce RTX 5070 Ti, 16303 MiB, driver 580.173.02 |
| Host | Linux Mint (still reported unsupported by the checker; Ubuntu 24.04 remains the fallback) |
| Stage / profile | `out/corridor.usda`, `nominal_m6_n3` (m = 6.0 m, n = 3.0 m) |
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
| Images recorded | 339 over a 23.800 s stamp span |
| Delivered rate | mean 14.20 Hz, max 15.00 Hz, min 5.00 Hz |
| Intrinsics | fx = fy = 417.032, cx = 320.0, cy = 180.0 |
| Distortion | `plumb_bob`, all coefficients zero |

The mean sits below the 15 Hz contract because a few frames stretched during
recording; the maximum reaches exactly 15.00 Hz. This is a delivery-rate
observation under a concurrent `ros2 bag record`, not a renderer result, and it
does not affect the speed measurement: the observer differentiates message
stamps, so a late frame moves when a gate crossing is interpolated, not how far
apart the gates are.

### Camera-derived enforcement

Truth speed 1.0 m/s. The observer never subscribed to pose, odometry, TF or the
configured speed; every row below came from ArUco correspondences in the frames
above.

| Gate | Measured speed | sigma | Local width | Limit | State |
|---:|---:|---:|---:|---:|---|
| 4.0 m | 1.001 m/s | 0.014 | 5.00 m | 1.20 m/s | compliant |
| 6.0 m | 1.002 m/s | 0.014 | 4.50 m | 1.20 m/s | compliant |
| 8.0 m | 0.963 m/s | 0.014 | 4.00 m | 0.80 m/s | over |
| 10.0 m | 1.019 m/s | 0.013 | 3.50 m | 0.80 m/s | over |

- **Maximum speed error 0.0371 m/s** at a 1.0 m/s truth speed.
- **Every enforcement gate after the first produced a measurement.** Gate 2.0 is
  the first crossing and cannot carry a speed, which needs two.
- **Exactly one violation**: event 1 at station 10.0 m, exceedance 0.194 m/s
  against the 0.80 m/s corner limit.

That is the demonstration in one run: an unchanged 1.0 m/s is legal on the wide
approach and becomes an offense once the corridor narrows and the rule tightens.
Both gates in the strict zone were measured, which is what lets the conservative
two-estimate confirmation fire at the corner at all.

### Resources

```
ISAAC_ROS_CAMERA_GPU name=NVIDIA GeForce RTX 5070 Ti used_mib=3411 total_mib=16303
ISAAC_ROS_CAMERA_PASS updates=1433 profile=nominal_m6_n3 static_probe=False drive=True render_products=1
```

3411 MiB of 16303 MiB, with one render product and one camera.

## Frames

| File | What it shows |
|---|---|
| [`corridor-approach.png`](corridor-approach.png) | t = 6.0 s. Corridor wall gates on both tapering faces plus the far-field references, all decoding together |
| [`corner-references.png`](corner-references.png) | t = 9.8 s, inside the strict zone. The corridor gates have left the frustum and the height-staggered references on the north-wall extension and the east face carry the pose. This is the coverage the reference plates were added for, working in a real render |

[`summary.json`](summary.json) carries the full per-message record.

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
- Exercise the GUI path. This run was `--headless`; the viewport variant shares
  the same graph but was not measured here.
- Say anything about profiles other than `nominal_m6_n3`.

## Defect this rehearsal found

The first attempt failed with `malformed launch argument 'corridor_profile:='`.
`ros2 launch` rejects an empty value outright, so with no profile requested the
entire ROS side refused to start while Isaac ran on regardless — a failure that
reads like a DDS problem and is not. `tools/run_demo.sh` now omits the argument
instead of passing it empty. Rehearsing is what surfaced it.
