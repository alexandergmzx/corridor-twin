# Static production-camera fiducial qualification

This topic records the first accepted proof that the real Isaac ROS camera path
delivers pixels from which the production estimator recovers surveyed station.
Bulk frames and logs remain under `out/evidence/static-fiducials/nominal-final/`.

```mermaid
flowchart LR
    Isaac["Isaac Sim 5.1<br/>one 640x360 product"] --> DDS["Image + CameraInfo + /clock"]
    DDS --> Capture["System Jazzy capture<br/>57 paired frames"]
    Capture --> Pixels["Pixel-only ArUco estimator"]
    Truth["Commanded camera poses<br/>file-only evaluator"] -.-> Compare["Acceptance comparison"]
    Pixels --> Compare
    Compare --> Pass["5/5 dwells pass"]
```

## Provenance

| Field | Value |
|---|---|
| Date | 2026-07-27 |
| Scene profile | `nominal_m6_n3` |
| Scene fix | `3f7fa37` |
| Isaac contract fix | `bb203c0` |
| Isaac Sim | `5.1.0-rc.19+release.26219.9c81211b.gl`, Python 3.11 |
| ROS consumer | ROS 2 Jazzy, system Python 3.12 |
| GPU / driver | NVIDIA GeForce RTX 5070 Ti 16 GB / 580.173.02 |
| Render path | Real-time `RaytracedLighting`, DLSS enum 3, no path tracing |
| Camera | One RGB product, 640x360, 15 Hz |
| Kit log | `kit_20260727_023440.log` |
| Scratch run | `out/evidence/static-fiducials/nominal-final/` |

The representative frame is an unmodified `rgb8` image selected from the first
accepted dwell. It is included for human inspection; the JSON result, not the
picture alone, carries the acceptance claim.

![Representative production ROS frame](representative-frame.png)

## Exact commands

Build the stage first:

```bash
.venv/bin/python -m scene.build --m 6.0 --n 3.0 --out out/corridor.usda
.venv/bin/python -m scene.occlusion \
  --stage out/corridor.usda \
  --manifest out/corridor.manifest.json \
  --out out/occlusion-report.json
```

Start the capture in a system-Jazzy shell:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_LOG_DIR=out/evidence/static-fiducials/nominal-final/ros-log \
PYTHONNOUSERSITE=1 .venv/bin/python tools/ros_aruco_capture.py \
  --out-dir out/evidence/static-fiducials/nominal-final/capture \
  --minimum-pairs 18 --timeout 240 --idle-after-minimum 3
```

Run Isaac in a separate shell without inherited system ROS Python paths:

```bash
env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u PYTHONPATH \
ROS_LOG_DIR=out/evidence/static-fiducials/nominal-final/isaac-ros-log \
OMNI_KIT_ACCEPT_EULA=YES \
~/isaac/env_isaaclab/bin/python tools/isaac_5_1_ros_camera.py \
  out/corridor.usda \
  --manifest out/corridor.manifest.json \
  --profile nominal_m6_n3 \
  --static-probe-out \
    out/evidence/static-fiducials/nominal-final/static-truth.json \
  --report-gpu-memory
```

Apply the positive gate and the actual-capture negative control:

```bash
PYTHONPATH=src/police_observer .venv/bin/python tools/aruco_render_gate.py \
  --capture out/evidence/static-fiducials/nominal-final/capture/capture.json \
  --truth out/evidence/static-fiducials/nominal-final/static-truth.json \
  --manifest out/corridor.manifest.json --profile nominal_m6_n3 \
  --out out/evidence/static-fiducials/nominal-final/aruco-gate.json

PYTHONPATH=src/police_observer .venv/bin/python tools/aruco_render_gate.py \
  --capture out/evidence/static-fiducials/nominal-final/capture/capture.json \
  --truth out/evidence/static-fiducials/nominal-final/static-truth.json \
  --manifest out/corridor.manifest.json --profile nominal_m6_n3 \
  --negative-control mirror --expect-fail \
  --out \
    out/evidence/static-fiducials/nominal-final/aruco-gate-mirror-negative.json
```

## Result

| Check | Threshold | Measured | Result |
|---|---:|---:|---|
| Required dwells | 5 | 5 | Pass |
| Passing selected frames | at least 2/3 per dwell | 3/3 at every dwell | Pass |
| Maximum station error | 0.050 m | 0.010563 m | Pass |
| Maximum detected-corner RMSE | 3.0 px | 1.550047 px | Pass |
| Maximum estimator reprojection RMSE | 3.0 px | 1.091647 px | Pass |
| Delivered image rate | 14.5-15.5 Hz | 14.999999 Hz | Pass |
| Unsurveyed IDs | 0 | 0 | Pass |
| Mirrored actual capture | must fail | 0 passing frames | Pass |
| GPU memory | below 14,336 MiB | 3,024 MiB | Pass |

The delivered K matrix was constant at
`[[417.0321045, 0, 320], [0, 417.0321045, 180], [0, 0, 1]]`; its maximum
difference from the configured pinhole model was 0.0000149 px. The estimator
analysis had no truth parameter. Only the independent comparison step read the
commanded static schedule.

The stable machine-readable result is
[`qualification-summary.json`](qualification-summary.json).
