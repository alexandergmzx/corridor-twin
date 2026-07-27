# corridor-twin

An interview-sized digital-twin scenario for OpenUSD, ROS 2 Jazzy, and NVIDIA
Isaac Sim. Robot A delivers a package to person B through a tapered corridor.
Police observer P has no direct line of sight to A, but is permitted to consume
A's front-camera feed and detect speed violations from surveyed ArUco markers.

## Current status

**Phase 1 baseline and first Isaac/ROS integration working — 2026-07-26.**
Parametric USDA generation, finite width variants, static colliders, ArUco
assets, a shared manifest, camera-only speed estimation, synthetic ROS playback,
violation events, and continuous occlusion evidence are implemented. The RTX
5070 Ti passes Isaac Sim 5.1's hardware checks. A version-specific adapter now
publishes the single 640×360 RGB camera, matching `CameraInfo`, and simulation
`/clock` through the installed Jazzy bridge. Independent ROS probes passed in
both headless and visible modes at exactly 15 Hz.

The checker still marks the host unsupported because it is Linux Mint. This is
recorded as a platform risk rather than hidden by the successful hardware and
live-stream checks.

The provisional A/B/P coordinates and demonstration speed policy still need to
be reconciled with the interviewer-supplied diagram before calling the geometry
final. Robot motion along the authored delivery path is the next integration
step.

## Architecture at a glance

```mermaid
flowchart LR
    Config["corridor.yaml"] --> Build["scene.build"]
    Build --> USD["corridor.usda"]
    Build --> Manifest["corridor.manifest.json"]
    USD --> Isaac["Isaac Sim 5.1"]
    Isaac --> Contract["Image + CameraInfo + /clock"]
    Synthetic["Synthetic camera"] --> Contract
    Contract --> Observer["police_observer"]
    Manifest --> Observer
    Observer --> Output["Speed estimate / violation"]
    USD --> Proof["Occlusion certificate"]
    Manifest --> Proof
```

The observer is prohibited from reading simulated pose, odometry, TF-derived
robot position, or synthetic ground truth. Its measurement timestamp comes from
the image header.

## Repository layout

- `src/corridor_scene`: `usd-core` authoring, marker assets, manifest generation,
  stage validation, and CPU geometric occlusion verification.
- `src/corridor_interfaces`: timestamped speed-estimate and violation messages.
- `src/police_observer`: pure perception core, ROS adapter, synthetic frame
  generator, and synthetic publisher.
- `docs/adr`: architecture decisions and consequences.
- `docs/DESIGN.md`: versioned system design and hardware budget.
- `docs/SENSOR-FEED.md`: the robot-to-observer interface contract.
- `test`: repository and end-to-end contract tests.
- `tools/isaac_5_1_ros_camera.py`: installed-version camera/clock adapter, kept
  outside the CPU-testable packages.
- `tools/ros_camera_contract_probe.py`: simulator-independent live ROS contract
  probe.

## Build and run

### 1. Reconcile the provisional inputs

- Add the supplied diagram to `docs/` or reconcile its coordinates with
  `src/corridor_scene/config/corridor.yaml`.
- Reconcile A, B, P, and the delivery path with the diagram.
- Approve the named `(m,n)` profiles.
- Define the demonstration-only width-to-speed-limit rules.

### 2. Review the demo GPU qualification

The installed RTX 5070 Ti reports 16303 MiB with driver 580.173.02. The complete
activation evidence and repeatable commands are in `docs/ACTIVATION.md`.
Headless and visible loaded-stage snapshots used 916 MiB and 871 MiB total,
respectively. With the one camera render product and ROS bridge active, measured
totals were 2,494 MiB headless and 2,591 MiB visible. Every run used real-time
`RaytracedLighting`; none used path tracing. The unsupported Mint result and
IOMMU warning remain known risks; Ubuntu 24.04 is the fallback rather than an
unrecorded last-minute change.

### 3. Create the development environment

Keep ordinary OpenUSD/ROS development separate from Isaac's Python environment:

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3.12 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-dev.txt
python -m pip install --no-deps -e src/corridor_scene -e src/police_observer
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
  --os=ubuntu:noble
```

Do not install pip `usd-core` into Isaac Sim's bundled Python. Do not source two
different ROS installations into the same shell.

### 4. Generate and prove the scene

```bash
source .venv/bin/activate
python -m scene.build --m 6.0 --n 3.0 --out out/corridor.usda
python -m scene.occlusion \
  --stage out/corridor.usda \
  --manifest out/corridor.manifest.json \
  --out out/occlusion-certificate.json
```

The build creates the USDA, `corridor.manifest.json`, and marker PNG files. The
occlusion command exits nonzero if any continuous path interval or composed-mesh
audit ray is uncertified.

### 5. Build and test ROS

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --symlink-install
source install/setup.bash
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
pytest -q
colcon test
colcon test-result --verbose
```

Or run the same sequence, including lint, with `bash tools/check_workspace.sh`.
The explicit `PYTHONPATH` is needed because `colcon test` launches the system
Python even when the calling shell has an active venv; it keeps both `pxr` and
the NumPy version used by ROS/OpenCV deterministic on this host.

### 6. Run the simulator-free ROS demo

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch police_observer synthetic_demo.launch.py \
  manifest:=$PWD/out/corridor.manifest.json speed_mps:=1.8
```

In another sourced shell:

```bash
ros2 topic echo /police/speed_violation \
  corridor_interfaces/msg/SpeedViolation --once --no-daemon
```

The launch deliberately sets `PYTHONNOUSERSITE=1`: this machine has a user-local
NumPy 2.2 wheel, while ROS Jazzy's OpenCV/cv_bridge binaries use the NumPy 1.x
ABI. No global package is modified. Add `use_sim_time:=true` to make the
synthetic publisher the single `/clock` source for both nodes.

### 7. Run the live Isaac camera contract

Start the external probe in a system-ROS terminal before starting Isaac:

```bash
source /opt/ros/jazzy/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 \
  tools/ros_camera_contract_probe.py --minimum-pairs 12 --timeout 90
```

In a second terminal, start the finite adapter. Do not source system ROS there;
the adapter re-executes with Isaac's bundled Jazzy libraries and rejects Python
path leakage from the host environment:

```bash
OMNI_KIT_ACCEPT_EULA=YES \
  ~/isaac/env_isaaclab/bin/python tools/isaac_5_1_ros_camera.py \
  out/corridor.usda --updates 420 --report-gpu-memory
```

Add `--gui` for the finite visible-viewport check. Success requires both
`ROS_CAMERA_PROBE_PASS` and `ISAAC_ROS_CAMERA_PASS`; an Isaac-only pass does not
prove the ROS interface. The adapter creates exactly one render product and no
pose, odometry, TF, depth, or test-truth publisher.

### 8. Repeat the installed Isaac 5.1 composition smoke

```bash
OMNI_KIT_ACCEPT_EULA=YES \
  ~/isaac/env_isaaclab/bin/python tools/isaac_5_1_smoke.py \
  out/corridor.usda --updates 60 --report-gpu-memory

OMNI_KIT_ACCEPT_EULA=YES \
  ~/isaac/env_isaaclab/bin/python tools/isaac_5_1_smoke.py \
  out/corridor.usda --gui --updates 240 --report-gpu-memory
```

The smoke uses explicit real-time `RaytracedLighting`, 640×360, no path tracing,
and no sensor render product. Both modes validate composition and
installed-version schemas; the finite GUI mode opens a visible viewport and
closes itself. Use the preceding live contract for camera/ROS validation.

## Deliberate GPU budget

The selected RTX 5070 Ti has 16 GB VRAM, but this demo still avoids spending that
capacity casually. The initial Isaac configuration remains:

- one 640×360 RGB camera at 15 Hz;
- one small robot and primitive static environment;
- small marker textures and simple opaque materials;
- primitive/convex colliders where possible;
- CPU physics until measurement shows a reason to change;
- no depth, LiDAR, radar, segmentation, Replicator, RL, or extra render products;
- no interactive path tracing.

The soft operating target is below 14 GB steady-state VRAM, leaving recovery
headroom for the GUI and ROS bridge.

## Interview talking points

- USD is generated deterministically and measured from the composed stage; the
  GUI is a consumer, not the source of truth.
- USD variants represent finite named corridor profiles, not arbitrary numeric
  sliders.
- Speed is inferred only from camera pixels, calibration, surveyed marker poses,
  and image acquisition timestamps.
- Synthetic frames make the estimator falsifiable before simulator rendering is
  trusted.
- P's occlusion is a tested continuous geometric property with failing controls,
  not a visual claim.
- Simulated time uses one `/clock` publisher and resets estimator state on time
  jumps.
- Isaac-specific code is isolated and checked against the installed version's API.

## Explicit non-goals

Docker, Replicator, Isaac Lab/RL, Unreal/Unity, a multi-sensor robot, large asset
packs, cloud deployment, and CI beyond the single lint-and-test workflow are out
of scope for this interview demo.

## Documentation

- [Visual project map and growth tracker](docs/README.md)
- [System design](docs/DESIGN.md)
- [Sensor-feed contract](docs/SENSOR-FEED.md)
- [Hardware and Isaac activation record](docs/ACTIVATION.md)
- [Development workflow, CI recovery, and commit history](docs/DEVELOPMENT.md)
- [Architecture decisions](docs/adr/README.md)

## License

Apache-2.0. See `LICENSE`.
