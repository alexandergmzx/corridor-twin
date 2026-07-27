# ADR 0009: Isolate a minimal installed-version Isaac ROS camera adapter

- Status: Accepted
- Date: 2026-07-26
- Decision owners: corridor-twin maintainers

## Context

Phase 1 deliberately authored USD and tested perception without importing Isaac
Sim. The first simulator integration must now publish A's camera feed without
weakening that boundary or giving P access to simulator truth.

This workstation has Isaac Sim 5.1.0 under Python 3.11 and system ROS 2 Jazzy
under Python 3.12. The installed 5.1 bridge extension includes its own Jazzy
libraries. Inheriting the system ROS `PYTHONPATH` in Isaac attempts to load a
Python 3.12 `rclpy` binary into Python 3.11. Extension namespaces and camera
schemas have also changed between Isaac releases, so an implementation recalled
from an older tutorial is unsafe.

## Decision

Keep a narrow, version-labelled executable in
`tools/isaac_5_1_ros_camera.py`. It will:

1. re-execute with Isaac's bundled Jazzy bridge libraries and with host ROS
   Python paths removed;
2. use only node types verified in the installed 5.1 extension and OGN metadata;
3. create exactly one 640×360 render product in real-time
   `RaytracedLighting` mode;
4. publish RGB `Image` and matching `CameraInfo` at 15 Hz from a 60 Hz fixed
   simulation timeline;
5. publish simulation `/clock` from the same graph;
6. configure explicit best-effort, volatile QoS;
7. publish no pose, odometry, TF, depth, segmentation, or ground truth; and
8. remain outside the CPU-testable ROS/OpenUSD packages.

Use the current `Camera.set_opencv_pinhole_properties` schema for calibration.
Do not persist the runtime ROS graph into the generated scenario USDA: the
generator remains simulator-independent, while the adapter is intentionally
tied to the installed simulator version.

An external system-Jazzy probe must validate DDS delivery, synchronized stamps,
frames, dimensions, encoding, intrinsics, clock behavior, endpoint QoS, rate,
and publisher cardinality. A local OmniGraph creation success is insufficient.

## Consequences

- Phase 1 packages and CI remain independent of Isaac Sim and a GPU.
- The environment ABI boundary is explicit and repeatable rather than dependent
  on the caller's shell startup files.
- The observer sees the same contract from synthetic and Isaac publishers.
- One render product keeps the integration auditable and the VRAM budget small.
- Runtime graph creation must be re-verified when Isaac Sim is upgraded; a new
  version-specific adapter or an explicit migration replaces guesswork.
- Robot motion remains a separate change. A static live image proves transport
  and timing, not end-to-end speed estimation in Isaac.

## Validation

On 2026-07-26, external probes passed against both headless and visible Isaac
runs. Each observed 12 synchronized 640×360 RGB/calibration pairs at 15.000 Hz,
a monotonic simulation clock, best-effort/volatile endpoints, and one publisher
per topic. Total GPU memory was 2,494 MiB headless and 2,591 MiB visible.

## Alternatives considered

- **Source system ROS before starting Isaac:** rejected because the Python ABIs
  differ and the installed bridge already supplies the supported Jazzy runtime.
- **Build the graph manually in the GUI:** useful for exploration, but rejected
  as the source of truth because it is harder to reproduce and review.
- **Publish robot pose for easier validation:** rejected by the camera-only
  evidence constraint and ADR 0002.
- **Add depth or a second render product now:** rejected because neither is
  required by the observer contract.

## Installed sources checked

- `isaacsim.ros2.bridge` extension metadata and OGN node documentation;
- the installed ROS 2 camera graph shortcut implementation;
- the installed `isaacsim.sensors.camera.Camera` API; and
- the installed Isaac Sim 5.1 ROS camera and clock documentation.
