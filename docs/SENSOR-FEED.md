# Robot camera to police observer contract

| Field | Value |
|---|---|
| Contract version | 0.2.1 |
| Status | Implemented baseline |
| Last updated | 2026-07-26 |

## Scope

P may read A's front-camera image and calibration data. P has no direct line of
sight and the `police_observer` must not consume ground-truth pose, odometry,
simulator transforms, or test truth.

Topic names in node code should be relative and remappable. The table shows the
resolved demo names.

## Topics

| Direction | Resolved topic | Type | QoS | Purpose |
|---|---|---|---|---|
| A → P | `/robot/front_camera/image_raw` | `sensor_msgs/msg/Image` | sensor data | Rectified or distortion-described color image |
| A → P | `/robot/front_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | sensor data | Intrinsics and distortion matching the image |
| P → consumers | `/police/speed_estimate` | `corridor_interfaces/msg/SpeedEstimate` | reliable, volatile, depth 10 | Validity, speed, uncertainty, width, and limit |
| P → consumers | `/police/speed_violation` | `corridor_interfaces/msg/SpeedViolation` | reliable, volatile, depth 10 | Debounced event, not a latched alarm |
| harness only | `/test/ground_truth/speed` | `geometry_msgs/msg/TwistStamped` | reliable, volatile, depth 10 | Evaluator reference in `twist.linear.x`; forbidden to observer |
| time source | `/clock` | `rosgraph_msgs/msg/Clock` | best effort, volatile, keep-last depth 1 | Present only in simulated-time modes |

The initial local demo uses raw images. A compressed transport may be added only
after measuring bandwidth and confirming installed Isaac/ROS support; lossy
compression must then be included in estimator accuracy tests.

## Image requirements

- Initial format: `rgb8` or `bgr8`; the configured value must be handled
  explicitly rather than guessed.
- Initial resolution: 640×360.
- Initial rate: 15 Hz.
- `header.stamp`: image acquisition/simulation time.
- `header.frame_id`: `robot_front_camera_optical_frame`.
- Optical axes: X right, Y down, Z forward.
- Empty or zero timestamps are invalid in simulated-time mode.

## CameraInfo requirements

- `header.frame_id` must equal the image frame.
- Width and height must match the image.
- `K`, distortion model, and coefficients describe the delivered pixels.
- For a per-frame publisher, the CameraInfo timestamp must equal the image
  timestamp. A static/transient calibration design requires a separate accepted
  contract revision.
- Changed calibration clears the estimator observation window.

## Estimate semantics

`SpeedEstimate.header.stamp` is the measurement time, normally the later
interpolated gate-crossing time, not publication time. Its frame is
`corridor_map`.

- `station_m`: longitudinal position on the surveyed centerline.
- `speed_mps`: camera-derived longitudinal speed.
- `speed_stddev_mps`: estimated one-sigma uncertainty.
- `corridor_width_m`: width evaluated at the measurement station.
- `speed_limit_mps`: active demonstration limit at that width.
- `gate_from_id` / `gate_to_id`: surveyed gates used for the interval.
- `observation_count`: image observations contributing to the estimate.
- `valid`: false until all timing, geometry, and confidence conditions pass.
- `corridor_profile`: profile whose marker map and width model were used.

Gate IDs are zero-based indices into the manifest's sorted unique marker-station
list. They are not ArUco IDs; north/south markers at one station form one gate.

Invalid estimates may be published for observability but can never produce a
violation.

## Violation semantics

A violation is a reliable, volatile event with a monotonically increasing
process-local event ID. It embeds the complete triggering estimate. The event is
emitted only after the conservative configured comparison and confirmation
duration/consecutive-estimate requirement pass.

Restarting the node may reset the event ID; consumers must combine it with the
header timestamp and publisher identity if global uniqueness is required.

## QoS

Image and CameraInfo use the ROS sensor-data profile: best effort, volatile, and a
small keep-last depth. The observer must not request reliable input from a
best-effort publisher because that pairing is incompatible.

Output estimates and events use reliable, volatile, keep-last depth 10. They are
not transient-local: a late subscriber must not mistake an old violation for a
new one.

## Time modes

### Isaac simulation

- Isaac publishes `/clock`.
- Camera helpers use simulation time, not system time.
- Observer and all launch participants set `use_sim_time=true`.

### Synthetic deterministic playback

- One harness component publishes `/clock` from a steady wall-driven scheduler.
- Synthetic publisher and observer set `use_sim_time=true`.
- Frame headers use the same synthetic timeline.

### Wall-time or hardware

- No `/clock` publisher.
- All participants set `use_sim_time=false`.
- The camera stamps acquisition as close to capture as the driver permits.

The observer clears all temporal state on backward jumps, non-monotonic image
stamps, profile changes, and clock initialization transitions.

The synthetic-clock integration test produced a violation stamped at 4.330643489
seconds on the generated timeline, rather than host wall time. Its camera-derived
speed was 1.8026 m/s for a 1.8 m/s truth input.

## Forbidden dependencies

Production observer code and launch files must not subscribe to:

- `/ground_truth/*` or `/test/ground_truth/*`;
- pose or model-state topics from a simulator;
- odometry used as a speed shortcut;
- a TF transform whose source is simulated robot truth.

TF may eventually express a surveyed static marker map only if an ADR and test
make that distinction explicit. Phase 1 uses the versioned manifest directly.

## Contract acceptance tests

- Image and CameraInfo frames, dimensions, and stamps agree.
- Best-effort camera publishers communicate with the observer.
- Callback arrival jitter does not change an estimate for identical header
  stamps.
- Observer topic introspection shows no truth subscription.
- Backward `/clock` jumps clear gate/debounce history.
- Late violation subscribers do not receive historical events.
