# Robot camera to police observer contract

| Field | Value |
|---|---|
| Contract version | 0.4.0 |
| Status | Implemented by synthetic and Isaac 5.1 publishers; `rgb8` gated at the wire and offline, both publishers share the production `cx=width/2` convention, and the two actors now sit on separate ROS domains |
| Last updated | 2026-08-04 |

The minor version moved for a reason worth stating: **a consumer written against
0.3.3 will receive nothing after this change.** Producer and consumer are no
longer on the same communication plane, so conforming now means being on the
right domain as well as speaking the right messages. See
[ADR 0020](adr/0020-communication-domain-isolation.md).

## Scope

P may read A's front-camera image and calibration data. P has no direct line of
sight and the `police_observer` must not consume ground-truth pose, odometry,
simulator transforms, or test truth.

Since ADR 0020, most of that sentence is enforced by the transport rather than by
convention. A publishes on the **robot domain** (default 42) and P runs on the
**police domain** (default 43); DDS discovery does not cross between them, so the
forbidden topics below are not merely refused by the observer, they are not
discoverable from P's side at all. The single exception is the gateway.

Topic names in node code should be relative and remappable. The table shows the
resolved demo names.

## Evidence access matrix

| Evidence | Producer/source | Reaches P how? | `police_observer` may consume? | Test evaluator may consume? | Why |
|---|---|---|:---:|:---:|---|
| RGB image | A's camera publisher | Gateway allowlist | Yes | Yes | Primary indirect observation |
| Camera calibration | Same publisher/render product | Gateway allowlist | Yes | Yes | Converts pixels into geometric rays |
| Surveyed marker map and width policy | Versioned scenario manifest | Local file, no topic | Yes | Yes | Known infrastructure, not robot truth |
| `/clock` in simulated-time mode | Single active harness or Isaac source | Gateway allowlist | Yes | Yes | Aligns acquisition timestamps |
| Robot ground-truth pose | Simulator/harness | **Not reachable** | **No** | Yes | Would bypass camera perception |
| Robot odometry | Robot/simulator | **Not reachable** | **No** | Yes | Would turn the observer into a direct speed reader |
| Simulator-derived robot TF | Simulator | **Not reachable** | **No** | Yes | Equivalent truth shortcut through another interface |
| Harness truth speed | Synthetic publisher/test harness | **Not reachable** | **No** | Yes | Used only to quantify estimator error |

"Not reachable" is literal, not a policy statement: those producers live on the
robot domain and are absent from the gateway allowlist, so they do not appear in
P's graph at all. The observer's permission boundary is still enforced in
source/topic contract tests as well — the two mechanisms are independent, and the
source audits catch a mistake made on the wrong side of the boundary that the
transport alone would not.

## Topics

| Direction | Domain | Resolved topic | Type | QoS | Purpose |
|---|---|---|---|---|---|
| A → P | robot → police, **bridged** | `/robot/front_camera/image_raw` | `sensor_msgs/msg/Image` | sensor data | Rectified or distortion-described color image |
| A → P | robot → police, **bridged** | `/robot/front_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | sensor data | Intrinsics and distortion matching the image |
| P → consumers | police only | `/police/speed_estimate` | `corridor_interfaces/msg/SpeedEstimate` | reliable, volatile, depth 10 | Validity, speed, uncertainty, width, and limit |
| P → consumers | police only | `/police/speed_violation` | `corridor_interfaces/msg/SpeedViolation` | reliable, volatile, depth 10 | Debounced event, not a latched alarm |
| harness only | **robot only** | `/test/ground_truth/speed` | `geometry_msgs/msg/TwistStamped` | reliable, volatile, depth 10 | Evaluator reference in `twist.linear.x`; not on the allowlist, so unreachable from P |
| time source | robot → police, **bridged** | `/clock` | `rosgraph_msgs/msg/Clock` | best effort, volatile, keep-last depth 1 | Present only in simulated-time modes |

The three bridged rows are the entire sanctioned surface between the two actors.
Nothing returns from police to robot: the gateway declares no `reversed` or
`bidirectional` entry, and `test_no_topic_is_bridged_back_toward_the_robot`
fails if one appears.

`/clock` is on that list for a reason that is easy to miss. Under `use_sim_time`
rclpy's `TimeSource` subscribes to it internally, so no observer source line
constructs it and no source audit can see the dependency. Remove it from the
allowlist and the observer's clock never advances, its pipeline resets on every
frame, and the run publishes no estimates while appearing entirely healthy.

ADR 0003 requires exactly one `/clock` publisher. That now reads *per domain*:
the Isaac adapter on the robot domain, the gateway on the police domain,
republishing the same source.

The initial local demo uses raw images. A compressed transport may be added only
after measuring bandwidth and confirming installed Isaac/ROS support; lossy
compression must then be included in estimator accuracy tests.

The installed Isaac adapter maps one render product to separate current-version
`ROS2CameraHelper` and `ROS2CameraInfoHelper` nodes. A single
`ROS2PublishClock` node is the only simulator clock publisher. The graph does not
publish pose, odometry, TF, depth, segmentation, or harness truth.

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

**One event means one continuous speeding episode**
([ADR 0014](adr/0014-violation-episode-semantics.md)). While an episode is open,
further over-limit estimates extend it silently, including across a transition
into a stricter zone; only a conservative estimate at or below the applicable
limit rearms the detector. A consumer may therefore read repeat events as
genuinely separate offenses.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Rearmed
    Rearmed --> Confirming: conservative speed &gt; limit
    Confirming --> Confirming: still over, below<br/>consecutive_estimates
    Confirming --> EpisodeOpen: threshold met<br/><b>emit SpeedViolation</b>
    Confirming --> Rearmed: conservative speed &le; limit
    EpisodeOpen --> EpisodeOpen: still over &mdash; extends silently,<br/>including into a stricter zone
    EpisodeOpen --> Rearmed: conservative speed &le; limit
    Confirming --> Rearmed: temporal or calibration reset
    EpisodeOpen --> Rearmed: temporal or calibration reset
```

Every comparison in that diagram is against the **conservative** speed —
the measurement discounted by `confidence_sigma` standard deviations — never
the raw one. One function computes it, and the detector, the violation
exceedance, and the RViz readout all call through it, so a measurement near
the margin cannot open an episode by one rule and clear it by another. The
reset edges matter as much as the speeding ones: a clock discontinuity or a
material `CameraInfo` change clears an open episode rather than carrying it
across a gap where continuity can no longer be asserted.

**Episode length is not published.** `confirmation_duration_s` is the interval
from the first confirming estimate to the one that triggered the event — the
confirmation latency, not the duration of the episode. The event is emitted near
the episode's start and is never revised when it ends, so no field carries the
total. A consumer needing episode length must derive it from the estimate
stream. Temporal resets clear an open episode, so continuity is never asserted
across a clock or profile discontinuity.

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

The synthetic-clock integration test produced a violation stamped at 4.368859941
seconds on the generated timeline, rather than host wall time. Its camera-derived
speed was 1.7858 m/s for a 1.8 m/s truth input.

## Message timing sequence

```mermaid
sequenceDiagram
    participant C as Clock source
    participant A as A camera publisher
    participant P as police_observer
    participant U as Result consumer
    participant H as Test truth source
    participant E as Test evaluator

    C-->>P: /clock advances simulation time
    A-->>P: Image(header.stamp = t, frame = optical)
    A-->>P: CameraInfo(header.stamp = t, same frame)
    Note over P: Pair by acquisition stamp<br/>callback arrival time is irrelevant
    P->>P: Detect markers and estimate station
    A-->>P: Next paired frame at t + Δt
    P->>P: Interpolate gate crossing and speed
    P-->>U: SpeedEstimate(measurement time)
    opt Conservative limit exceeded long enough
        P-->>U: SpeedViolation(event)
    end
    P-->>E: Camera-derived result
    H-->>E: Ground-truth speed for error measurement
    Note over P,H: No truth message or transform is sent to P
```

## Forbidden dependencies

Production observer code and launch files must not subscribe to:

- `/ground_truth/*` or `/test/ground_truth/*`;
- pose or model-state topics from a simulator;
- odometry used as a speed shortcut;
- a TF transform whose source is simulated robot truth.

Since ADR 0020 there is a second, independent barrier: every one of those
producers is on the robot domain and none is on the gateway allowlist, so an
observer that tried to subscribe would find nothing to subscribe to. The rules
above are kept rather than replaced. They still catch the case the transport
cannot — code added on the *wrong side* of the boundary, where the topic is
reachable — and a boundary defended one way is a boundary that fails silently
when that one way is misconfigured.

Adding a topic to the allowlist is therefore a contract change, not a
configuration tweak: it widens what P can observe, and
`src/corridor_gateway/test/test_gateway_config.py` fails until the change is
made deliberately in both the configuration and the restated expectation.

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

The live external probe passed against Isaac Sim 5.1 in both headless and visible
modes on 2026-07-26. Each run observed 12 exactly timestamp-paired RGB
`Image`/`CameraInfo` messages at 640×360 and 15.000 Hz, a monotonic simulation
clock that reached the image stamps, valid zero-distortion pinhole intrinsics,
the required optical frame, best-effort/volatile publishers, and exactly one
publisher on each of the three input/time topics.
