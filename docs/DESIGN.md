# corridor-twin system design

| Field | Value |
|---|---|
| Design version | 0.3.0 |
| Status | Phase 1 implemented; RTX 5070 Ti qualified; Isaac camera bridge pending |
| Last updated | 2026-07-26 |
| Target ROS | ROS 2 Jazzy |
| Target host | Linux Mint 22 demo host / Ubuntu 24.04 supported fallback |
| Current / target GPU | RTX 5070 Ti, 16 GB |

## Purpose

Demonstrate a small, explainable robotics digital twin in which A delivers a
package to B while P observes speed indirectly through A's camera stream. The
system must separate authored scene truth, simulated sensor data, and the
observer's permitted evidence.

This is an interview artifact, not a production traffic-enforcement system.

## Design principles

1. **Interface first.** The observer consumes standard camera messages and does
   not know whether their publisher is synthetic, Isaac Sim, or future hardware.
2. **Truth isolation.** Simulator pose and synthetic truth are evaluation inputs,
   never observer inputs.
3. **Deterministic authoring.** The USDA and its manifest are generated from
   versioned configuration with `pxr`; no GUI edits are required to reproduce it.
4. **Independent evidence.** Validators inspect the composed USD rather than
   trusting generator metadata.
5. **Small GPU surface.** One modest RGB camera and primitive geometry are enough
   to demonstrate the system claim.
6. **Installed-version APIs.** Isaac namespaces are isolated and committed only
   after checking the installed documentation and examples.

## Component boundaries

```text
Scenario description
  ├─> OpenUSD authoring ─> USDA + sidecar manifest
  ├─> synthetic renderer ─> Image + CameraInfo + harness-only truth
  └─> occlusion verifier ─> pass/fail certificate

Image + CameraInfo + manifest
  └─> police observer
        ├─> fiducial detections
        ├─> camera station observations
        ├─> gate-crossing speed estimates
        └─> debounced violation events

Isaac 5.1 smoke tool (installed-version API)
  └─> composes USDA and checks profiles, camera count, and colliders

Future narrow Isaac/ROS adapter
  └─> publishes the same camera contract
```

`corridor_scene` and the perception core must remain importable without Isaac
Sim. The ROS adapter may import ROS packages but must keep perception algorithms
free of ROS node state so they can be tested with ordinary arrays and timestamps.

## Scenario and coordinate contract

- Stage units: meters (`metersPerUnit = 1.0`).
- Up axis: Z.
- Corridor station: distance along the authored delivery centerline.
- Map frame: `corridor_map`.
- Camera optical frame: X right, Y down, Z forward, following ROS camera
  conventions.
- A, B, P, and the full path are not frozen until the supplied diagram is added.

The current draft assumes a 12 m corridor and provides three named profiles. The
numbers are configuration defaults, not recovered facts from the absent diagram.

## Corridor variants

The root scenario prim owns one `corridorProfile` variant set. Each variant is a
complete, named `(entry width m, corner width n)` profile. The selected variant
authors numeric width attributes and all dependent wall/marker geometry.

The CLI `--m` and `--n` values define and select the nominal requested profile.
Additional named profiles come from configuration. Variants are deliberately not
described as continuous numeric parameters.

## Generated artifacts

`scene.build` writes:

- a human-readable `.usda` stage;
- small ArUco texture files where required;
- `corridor.manifest.json`, containing schema version, profile definitions,
  marker corner coordinates, camera model, path stations, P bounds, and the
  demonstration speed policy.

The build is deterministic for equivalent arguments. Generated outputs live
under `out/` and are not source-controlled by default.

## Stage contents

Stable prim paths:

```text
/World
  /PhysicsScene
  /Environment
    /Ground
    /CrossStreet
    /Corridor
      /LeftBuilding
      /RightBuilding
      /Fiducials
  /Actors
    /A
      /CameraMount
        /FrontCamera
    /B
    /P
  /Paths
    /DeliveryPath
```

Buildings are closed low-poly volumes, not zero-thickness display faces. Ground
and buildings are static colliders. Applying collision without a parent rigid
body intentionally produces a static collider.

## Camera-derived speed

Markers have unique IDs, known metric size, and surveyed 3D corners. Plates are
canted toward the corridor approach so a forward camera does not view markers at
near-edge-on angles.

For each image:

1. Validate image and matching calibration stamps/frames.
2. Detect marker IDs and subpixel corners.
3. Reject unknown IDs, border-clipped markers, and high reprojection error.
4. Estimate camera pose/station from marker-to-image correspondences.
5. Retain the previous `(image acquisition time, station, uncertainty)` sample.
6. Interpolate the time at which the pair crosses each surveyed gate.
7. Estimate average speed between gates; optionally publish a robust rolling
   station-vs-time slope for diagnostics.
8. Compare a conservative confidence bound to the configured local limit.

A violation requires a valid estimate and configurable confirmation. Arrival time
at the subscriber is never used for speed.

## Speed-limit policy

Corridor width alone does not define a legal limit. The manifest contains an
explicitly provisional, demonstration-only piecewise policy. It must be approved
before the interview geometry is frozen. The event records the policy profile,
width, limit, measured speed, uncertainty, gates, and confirmation duration.

## Synthetic validation

The synthetic publisher renders actual ArUco images through a calibrated pinhole
camera model while moving it along a known station function. Truth is published
on a test-only topic for the evaluator. A source contract test proves that the
observer adapter contains no truth or odometry subscription.

The implemented clean synthetic test uses actual ArUco pixels, camera intrinsics,
detection, PnP, gate interpolation, and the violation debounce. At 1.8 m/s it
measured 1.8016 and 1.8026 m/s (maximum absolute error 0.0026 m/s) and emitted
one event; at 1.0 m/s it emitted no event. Noise, blur, dropped-frame, and
acceleration cases remain extensions rather than claimed coverage.

A live ROS 2 launch was also exercised in both wall-time and synthetic-clock
modes. The latter produced the same 1.8026 m/s event at simulated time 4.3306 s,
confirming that the observer uses image acquisition stamps rather than callback
arrival time.

## Occlusion verification

The primary certificate is continuous over every authored path segment, not a
dense sample claim. Before the turn it finds interval witnesses at which every
camera-to-P-box ray crosses the south wall in both horizontal and vertical
extent. Once P lies wholly outside one strict camera-frustum half-space, the
certificate records frustum exclusion. Together these cover the whole route.

An independent diagnostic then reads composed world-space meshes from the USD and
performs segment/triangle intersections. A negative test moves P into the clear
corridor and must fail. Camera orientation may provide an additional frustum
check, but off-axis placement alone does not count as occlusion.

## ROS time model

- Image `header.stamp` is acquisition/simulation time and is the only estimator
  timebase.
- Isaac mode has exactly one simulator `/clock` publisher and every participant
  sets `use_sim_time=true`.
- Wall-time mode has no `/clock` publisher and uses `use_sim_time=false`.
- Synthetic-clock mode has exactly one harness clock source.
- Zero time, backward jumps, profile changes, or non-monotonic image stamps clear
  estimator and debounce state.

## GPU and performance budget

The installed 16 GB RTX 5070 Ti is the qualified demo GPU. Its capacity changes
the activation status, not the deliberately small system boundary. Initial
limits remain:

| Resource | Initial budget |
|---|---:|
| RGB cameras | 1 |
| Camera resolution | 640×360 |
| Camera rate | 15 Hz |
| Other rendered sensors | 0 |
| Steady-state VRAM soft ceiling on 5070 Ti | 14 GB |
| Materials | Opaque, simple, few |
| Physics | CPU initially |
| Render mode | Real-time `RaytracedLighting`; no path tracing |

On 2026-07-26, `nvidia-smi` reported 16303 MiB and driver 580.173.02. The
installed Isaac Sim 5.1 checker passed its GPU, driver, VRAM, CPU, RAM, storage,
and display gates. The aggregate checker still failed because Linux Mint 22.3 is
unsupported; Ubuntu 24.04 remains the fallback. IOMMU is also reported as a
warning.

The project stage passed both headless and visible installed-version validation
at 640×360 with real-time `RaytracedLighting`. Total GPU memory snapshots were
916 MiB headless and 871 MiB visible, against a 468 MiB initial desktop
baseline. No sensor render product existed in these runs. The one-camera
steady-state measurement is an explicit acceptance gate for the next adapter,
not something inferred from these low composition-smoke numbers.

The earlier RTX 5060 8 GB check remains historical evidence: the checker failed
both its internal 10 GB VRAM threshold and the unsupported OS gate, although the
small stage composed. Replacing the card resolves the capacity gate but does not
make Mint a supported NVIDIA platform.

If the budget is exceeded: close redundant viewports, lower camera resolution,
reduce texture streaming budget, remove material features, and inspect render
products before changing algorithms.

## Runtime environments

### Development/ROS environment

System Python 3.12 venv with `--system-site-packages`, ROS 2 Jazzy, `usd-core`, and
test tooling. It authors and validates USD and runs ROS nodes.

### Isaac environment

Isaac Sim 5.1.0 / Isaac Lab 2.3.2 use the installed Python 3.11 environment at
`~/isaac/env_isaaclab`. It loads the already-authored stage. Pip `usd-core` is not
installed into this environment, and `omni`/`isaacsim` imports do not leak into
Phase 1 packages. The version-specific smoke code is isolated in
`tools/isaac_5_1_smoke.py`.

## Validation matrix

| Claim | Evidence |
|---|---|
| Stage is reproducible | Generator tests and readable USDA output |
| Widths equal m/n | Composed mesh measurement for every selected variant |
| Colliders exist | Applied-schema/attribute tests and Isaac 5.1 smoke |
| Observer is camera-only | Source/topic contract tests and ROS graph design |
| Estimator is accurate | Synthetic pixels measured against harness-only truth |
| P cannot be seen | Continuous certificate plus 226-ray composed-USD audit |
| Time reset behavior | Non-monotonic stamp and backward-station unit tests |
| Demo GPU is qualified | 5070 Ti checker component results, headless/GUI smoke, and measured VRAM snapshots |
| GPU constraints are explicit | One camera, 640×360, real-time rendering, no path tracing, and a 14 GB soft ceiling |

## Risks and mitigations

- **Mint is unsupported by NVIDIA:** all hardware checker components pass, but
  the aggregate compatibility check fails; retain an Ubuntu 24.04 fallback plan.
- **Exactly 16 GB target VRAM:** preserve a 2 GB soft reserve and avoid extra
  render products.
- **Variant changes versus physics caches:** pause/reset simulation when switching
  profiles until installed-version behavior is verified.
- **Wall-marker perspective:** cant plates and combine multiple markers.
- **Missing diagram values:** configuration remains explicitly draft rather than
  embedding undocumented geometry.
- **Clock discontinuity:** clear temporal state on jumps.
- **User-site NumPy 2.2 conflicts with Jazzy OpenCV:** the demo launch disables
  user-site packages; the repo venv pins NumPy below 2.

## Version history

- **0.3.0 — 2026-07-26:** Qualified the installed RTX 5070 Ti and driver with
  the Isaac 5.1 checker, fresh occlusion proof, headless stage validation, visible
  real-time viewport, and measured VRAM snapshots. Mint remains unsupported.
- **0.2.1 — 2026-07-26:** Added repeatable local environment isolation and
  verified the explicit Jazzy `/clock` QoS in a live simulated-time ROS run.
- **0.2.0 — 2026-07-26:** Implemented USD/variants/colliders, continuous
  occlusion certificate, synthetic ArUco observer, live ROS 2 validation, and an
  installed-API Isaac 5.1 smoke on the RTX 5060.
- **0.1.0 — 2026-07-24:** Initial source-only architecture, RTX 5070 Ti budget,
  interface contract, and activation gates.
