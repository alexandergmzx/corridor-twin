# ADR 0013: Size and mount fiducials from the delivered camera

- Status: Accepted
- Date: 2026-07-27
- Extends: [ADR 0002](0002-camera-only-speed-observation.md),
  [ADR 0006](0006-scenario-manifest.md), and
  [ADR 0009](0009-installed-isaac-ros-camera-adapter.md)

## Context

The simulator-free renderer proved the estimator logic, survey convention, and
camera contract, but it could not prove that the production Isaac renderer
delivered readable wall markers. The first static GPU capture detected no IDs.

Two physical defects were visible only in the production pixels:

1. the 24 cm codes occupied too few pixels per module in the required oblique
   640x360 views; and
2. a canted plate was centered only 15 mm from its wall, so roughly half of the
   plate intersected the opaque building mesh.

The initial textures also ended at the black code border. ArUco detection needs
a contrasting quiet zone around that border. Increasing camera resolution or
adding a second camera would consume the resource budget without correcting the
buried-plate geometry.

The installed renderer also exposed a configuration lesson. Isaac Sim 5.1
materializes the Hydra render product during playback warm-up and its active
real-time super-resolution mode is DLSS, enum value 3. Evidence must record the
mode that remains active after product creation, not merely a requested startup
value.

## Decision

Keep one 640x360 RGB render product at 15 Hz and make the surveyed target fit
that delivered sensor:

| Property | Accepted value |
|---|---:|
| ArUco code size | 0.40 m |
| White backing size | `9/7` of the code in each dimension |
| Plate cant | 35 degrees from the local corridor-facing wall normal |
| Minimum backing-to-wall clearance | 0.015 m |
| Backing depth behind code | 0.002 m |

For each wall, compute its actual local corridor-facing unit normal. Rotate that
normal toward the approach by the cant angle, then solve the bracket standoff
so the nearest corner of the complete white backing remains at least 15 mm on
the corridor side of the wall. The manifest continues to survey the black code
corners in OpenCV order; the white backing is physical contrast, not part of the
metric code size.

The static Isaac gate discards 12 product/shader warm-up updates. It verifies
the active and default anti-aliasing/super-resolution enum after warm-up and at
every dwell. The qualified installed build consistently reports value 3. The
gate then evaluates actual ROS `Image`, `CameraInfo`, and `/clock` messages; the
commanded camera pose remains in a separate file-only evaluator.

## Consequences

- All plate corners are regression-tested against the straight north face and
  the tapered south face for every corridor variant.
- The authored USDA visibly contains a white quiet-zone plate behind each code.
- The nominal static production-camera gate passes all five required world-X
  dwells. Maximum station error is 0.0106 m, maximum corner RMSE is 1.5501 px,
  and maximum estimator reprojection RMSE is 1.0917 px.
- A horizontal mirror applied to the same captured frames produces no passing
  dwell and satisfies the required failing control.
- The final one-product headless run uses 3,024 MiB, still far below the 14,336
  MiB soft ceiling.
- The larger targets are a deliberate sensor-design choice, not a claim that
  the supplied diagram specifies their metric size.

## Alternatives considered

- **Increase camera resolution.** Rejected for this milestone: the 640x360
  contract is already adequate once the physical target is correctly mounted,
  and the smaller feed preserves GPU headroom.
- **Add another camera or depth sensor.** Rejected: it weakens the intentionally
  small architecture and is unnecessary for the demonstrated measurement.
- **Tune the detector until partial tags decode.** Rejected: detector tolerance
  cannot make an opaque wall intersection into a valid surveyed target.
- **Author a white border inside the 0.40 m survey.** Rejected: that would shrink
  the metric ArUco code while pretending its surveyed corner size was unchanged.
