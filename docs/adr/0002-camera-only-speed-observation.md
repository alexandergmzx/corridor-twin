# ADR 0002: Derive speed only from camera evidence

- Status: Accepted
- Date: 2026-07-24

## Context

P cannot see A directly but may read A's camera. Reading simulator pose or
odometry would bypass the central perception problem.

## Decision

Detect surveyed ArUco markers in camera images, estimate camera station from
marker corners and calibration, interpolate gate-crossing times, and derive
speed from surveyed distance divided by acquisition-time difference.

## Consequences

- The observer depends on image quality, calibration, marker geometry, and
  timestamps.
- Synthetic truth is available only to an external test harness.
- Flush wall markers may have poor perspective, so small canted plates are part
  of the scene design.
- Uncertain or insufficient observations produce no violation.

## Alternatives considered

- Simulator pose, odometry, or TF: rejected as ground-truth shortcuts.
- First/last marker detection timestamps: rejected because visibility timing
  changes with FOV and detection threshold.
- Optical flow alone: rejected because metric scale and drift are less explicit
  than surveyed fiducials for this demo.
