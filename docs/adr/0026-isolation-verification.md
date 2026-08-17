# ADR 0026: Verify the committed domain-isolation mechanism under the recast crossing

- Status: Accepted
- Date: 2026-08-11
- Source: live measurement on the RTX 5070 Ti, Isaac Sim 5.1.0.0, recorded in
  [`docs/evidence/crossing/NOTES.md`](../evidence/crossing/NOTES.md) with the
  machine-readable artifacts beside it.
- Verifies [ADR 0020](0020-communication-domain-isolation.md) rather than
  replacing it: 0020 chose and shipped the mechanism, and recorded that it
  "changes no measured result". This record supplies the measurement.
- Operates under [ADR 0021](0021-police-owned-sensing-and-isolation-gate.md)'s
  recast crossing, in which the relayed feed is P's own enforcement camera.

## Context

ADR 0020 put A and P on separate ROS domains and shipped a one-way
`domain_bridge` allowlist, but it was a *design* record: nothing had been run.
ADR 0021 then made the isolation certificate the requirement gate — the
assignment's "the robot cannot see the traffic police" is a communication-domain
statement, not a sightline — which left the project's headline claim resting on
an unmeasured mechanism.

This record is therefore a **verification ADR**, not a mechanism choice. The
decision it commits to is that the committed mechanism is adopted under the
recast crossing *because it was measured*, together with the gate definitions
that measurement had to be decomposed into before it meant anything.

## Decision

**Adopt the committed ADR 0020 mechanism unchanged**, verified by four gates,
all green for the pinned 640×360 configuration:

| Gate | Definition | Result |
|---|---|---|
| Producer | Frames the adapter rendered per simulation second, from its own schedule, vs the declared rate | **PASS** 14.993 Hz vs 15.0, ratio 0.9995 |
| Crossing | Delivered in P's plane vs published on A's plane, same QoS both sides | **PASS** 0.954 (floor 0.95) |
| Certificate | P's observed graph equals the declared allowlist exactly, `/clock` advancing | **GREEN** |
| Mutation | One extra A-plane topic relayed; certificate must go red | **RED**, naming `/test/ground_truth/speed` |

The allowlist verified is exactly `/p_cam/image_raw`, `/p_cam/camera_info`,
`/clock`, one way, 42 → 43.

## Why the delivery gate had to be decomposed

The v2 plan set a single gate: delivery ≥ 95 % of nominal over 60 s. That number
cannot fail informatively, because it mixes three unrelated things — how fast the
adapter rendered, how much the transport lost, and how long the source lived —
and it produced two wrong conclusions before it was split:

1. **0.37 of nominal**, because the authored route finishes in ~24 s and the
   window was 60 s. The bridge had carried 95.7 % of everything published.
2. **0.79 of nominal with a "publisher under-runs at 12.93 Hz" finding.** That
   was the measuring instrument. CameraInfo and Image leave the same render
   product on the same tick and differ only in size; on a simulation-time basis
   CameraInfo read 14.39 Hz and Image read 12.93 Hz, at subscribers that are
   BEST_EFFORT out of necessity because the publisher offers BEST_EFFORT.

So the gate is now two gates, each separately actionable:

- The **producer gate** reads the adapter's own `--drive-out` schedule. No DDS
  is in that path, so a subscriber that drops frames cannot depress it. It
  measures render-loop *intent*; the CameraInfo crossing ratio is reported
  beside it to cover a graph-level publish failure, and neither is quoted alone.
- The **crossing gate** compares P's plane against A's plane with identical QoS
  and queue depth, so the difference is attributable to what sits between them,
  and counts CameraInfo on both planes as a size-independent control.

Three methods now agree the adapter renders on rate: its own schedule at
14.993 Hz, a CameraInfo tap at 15.021 Hz, and the image tap at 13.794 Hz — and
only the third moves with resolution.

## What the crossing gate does and does not attribute

At 640×360 the small stream crossed at 0.993 and the large stream at 0.954; at
1280×720, 0.998 and 0.926. **This is recorded as size-dependent transport loss
on both legs, not as bridge fault.** The bridge is one leg of a best-effort path
carrying 691 kB–2.7 MB messages; the small stream crossing intact at both
resolutions is what rules out a bridge that declines to forward.

An instrument that cannot drop would be needed to attribute the remaining
few percent precisely. This record does not claim to have one, and makes no
claim about the bridge's intrinsic loss rate.

## The 1280×720 verdict

**720p fails the crossing gate at 0.926 and is not adopted.** The ceiling is the
transport, not the renderer: the adapter rendered 720p at the same 14.993 Hz,
VRAM rose 41 MiB to 2915 MiB of 16303, latency stayed far under one camera
period, and only the large-message crossing ratio fell.

640×360 stays pinned for the crossing. This record does **not** choose the v2
resolution — [ADR 0024](0024-learned-enforcement-perception.md) owns that — and
if 0024 wants 720p, the transport question must be answered first (larger DDS
buffers, a compressed transport, or a reliability change; none attempted here).

## Real-time factor

0.974 and 0.978 across the two captures. Recorded as a performance metric, not a
contract breach: the contract is stated in simulation time and the adapter meets
it there. Added latency, measured as a per-frame difference between the two
planes against one wall clock, was 1.34 ms median and 6.65 ms worst case at
640×360 against a 66.7 ms ceiling — an order of magnitude of headroom.

## Alternatives considered

**Dual `ROS2Context` on one domain — viable, not pursued.** The Isaac ROS 2
bridge exposes per-graph context nodes, so A's and P's graphs could each hold
their own context and be separated without a second domain (v2 plan F31). It is
rejected here not because it fails but because it moves the boundary *inside one
process*: the isolation would then rest on graph wiring that no external
instrument can introspect, and the certificate above — a node standing in P's
domain enumerating its own graph — would have nothing to stand in. A boundary
that cannot be independently observed cannot be certified, and this project's
requirement gate is the certificate.

**Per-robot domain isolation across the fleet — rejected there, and that
rejection stands.** The fleet's R-06 keeps all robots on `ROS_DOMAIN_ID=20`,
separated by namespace, because they share one map and must see each other's
topics. That reasoning is sound and is not disturbed: the corridor's split is
between *a robot and an observer that is required not to see it*, which is the
opposite requirement. Fleet ledger D-20 records the corridor's 42/43 as a scoped
exception to D-09 for exactly this reason.

## Consequences

- The v2 requirement gate has live evidence, and a mutation control proving the
  instrument detects a leak rather than merely never having seen one.
- The certificate is only meaningful with a non-allowlisted topic live on A's
  plane. `tools/truth_source.py` exists for that, and a run without it certifies
  `INCONCLUSIVE` by design rather than green.
- 640×360 is pinned for the crossing until ADR 0024 revisits resolution.
- The producer gate depends on `--drive-out`, so a run without it defers that
  gate rather than failing it.
- **Not verified here:** P's camera placement (still A's old mount), the
  estimator, autonomy, and the detector. The certificate ran against an A-plane
  carrying the adapter and one truth publisher; a fuller A-plane with lidar,
  odometry, TF and Nav2 is a stronger test and remains unrun.
