# Live crossing measurement and isolation certificate — v2 plan T2.2 / T2.3

The 42 → 43 crossing under the recast v2 rules, with the delivery gate
decomposed into a producer gate and a crossing gate, and the isolation
certificate taken from inside P's plane.

**All four gates are green for the pinned 640×360 configuration.** 1280×720
fails the crossing gate and is recorded as a throughput verdict, not a session
failure.

| Gate | 640×360 | 1280×720 |
|---|---|---|
| Producer (rendered vs declared) | **PASS** 0.9995 | **PASS** 0.9995 |
| Crossing (delivered vs published) | **PASS** 0.954 | **FAIL** 0.926 |
| Isolation certificate | **GREEN** | not re-run (config unchanged) |
| Mutation control | **RED** as required | — |

## Environment

| Field | Value |
|---|---|
| Date | 2026-08-11 |
| Isaac Sim | 5.1.0.0 (`~/isaac/env_isaaclab`, Python 3.11) |
| GPU | NVIDIA GeForce RTX 5070 Ti, driver 580.173.02, 16303 MiB |
| ROS | Jazzy; measurement on system Python 3.12 |
| Domains | A = 42, P = 43, one-way `domain_bridge` |
| Drive | 0.35 m/s, so the ~24.6 m route outlives the 60 s capture window |

```bash
bash tools/crossing_session.sh --label 640x360
bash tools/crossing_session.sh --label 1280x720 --camera-resolution 1280x720 --certificate no
python3 tools/rate_basis_analysis.py --out out/evidence/crossing/rate-basis.json
```

## A correction to the previous measurement

The earlier capture reported "the binding constraint is the publisher, not the
crossing", from a publisher rate of 12.93 Hz against a declared 15. **That was
an artifact of the measuring instrument and is retracted.**

`rate-basis.json` is the analysis that caught it. The real-time factor was 0.974,
so simulation time explained ~2.5 % of a ~14 % gap — but CameraInfo and Image
leave the same render product on the same tick, and on a simulation-time basis
CameraInfo read 14.39 Hz while Image read 12.93 Hz. The only property separating
the two streams is message size, and both subscribers are BEST_EFFORT out of
necessity, because the publisher offers BEST_EFFORT and a RELIABLE subscriber
matches nothing.

Three independent measurements now agree the adapter renders on rate:

| Method | DDS in the path? | 640×360 | 1280×720 |
|---|---|---|---|
| Adapter's own `--drive-out` schedule | no | **14.993 Hz** | **14.993 Hz** |
| CameraInfo tap on A's plane | yes, small messages | 15.021 Hz | 15.0 Hz |
| Image tap on A's plane | yes, ~691 kB–2.7 MB messages | 13.794 Hz | 10.8 Hz |

The first two agree to 0.2 %. The third is the one that moves with resolution,
which is what a size-dependent transport loss looks like and what a producer
shortfall does not.

## Producer gate — **PASS at both resolutions**

Defined as frames the adapter rendered per simulation second, from its own
schedule, against the declared rate. The schedule has no DDS in it: it is the
per-update simulation-time record written by the process that owns the render
loop, so a subscriber that drops frames cannot depress it.

4219 updates over 70.3 simulation seconds, divider 4 → 1054 frames → **14.993 Hz
against 15.0 declared, ratio 0.9995**, identical at both resolutions.

It measures intent rather than emission — a graph that silently failed to
publish a rendered frame would still appear here — which is why the CameraInfo
crossing ratio is reported beside it and neither is quoted alone.

## Crossing gate — **PASS at 640×360, FAIL at 1280×720**

Defined as delivered in P's plane vs published on A's plane, with the same QoS
and queue depth on both sides so the difference is attributable to what sits
between them. Queue depth is 200, not the contract's 5: the subscriber is an
instrument, and a queue overflowing while the thread services the other domain
would be recorded as transport loss.

| | 640×360 | 1280×720 |
|---|---|---|
| Image crossing ratio | **0.954** | **0.926** |
| CameraInfo crossing ratio | 0.993 | 0.998 |
| Attribution | size-dependent transport loss | size-dependent transport loss |

The small stream crosses essentially intact at both resolutions while the large
one degrades with size. **This is not the bridge declining to forward.** It is
best-effort delivery of large messages, and it happens on both legs; the bridge
is one of them, not the cause.

## Latency, VRAM, bridge CPU

| | 640×360 | 1280×720 | Ceiling |
|---|---|---|---|
| Added latency, median | 1.34 ms | 4.92 ms | — |
| Added latency, p95 | 3.31 ms | 8.05 ms | — |
| Added latency, max | 6.65 ms | **23.86 ms** | 66.7 ms — pass |
| VRAM peak during capture | 2874 MiB | 2915 MiB | of 16303 |
| Bridge CPU, max | 4.0 % | 8.0 % | — |

Added latency is a difference and is measured as one: the same topic is
subscribed on both domains in one process and frames matched by header stamp, so
the delta is the bridge's contribution against a single wall clock. Comparing a
header stamp to wall time would instead have measured Isaac's real-time factor.

Stamps were monotonic in P's plane at both resolutions (0 violations), and
`/clock` advanced in P's plane throughout.

## Isolation certificate — **GREEN**, mutation **RED**

| Artifact | Verdict | Unexpected topics in P's plane |
|---|---|---|
| `certificate-640x360.json` | **GREEN** | none |
| `certificate-640x360-mutated.json` | **RED** | `/test/ground_truth/speed` |

P's observed graph equalled the declared allowlist exactly, with `/clock`
present and advancing. The mutation relayed one extra A-plane topic and the
certificate went red naming it.

The green counts only because something was available to leak:
`tools/truth_source.py` publishes `/test/ground_truth/speed` on A's plane,
unbridged, throughout the certificate phase. Without it an adapter-only session
publishes nothing on A's plane except the allowlist itself, and the first
attempt certified `INCONCLUSIVE` for exactly that reason.

## The 1280×720 verdict

**The ceiling is the transport, not the renderer.** The adapter rendered 720p at
the same 14.993 Hz, VRAM rose only 41 MiB, and latency stayed far under one
camera period — but the image crossing ratio fell to 0.926, below the 0.95
floor, while CameraInfo crossed at 0.998.

640×360 stays pinned for the crossing. No resolution is chosen here; ADR 0024
decides the v2 resolution, and if it wants 720p the transport question has to be
answered first (larger DDS buffers, a compressed transport, or a reliability
change — none of which is attempted here).

## What this does not show

- Nothing about P's camera **placement**. The camera still sits on A's old mount
  and aim; only naming and ownership moved.
- No estimator, autonomy, or detector claim.
- The certificate ran with the adapter and one truth publisher live. A fuller
  A-plane (lidar, odometry, TF, Nav2) is a stronger test and has not been run.
- The producer gate measures render-loop intent; a graph-level publish failure
  would need the CameraInfo ratio to catch it.
