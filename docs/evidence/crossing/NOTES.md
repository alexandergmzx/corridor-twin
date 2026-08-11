# Live crossing measurement and isolation certificate — v2 plan T2.2 / T2.3

First live measurement of the 42 → 43 crossing under the recast v2 rules, and
the first isolation certificate taken from inside P's plane.

**T2.3 passed. T2.2's delivery gate did not.** ADR 0026 is therefore not
written: the v2 plan makes it Accepted only when every T2 gate is green.

## Environment

| Field | Value |
|---|---|
| Date | 2026-08-11 |
| Isaac Sim | 5.1.0.0 (`~/isaac/env_isaaclab`, Python 3.11) |
| GPU | NVIDIA GeForce RTX 5070 Ti, driver 580.173.02, 16303 MiB |
| ROS | Jazzy; observer/measurement on system Python 3.12 |
| Domains | A = 42, P = 43, one-way `domain_bridge` |
| Drive | 1.0 m/s finishes the authored route in ~24 s, so 0.35 m/s was used to keep the source alive across the 60 s window |

```bash
bash tools/crossing_session.sh --label 640x360
bash tools/crossing_session.sh --label 1280x720 --camera-resolution 1280x720 --certificate no
```

## T2.3 — isolation certificate: **GREEN**, mutation **RED**

| Artifact | Verdict | Unexpected topics in P's plane |
|---|---|---|
| `certificate-640x360.json` | **GREEN** | none |
| `certificate-640x360-mutated.json` | **RED** | `/test/ground_truth/speed` |

P's observed graph equalled the declared allowlist exactly —
`/p_cam/image_raw`, `/p_cam/camera_info`, `/clock` — with `/clock` present and
advancing. The mutation relayed one extra A-plane topic and the certificate
went red naming it, so the instrument is shown to detect a leak rather than
merely never having seen one.

**The green only counts because something was available to leak.** The first
attempt returned `INCONCLUSIVE`: an adapter-only session publishes nothing on
A's plane except the allowlist itself, so "P sees exactly the allowlist" was
trivially true with nothing to hide. `tools/truth_source.py` now publishes
`/test/ground_truth/speed` on A's plane, unbridged, for the whole certificate
phase — simulator truth, the thing truth-isolation forbids reaching P. The
certificate records that it was live on 42 and absent from 43.

## T2.2 — crossing measurement: **FAIL on delivery**, pass on everything else

| Measure | 640×360 | 1280×720 | Gate |
|---|---|---|---|
| Publisher rate while alive | **12.93 Hz** | **9.39 Hz** | 15 Hz declared |
| Delivered / nominal | **0.790** | **0.576** | ≥ 0.95 — **FAIL** |
| Delivered / published | **0.941** | **0.940** | ≥ 0.95 — **FAIL** |
| Added latency, median | 1.34 ms | 5.26 ms | — |
| Added latency, p95 | 3.31 ms | 8.41 ms | — |
| Added latency, max | 6.65 ms | 17.5 ms | < 66.7 ms — **pass** |
| Stamp monotonicity (P's plane) | 0 violations | 0 violations | pass |
| `/clock` advancing in P's plane | yes, 58.5 s span | yes, 58.8 s span | pass |
| VRAM peak during capture | 2874 MiB | 2934 MiB | of 16303 |
| Bridge CPU, max | 4.0 % | 8.0 % | — |

Added latency is a *difference*, so it is measured as one: the tool subscribes
to the same topic on both domains in one process and matches frames by header
stamp, so the delta is the bridge's contribution against a single wall clock.
Comparing a header stamp to wall time would instead have measured Isaac's
real-time factor and reported it as transport delay.

### Why delivery fails, and where it does not fail

**The binding constraint is the publisher, not the crossing.** At 640×360 the
adapter emitted 12.93 Hz against a declared 15 Hz — 86 % of nominal — so
`delivered / nominal` is capped at 0.86 before any transport is involved and
cannot reach 0.95 no matter what the bridge does.

The bridge's own fidelity is 94.0–94.1 % at both resolutions, just under the
same floor. **That figure is a lower bound, not a measurement of bridge loss.**
Both measurement subscribers are BEST_EFFORT — forced, since the publisher
offers BEST_EFFORT and a RELIABLE subscriber would match nothing — so frames
dropped by the measuring node are indistinguishable from frames dropped by the
bridge. Corroborating detail: small `CameraInfo` messages arrived 843–844 times
while large `Image` messages arrived 518–747 times in the same windows, which is
the signature of large-message loss somewhere on the path, not of a bridge that
refuses to forward.

Separating the two needs an instrument that cannot drop, which this one is not.
**No claim is made here that the bridge loses 6 % of frames.**

### The 1280×720 trial

The throughput ceiling is **the renderer, not the crossing**. At 720p the
publisher fell from 12.93 to 9.39 Hz while bridge fidelity was unchanged
(0.940 vs 0.941) and latency, though ~4× worse, stayed far under one camera
period. VRAM rose only 60 MiB; bridge CPU doubled from 4 % to 8 %.

Per the plan, a 720p shortfall is not a session failure and no resolution is
chosen here. 640×360 stays pinned for the crossing. ADR 0024 decides the v2
resolution.

**The first 720p attempt was invalid and is not published.** It was run by
building a stage with a 1280×720 camera block, but the adapter takes its
resolution from a hardcoded `ADAPTER_CONTRACT`, not from the manifest, so it
published 640×360 under a `1280x720` label. It was caught by checking the
delivered frame sizes rather than the label. `--camera-resolution` was added to
the adapter so a trial resolution can be measured without editing the contract;
the numbers above are from a run whose delivered frames really are 1280×720.

## What this does not show

- Nothing about P's camera **placement**. The camera still sits on A's old mount
  and aim; only naming and ownership moved. Placement is a later task.
- No resolution or rate decision. The 720p row is a ceiling input for ADR 0024.
- Nothing about the estimator, autonomy, or the learned detector.
- The isolation certificate was taken with the adapter and one truth publisher
  live. A fuller A-plane (lidar, odometry, TF, Nav2) is a stronger test and has
  not been run.
