# `tools/demo.sh enforce` reproduces the recorded F3.1 pass

The entry point was built to close three defaults that produce a complete run of
the wrong scenario. This is the check that it produces the **right** one: a
single live pass, driven by the wrapper rather than by a hand-assembled
environment, compared against the run every figure in `DELIVERY.md`'s speed
table came from.

## Command

```bash
bash tools/demo.sh enforce
```

That is the whole invocation. It resolved `--robot robot1`, the
`nominal_m6_n3` profile, the composed arena, and the six-variable F3.1
environment itself, and it acquired and released the machine-wide Isaac lock —
which `tools/run_demo.sh` does not take on its own.

## Environment

| Field | Value |
|---|---|
| Date measured | 2026-08-17, 06:16–06:17 CST |
| Host | Linux Mint, RTX 5070 Ti (16303 MiB) |
| Isaac Sim | 5.1, headless, `RaytracedLighting`, no path tracing |
| Stage | `out/arena_corridor_robot1_nominal_m6_n3.usd` (composed arena, **not** the v1 stage) |
| Resolution / rate | 640×360, one render product |
| Domains | A on 42, P on 43; bag recorded **from domain 43** |
| Artifacts | `out/evidence/demo/nominal_m6_n3-enforce/` (bulk, gitignored) |

## Result: PASS — matches the recorded run

| | ship-day F3.1 (recorded) | this run |
|---|---|---|
| Route | 7.380 m, `reached_end=True` | **7.380 m, `reached_end=True`** |
| Sim span | 33.55 s | **33.550 s** |
| `/p_cam/image_raw` delivered to P | 376 | **371** |
| `/clock` | 2057 | **2058** |
| VRAM | 3313 MiB | 3254 MiB |
| Render products | 1 | **1** |
| Commanded speed | 0.22 m/s | **0.22 m/s** |

Route length and sim span are identical to three decimal places. The image count
differs by 5 frames (1.3%) and the bag reports 83 messages lost on the transport
layer, which is ordinary recorder loss and is recorded here rather than smoothed
away.

The renderer mode is **read back, not requested** —
`active_render_mode='RaytracedLighting'` beside
`default_render_mode='RaytracedLighting'`, with `reset_events=0`. That
distinction is the one the invalidated v1 static qualification got wrong.

## What this run does NOT show

- **`/police/speed_estimate` and `/police/speed_violation` are both 0**, and that
  is expected, not a regression. `police_observer` is the v1 ArUco-fiducial
  estimator; v2's speed table and violation verdicts are produced **offline**
  from this bag by the learned detector (`tools/p_cam_infer.py`, ADR 0024). This
  run's job is to produce the bag that path consumes. No live violation is
  claimed here.
- **No speed or violation figure is quoted from this run.** It is a
  reproduction check on the entry point, not a re-measurement of the estimator.
  The estimator's numbers remain those in
  [`docs/evidence/estimator/`](../estimator/NOTES.md).
- **One profile, one pass.** `nominal_m6_n3` only.

## Files here

| File | What |
|---|---|
| `isaac-markers.txt` | the four adapter markers: renderer readback, drive completion, VRAM, render-product count |
| `rosbag-info.txt` | per-topic message counts, proving what crossed the gateway to P's plane |
| `gateway.log` | the domain bridge coming up as the only participant in both domains |

The 245 MiB bag itself stays under `out/evidence/` and is not promoted.
