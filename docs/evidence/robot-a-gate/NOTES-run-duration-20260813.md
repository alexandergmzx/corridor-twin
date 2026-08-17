# What the run costs, measured — and the part that cannot get shorter yet

**2026-08-13.** Baseline `20260813-000546`, after `20260813-015009`. Both
`nominal_m6_n3`, gated, dock off, robot1, domain 67.

Until tonight the wall clock had to be reconstructed from artifact mtimes,
because `runner.log` carried no time anywhere. Every phase banner now prints a
clock and an elapsed count and appends to `phases.log`, so this table is read
off the run rather than inferred:

```
01:50:09 +0s    corridor profile run: nominal_m6_n3 (GATED)
01:50:10 +0s    simctl start
01:51:03 +53s   precondition: robot1 contract (--seconds 8)
01:51:27 +77s   waiting for the TF chain
01:51:32 +82s   nav stack
01:51:39 +89s   transit recorder + governed Nav2 goal
01:55:03 +293s  transit recorder verdict
01:56:07 +357s  map quality
                run.json at +359s
```

## The result

| | baseline `000546` | after `015009` | |
|---|---|---|---|
| **total** | **403 s** | **359 s** | −44 s (−11%) |
| start → goal sent | 144 s | **89 s** | **−55 s (−38%)** |
| goal → nav gate returns | 201 s | 204 s | unchanged |
| scoring + teardown | 58 s | 66 s | +8 s |

**Bring-up is 38% faster** and that is where every saving came from: the
contract check dropped 30 s (it drives nothing, it is a rate report, and its
verdict is overridden on every run), the lifecycle polls detect in 1 s instead
of 5, and bt_navigator came up on the first attempt in 7 s by watching its
launch log rather than asking a daemon-backed CLI.

## The part that did not get shorter, and why

`gate.json` reads `observed_s: 210.0` — the recorder ran its **entire** window.
The early-finish added this session did not fire, and it could not have:

```
nav.json: goal_accepted true, travelled_m 7.738,
          failure "no action result within 200.0 s"
```

**Nav2 never returned a result.** The goal was accepted on the first send and A
drove its 7.7 m, but the arrival gate is a map-frame goal that the diverging
map never satisfies (ADR 0028, ADR 0029), so there is no completion event for
the recorder to finish on. The window is consumed by definition.

That is the honest bound on this work: **the transit cannot shorten until the
arrival gate is green.** The recorder change is still correct — it TERMs a
recorder that outlives a gate which has returned, and `observe()` writes a
complete report on SIGTERM — but it is dead code in this failure mode and it is
recorded as such rather than credited with a saving it did not make.

Deliberately not done, with the reason: `TRANSIT_WINDOW_S` stays at 200 s. The
measured worst closest-approach is 109.9 s, so 130 is a real evidence
trade-off, not free. `SIM_MAX_S` stays at 600 until there is a new distribution
to size it against; it also feeds the derived nav window.

## The gate, this run

| metric | value | bound | |
|---|---|---|---|
| yaw scale | **0.9454** | 1.0 ± 0.1 | pass |
| duplicate wall | **0.340 m** | ≤ 0.20 m | **FAIL** |
| EKF output gap | **1.028 s** | ≤ 0.4 s | **FAIL** |
| travelled | 7.738 m | — | — |
| Nav2 result | **none within 200 s** | SUCCEEDED | **FAIL** |

Duplicate wall improved from 0.720 m; yaw is inside the bound for the second
run running with the window fix. **The EKF output gap is new and getting
worse** — 0.242 s on 08-12, 0.727 s at 00:05, 1.028 s here — and it is now the
gate's own named failure. Three points is a trend worth naming and not yet a
finding worth explaining; it is not investigated here.

## What the lens showed, which no artifact did

The first live look at a corridor run since the lens was written
([the render loop had never survived frame one](../lens/NOTES.md)):

- [`transit.png`](../lens/live-run/transit.png) at t=23 s — corridor mapped,
  scan endpoints on the walls, **pose-vs-truth already 1.42 m and the yaw tile
  reading 3.78×** with the SLAM pose visibly separated from the truth ghost.
- [`corner.png`](../lens/live-run/corner.png) at t=213 s — **ADR 0029's map
  divergence, drawn**: the corridor mapped twice at an angle to itself,
  `scan→map fit` collapsed to **0.01**, pose-vs-truth 2.45 m, and B's marker
  outside the map entirely. A confirmed landmark sits ~2.5 m from where B is.

That last image is the duplicate-wall metric as a picture. It is not a new
finding — ADR 0029 measured and named it — but it is the first time the failure
has been *watched* rather than scored afterwards, and it took seven seconds to
capture.
