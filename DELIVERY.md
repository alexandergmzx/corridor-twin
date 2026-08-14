# corridor-twin — delivery

**2026-08-14.** An interview-sized digital twin: robot **A** delivers a package
to person **B** through a tapered corridor and around a corner, while traffic
police **P** measures A's speed from P's own roadside camera. A and P live on
separate ROS communication domains and cannot see each other's graphs.

The scenario comes from `docs/ROBO_TASK.pdf`. Its prose and topology are
authoritative; its drawing has no scale bar, so every metric dimension here is
a stated demo choice and never a surveyed value.

This document is the map. Every number in it names the artifact it came from,
and the section at the end lists what is **not** claimed.

---

## The three corrections from 2026-08-04

The task author's feedback carried three corrections. Each one has a mechanism,
an artifact, and a measurement.

### 1. "The robot cannot see the traffic police" means communication domains

**It was read as a sightline and it meant a ROS domain.** The geometric reading
was implemented, proved and is still true — an opaque wall intersects the
segment between A and P, asserted by `scene.occlusion` — but it was answering
the wrong question. [ADR 0020](docs/adr/0020-communication-domain-isolation.md)
put A on domain 42 and P on domain 43 with a one-way, allowlisted gateway
between them; [ADR 0021](docs/adr/0021-police-owned-sensing-and-isolation-gate.md)
made the **isolation certificate** the requirement gate and demoted the
sightline to scenario realism.

`CLAUDE.md` now keeps five distinct visibility concepts apart in a table,
because conflating them is what caused the original miss.

**Measured** — `docs/evidence/crossing/`:

| | |
|---|---|
| Certificate verdict | **GREEN**, P's observed graph equals the declared allowlist **exactly** |
| Mutation control | **RED** — a deliberately widened bridge is caught |
| Positive control | the robot plane shows strictly more than the allowlist, so the check can fail |
| Image crossing ratio | 0.960 |
| CameraInfo crossing ratio | 0.999 |
| Added latency | under one camera period |
| Producer gate | **15.0 Hz rendered against 15.0 declared, ratio 1.0000** |
| Clock | 713 messages, 710 distinct, advancing |

The mutation control matters more than the certificate. A green certificate
against a check that cannot go red is decoration.

### 2. A navigates autonomously

**No scripted route.** [ADR 0023](docs/adr/0023-governed-nav2-live-slam.md):
governed Nav2 on a map SLAM builds live, with a safety governor between the
planner and the wheels. A is told B's *address*, never the path
([ADR 0028](docs/adr/0028-goal-directed-navigation-on-a-live-map.md)), and the
terminal approach is a governed docking creep onto a landmark A detects for
itself ([ADRs 0031](docs/adr/0031-b-is-the-cylinder.md),
[0033](docs/adr/0033-arrival-is-contact.md),
[0034](docs/adr/0034-the-mask-is-the-target.md)).

**A physically touches B.** Run `20260814-031348` closed to **0.2146 m**
against a **0.2175 m** modelled contact — 2.9 mm *past* it — measured from
ground truth, with the governor permitting 100% of the creep.

**And the delivery does not depend on the map.** Six consecutive runs, with the
map quality varying 4.3×:

| run | duplicate wall extent | A→B |
|---|---|---|
| 085419 | 0.520 m | 0.2249 |
| 085821 | 0.700 m | 0.2263 |
| 090216 | 0.920 m | 0.2252 |
| 090613 | 0.940 m | 0.2262 |
| 093434 | **2.240 m** | 0.2284 |
| 093830 | 0.520 m | 0.2264 |

**A→B spans 3.5 mm while the map varies 4.3×.** That is the direct measurement
of what ADR 0029's geometric landmark buys: the terminal approach is closed on
something A sees, not on the map's opinion of where it is. A seventh run today,
`20260814-125254`, reached 0.2251 m — inside the same band.

### 3. Active AI/ML

**A learned detector on P's own camera, and — for the first time — a
measurement of what it is worth for enforcement.**
[ADR 0024](docs/adr/0024-learned-enforcement-perception.md) chose an RT-DETR
fine-tune trained on 3000 Replicator frames rendered from P's mast. Until today
the only number was a 99.3% detection rate, which asks whether a box overlaps
the robot — not whether it puts the robot in the right metre.

The pipeline is pixels only: box → bottom-centre pixel → ray through the
published K → ground plane → world X → least-squares speed over a ±0.30 m
window. No pose, no odometry, no TF, no depth, no simulator truth reaches the
estimate path.

**Measured** — `docs/evidence/estimator/`:

| | |
|---|---|
| Gate coverage | **5 of 5**, from pixels alone |
| Violations | **exactly one**, confirmed at gate 3.0, three compliant gates before it |
| Speed error, as measured | −10.59% mean over five gates |
| ...of which a newly-measured recording-path timing defect | −11.20% |
| **...leaving the estimator's own contribution** | **+0.62%** |
| Projection accuracy, 412 labelled frames | bias **+0.023 m**, sd 0.056 m |
| Detector gross-failure rate | **19.5%** of frames >0.5 m out |
| Can confidence tell you which? | **No.** 0.945 for failures, 0.936 for good ones |

The attribution is **reported, never applied**. Nothing in the table is
corrected by the lag: a correction derived from the run it corrects is a fitted
parameter wearing a mechanism's clothes.

---

## The speed policy is pinned to what A actually does

A demonstration where the limit cannot be exceeded is not a demonstration.
v1's limits are 0.8/1.2/1.5 m/s; scaled to the robot-sized corridor they are
0.24/0.36/0.45. **Robot1's entire measured band is 0.056–0.207 m/s** — below
all of them, so no violation could ever have arisen.

[ADR 0038](docs/adr/0038-the-speed-policy-pinned-to-a-measured-profile.md) pins
the policy to the profile measured over six runs, by one rule applied three
times: *a permissive tier sits above the fastest measurement in its zone, a
strict tier below the slowest.*

| tier | zone | A's measured band | limit | margin |
|---|---|---|---|---|
| wide | width > 1.5 m | max 0.2066 | **0.30** | +45% |
| mid | 1.2 < width ≤ 1.5 m | max 0.2023 | **0.25** | +24% |
| strict | width ≤ 1.2 m | **min 0.0555** | **0.04** | −39% |

Margins are against the extremes of six runs, not the mean, so which run gets
recorded does not decide whether the demonstration has a violation in it.

**Verifying that pin found a defect that had been live since ADR 0030.**
`width_at(2.4)` returns `1.2000000000000002`, so a bare `<=` put that gate in
the permissive zone; the strict zone held **one** gate against a confirmation
rule that needs two, and **a corner-confined violation could never have been
confirmed** — under this policy or any other. It hid because it is
scale-dependent: at v1's authored metres the same expression is exactly `4.0`.

---

## Two defects this delivery measured rather than inherited

### The image does not show the scene its timestamp claims

`CLAUDE.md` has listed pose-to-render latency as uncharacterised since v1 and
bounded it at one camera period. It is **three orders of magnitude larger than
that bound**, and it is not a latency — it is a rate deficit.

`tools/p_cam_render_lag.py` subtracts two times that share a clock: when the
schedule put A at a station, and when the pixels first show A there. The gap
grows **+0.112 s per second of sim time**, so content advances at 0.888× the
clock and any speed read off those stamps is low by 11.2%. The 0.08 m/s pass,
needing 2.8× more sim time for the same route, degrades to 0.713× exactly as
that predicts — which is why it is published as a **timing control**, not as a
compliant result.

### The lens can serve and see nothing

Two of six runs this morning were watched by an instrument that answered
`/healthz` for their whole length and resolved nothing — 500 samples, 100.4 s,
every metric column null. **"Zero faux launches" measured serving, not seeing**,
and that claim is withdrawn in the evidence where it was made.

[ADR 0037](docs/adr/0037-the-banner-means-seeing.md) makes `/healthz` report
per-topic rates and gates the banner on a non-zero scan rate.
[ADR 0039](docs/adr/0039-the-lens-is-asked-twice.md) adds a second checkpoint
immediately before the robot moves, **because 0037's own correlation acquired a
counterexample the same day**: run `20260814-125254`'s lens was created after
`simctl start`, passed the gate, and went deaf within seconds while delivering
normally. The placement keeps its conclusion and loses its reason.

Every run now records `lens_resolved_frac` in `run.json`. Over the six morning
runs it separates them completely: 0.604, 0.633, 0.632, 0.776 for the four that
saw, and exactly 0.000 twice.

---

## What is not claimed

- **The autonomous run and the enforcement run are not the same run.** The
  scripted constant-speed passes drive A at the speed A was *measured* driving,
  through P's real camera on both planes; the autonomy is evidenced separately
  on A's own plane. Two blocks prevent unifying them today: the v1 estimator
  returns the *camera's* station and the camera is now a static mast, and the
  fleet's `sim_runner.py` carries no camera.
- **`DELIVERED_CONFIRMED` has never been reported and cannot be.** Contact is
  physically demonstrated and truth-measured; the *confirmation witness* is not.
  The offline bench that appears to prove it feeds `robot.truly_stationary` —
  ground truth — and says so in its own comment. No drive-effort or current
  topic exists in the twin, and the laser ε came from a parked-robot regime
  ([ADR 0034](docs/adr/0034-the-mask-is-the-target.md)).
- **SLAM divergence is characterised, not solved.** Six samples spanning
  0.52–2.24 m with no ordering: heavy-tailed or bimodal, not a trend. A trend
  reported after four runs was refuted by the next two and is withdrawn in the
  evidence. It does not affect delivery accuracy (3.5 mm above); it affects
  transit, and a map bad enough to break planning aborts a run.
- **ADR 0029's fusion anomaly is open** — the EKF reporting 23.4× its own
  input, unexplained.
- **One profile, one pass per speed.** `nominal_m6_n3` only.
- **+0.62% is a residual, not a certified accuracy.** One run, and the
  difference of two measurements each with their own error.
- **No v1 certificate number is quotable for v2**
  ([ADR 0022](docs/adr/0022-robot-a-selection-gate.md)). The v1 figures that
  remain in the README describe the v1 run they were taken from.

## Cut for this delivery, and why

Each of these is a real piece of work that was scoped out today rather than
half-done. None is blocked; all are unstarted or unfinished.

| cut | one honest line |
|---|---|
| ArUco baseline A/B | ADR 0024's classical control exists but has never been run against the learned path, so "learned beats classical" is unmeasured and is not claimed |
| Matcher A/B | ADR 0027 attributes robot2's failure to a fleet-tuned matcher; a retune would justify re-running the same thresholds as a superseding ADR, and it has not been done |
| Hardware echo | no hardware has been touched; every figure here is simulation |
| Static requalification rows | the v1 dwell run reported a *requested* render mode as measured, so its renderer claim is invalidated. The replacement rows are **not measured** — never estimated, and never quoted |
| Mast scenery pole | P's camera is a prim at a surveyed pose with no visible mast body; cosmetic, and the viewport looks the poorer for it |
| Robust fitting for the 19.5% outliers | the obvious answer to a measured 20% contamination, deliberately **named and not applied** — changing the method after seeing the result is tuning to the answer |
| Fixing the render-lag defect | measured and attributed today; the fix is a change to how frames are stamped and is not a ship-day change |

## The capture

[`docs/evidence/ship-day/enforcement-f3.1.mp4`](docs/evidence/ship-day/enforcement-f3.1.mp4)
— 25 seconds of the violation pass with P's overlay: the detector's box and
score, the station it back-projects to, the local width, the limit that width
selects, and the five-gate table filling in as A passes each one until the
violation confirms.

It is rendered from the committed frames, stations and table by
`tools/render_enforcement_video.py`, **not screen-grabbed**, so it reproduces
byte-for-byte from the same inputs and a test asserts the overlay reads its
verdicts rather than deciding them again. Truth is drawn beside the estimate,
labelled `EVAL` — which incidentally puts the render-lag defect on screen: at
station 2.667 the truth line reads 3.009.

## Running it

```bash
source .venv/bin/activate
python -m scene.build --m 6.0 --n 3.0 --out out/corridor.usda
bash tools/check_workspace.sh          # ruff, pytest, colcon build, colcon test

# the autonomous delivery, on A's plane, watched
bash tools/corridor_profile_run.sh --robot robot1 --profile nominal_m6_n3 \
  --allow-contract-fail --corridor-slam

# P's camera across the gateway, both planes
bash tools/run_demo.sh --headless --record
```

`ros-jazzy-domain-bridge` is a runtime prerequisite. The two halves run on
separate domains, so a bare `ros2 topic list` in an unconfigured shell shows
nothing from either — set `ROS_DOMAIN_ID` to the side you mean to inspect.

## Where the evidence is

| topic | what |
|---|---|
| [`docs/evidence/estimator/`](docs/evidence/estimator/NOTES.md) | correction 3's table, the render-lag probes, the station controls |
| [`docs/evidence/crossing/`](docs/evidence/crossing/NOTES.md) | isolation certificate and mutation control |
| [`docs/evidence/speed-profile/`](docs/evidence/speed-profile/NOTES.md) | A's measured profile and the policy pinned from it |
| [`docs/evidence/lens/first-instrument/`](docs/evidence/lens/first-instrument/NOTES.md) | the six-run series, and two withdrawn claims |
| [`docs/evidence/bump-live/`](docs/evidence/bump-live/NOTES.md) | A touching B, truth-measured |
| [`docs/adr/`](docs/adr/README.md) | 38 decision records with a decision map |
| [`docs/REVIEW-LOG.md`](docs/REVIEW-LOG.md) | every finding raised and how it was dispositioned |

The commit history is part of the deliverable. Each behaviour commit carries
its own tests; each documentation commit records a measured result rather than
a promised one; and where a claim turned out to be wrong, the correction is a
new commit that says what it corrects.
