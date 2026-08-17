# Delivery day: the four runs, and what each one settled

**2026-08-14.** Isaac Sim 5.1, RTX 5070 Ti, scale 0.30,
`nominal_m6_n3`. Every run's artifacts are session-scoped under
`out/evidence/ship-day/` and `out/evidence/robot-a-gate/`.

| run | purpose | outcome |
|---|---|---|
| F3.0 probe | can the v1 adapter drive the twin in a composed arena? | **PASS, first attempt** |
| F3.1 | violation pass at A's measured cruise, 0.22 m/s | **PASS** — the speed table's source |
| F3.2 | compliant pass at 0.08 m/s | **timing control**, not a compliant result |
| F3.3 | isolation certificate + mutation control | **PASS** — green with red control |
| F3.4 | autonomous delivery, watched | **FAIL on two known gates**, delivery itself fine |

## F3.0 — the arena probe

The kill criterion for the whole enforcement path. The v1 adapter drives
`/World/Actors/A`, v1's kinematic box; the composed arena deactivates that prim
and puts the real yahboom twin at `/World/Robot` beside a `PhysicsScene` the v1
stage does not have.

```
ISAAC_ROS_CAMERA_PHYSICS scene_present=True deactivated=True robot_prim=/World/Robot
ISAAC_ROS_CAMERA_DRIVE speed_mps=0.22 route_s_m=2.196 reached_end=False updates=600 sim_span_s=9.983
ISAAC_ROS_CAMERA_GPU used_mib=3311 total_mib=16303
ISAAC_ROS_CAMERA_PASS render_products=1
```

2.196 m in 9.983 s is 0.22 m/s to four figures. `--deactivate-physics` is not a
workaround: `replicator_p_cam_dataset.py` rendered every one of the detector's
3000 training frames by writing a translate onto that same prim with the
timeline never played, so this reproduces the training distribution rather than
departing from it. A test asserts the two prim constants match across the files.

## F3.1 — the violation pass

```bash
STAGE=out/arena_corridor_robot1_nominal_m6_n3.usd MANIFEST=out/corridor.manifest.json \
CORRIDOR_PROFILE=nominal_m6_n3 ROBOT_PRIM=/World/Robot DEACTIVATE_PHYSICS=1 \
SPEED_MPS=0.22 UPDATES=3000 EVIDENCE_DIR=out/evidence/ship-day/f3.1-violation \
bash tools/run_demo.sh --headless --no-rviz --record
```

Full route, `reached_end=True`, 7.380 m in 33.55 s of sim time, 3313 MiB.
Recorded **from domain 43**, so every frame in the speed table crossed the
gateway. 376 images, 640×360, `/clock` 2057 messages.

Its analysis is `../estimator/NOTES.md` and is not repeated here.

## F3.2 — the compliant pass, published as a timing control

Full route at 0.08 m/s, `reached_end=True`, 5537 updates, 92.267 s, 1073 images.

Its speed figures are **not quoted as a compliant result.** Two measured
reasons, both in `../estimator/NOTES.md`: the render lag is 2.6× worse on this
run (content at 0.713× the clock against 0.888× on F3.1, exactly as a
throughput-limited renderer over 2.8× more sim time predicts), and the run is
three times longer with A out of frame for much of the tail, so the detector's
19.5% gross-failure population dominates the fits.

It earns its place by making the lag *scale* visible. One run shows a lag; two
runs at different speeds show it is a rate deficit rather than a constant, and
that is what separates "an offset to subtract" from "a defect to fix".

## F3.3 — the isolation certificate

Recorded in `../crossing/NOTES.md`. GREEN with the mutation control RED,
producer gate 1.0000, image crossing 0.9603 against a 0.95 floor.

## F3.4 — the autonomous delivery

```bash
bash tools/corridor_profile_run.sh --robot robot1 --profile nominal_m6_n3 \
  --allow-contract-fail --corridor-slam
```

`20260814-125254`. **The delivery worked and the run failed its gates.** Both
statements are true and they are about different things.

| | |
|---|---|
| Handoff | fired |
| Creep ticks | 3466 |
| Terminal state | `ARRIVED_UNPROVEN` — as ADR 0034 says every run must be |
| Closest approach | 0.0698 m |
| **A→B** | **0.2251 m**, inside the 3.5 mm band of the six morning runs |
| Gate failure 1 | EKF output gap 1.063 s exceeds 0.4 s |
| Gate failure 2 | map-frame goal error 370 mm exceeds 150 mm (ADR 0022) |

Both failures are known open items and neither is touched by anything in this
session. The map-frame goal error is ADR 0028's recorded state — the method is
validated in world frame and the map-frame gate stays red — and the EKF gap
belongs to ADR 0029's unexplained fusion anomaly.

### What this run actually settled

**It broke ADR 0037's correlation, hours after that record was accepted.**

The lens was created 71 s after `simctl start` — the placement 0037 adopted
*because* no lens created there had been observed to go blind — passed the
seeing gate, printed its banner, and went deaf within seconds:

```
lens coverage recorded in run.json: lens_rows=300 lens_span_s=60.2 lens_resolved_frac=0.000
**THE LENS SAW NOTHING: it served this whole run and resolved no pose.
  The run's navigation artifacts stand; its lens.json is not evidence of anything. ADR 0037.**
```

300 samples over 60.2 s of a 250 s run, every metric column null, frozen by the
60 s idle rule. The mechanism is still unidentified.

Two things came out of it. The `lens_resolved_frac` covariate — added that
morning for exactly this case — worked, and made a silent failure loud. And
[ADR 0039](../../adr/0039-the-lens-is-asked-twice.md) turned the one-shot gate
into one function called twice, so the next lens that dies during bring-up
refuses the run instead of producing an unwatchable success.

**The correction is recorded rather than the record edited.** ADR 0037 is
immutable; its placement keeps its conclusion — the seeing gate cannot be asked
before `/scan` exists — and loses its stated reason.

## The capture

`enforcement-f3.1.mp4` — 376 frames, **25.1 s at 15 fps**, rendered by
`tools/render_enforcement_video.py` from the committed frames, stations and
table.

**A capture of the artifacts, not of a screen.** A screen recording of a live
replay depends on an X server, a window manager, RViz's startup timing and
whatever else the desktop is doing — the least reproducible artifact in a
repository whose discipline is reproducible artifacts. This renders the same
video every time from the same inputs, and a test asserts the overlay *reads*
its verdicts from the table rather than deciding them again.

The overlay shows what P knows — the detector's box and score, the station it
back-projects to, the local width, the limit that width selects, and the gate
table filling in as A passes each one. Truth is drawn beside it in a separate
colour, labelled `EVAL`, which incidentally makes the render lag visible: at
station 2.667 the truth line reads 3.009.

## Verifying the launch: two more runs, after ADR 0039

`20260814-131949` and `-132443`, run to exercise the second checkpoint, which
had never fired live when it was written.

**Both mechanisms worked, and run 1 exercised the failure path as well as the
happy one.**

| | run 1 `131949` | run 2 `132443` |
|---|---|---|
| Previous run's lens reaped | yes, pid 3738372 | yes, pid 3801436 |
| Seeing gate, attempt 1 | **deaf** — "bound port 8765 but heard no scans in 20 s" | passed |
| Restart-once | **fired, and the second lens saw** | not needed |
| Second checkpoint (ADR 0039) | **"lens still seeing at the mission start"** | same |
| `lens_resolved_frac` | **0.861** | **0.886** |
| Handoff | fired | fired |
| Creep ticks | 3205 | 3442 |
| A→B | 0.2285 m | 0.2257 m |
| Verdict | FAIL on known gates | FAIL on known gates |

Run 1 is the more valuable of the two. A lens went deaf, the gate caught it,
the restart replaced it, and the replacement watched the whole run — **0.861 is
the highest coverage measured on any run to date**, against 0.776 for the best
of the morning six and 0.000 for the run that prompted ADR 0039. The mechanism
converted a would-be blind run into the best-watched one.

Run 2 needed no restart, which is the other half of the evidence: the retry
fires when it is needed and not otherwise.

Neither run passed its gates, and both failed on items that predate this
session — map-frame goal error (ADR 0028's recorded state, 381 and 409 mm),
midpoint longitudinal drift (ADR 0022, 0.235 and 0.182), and on run 2 an EKF
output gap of 0.566 s (ADR 0029's unexplained fusion anomaly). **The delivery
worked on both**: handoff fired, and A finished 0.2285 m and 0.2257 m from B,
inside the band of every other run this month.

### Runs 3 and 4: the acknowledgement, and the refusal firing for real

Runs 1 and 2 both finished *before* the wait-acknowledgement commit landed, so
two more were needed to exercise it. They exercised more than that.

**Run 3 `133559` — ADR 0039's second checkpoint fired for the first time.**

```
=== [13:36:00 +0s] simctl start ===  (~62s when healthy)
=== [13:37:21 +82s] lens ===  (~7s when healthy)
  lens: http://127.0.0.1:8765/  (map, scan, 3 pose ghosts, landmark)
...
**INFRASTRUCTURE: the lens went deaf during bring-up -- it passed its gate and
  stopped hearing, so the mission would be unwatched.**
```

The lens bound, heard scans, passed the gate, printed its banner — and was deaf
by the time the robot was due to move. The run refused instead of producing an
unwatchable success, which is exactly what that record was written for. Its
dump is the now-familiar signature: 300 rows, 60.2 s, **zero** resolved fits.

**Run 4 `133922` — clean launch, and Nav2 aborted.** Lens seen on the first
attempt, second checkpoint passed, coverage 0.818. Then `ABORTED` at 3.055 m
from B with yaw scale 1.1353 and midpoint drift 0.355 — a navigation failure in
ADR 0029's territory, with nothing to do with the launch.

### The launch is trustworthy; it is not yet reliable

Four runs under the current code:

| run | lens | outcome |
|---|---|---|
| `131949` | attempt 1 deaf → **restart worked** | watched **0.861**, delivered 0.2285 m |
| `132443` | clean | watched **0.886**, delivered 0.2257 m |
| `133559` | passed gate, deaf before the mission | **REFUSED** by the second checkpoint |
| `133922` | clean | watched **0.818**, Nav2 aborted at 3.055 m |

**Zero unwatchable successes, on the seeing definition** — which is the property
that was missing this morning, when the same definition would have failed 2 of
6. Every run either produced a watched mission or refused and said why.

**But deafness hit 2 of these 4 runs**, and it is now the dominant infrastructure
cost: each refusal spends an Isaac load. The signature is identical every time
— the lens hears its first messages, clears the gate, then receives nothing and
freezes at the 60 s idle rule with no resolved fit. `/dev/shm` sits at 372 MiB
of 24 GiB during a run, so it is not segment exhaustion by volume.

The mechanism is still unidentified and this is the third session it has
survived. It is the top open item on the instrument, and it is a DDS discovery
question rather than a corridor one.

### What the bring-up measurement said

Phase medians over the seven runs of the day, which is where the
"~120 s before the robot moves" header comes from:

| phase | median | share |
|---|---|---|
| `simctl start` | **62 s** | **52%** |
| contract precondition | 17 s | 14% |
| nav stack | 16 s | 13% |
| TF chain | 11 s | 9% |
| lens | 7 s | 6% |
| SLAM activation | 3 s | 2% |

Over half of bring-up is Isaac/Kit's cold start, and it is the step that
printed nothing of its own. That is why every phase banner now carries its
typical duration and the header names the 62 s step as expected rather than
leaving a first-time reader to guess. Two levers exist and neither was pulled
today: the contract precondition costs 17 s for a verdict this runner overrides
on every run, and `simctl start` is per-run only because the simulator is
stopped between runs.

## What none of these runs show

- **No run here is both autonomous and enforced.** F3.1 and F3.2 are scripted
  passes through P's real camera; F3.4 is autonomous on A's plane with no
  camera. The two blocks are named in `DELIVERY.md`.
- **One profile.** `wide_corner_m6_n4_5` and `uniform_m6_n6` are untouched
  today, so nothing here says the profile variant changes the policy visibly.
- **No repeat of F3.1.** The speed table is one pass. Its residual is a
  difference of two measurements, not a distribution.
