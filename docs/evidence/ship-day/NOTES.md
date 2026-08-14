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

## What none of these runs show

- **No run here is both autonomous and enforced.** F3.1 and F3.2 are scripted
  passes through P's real camera; F3.4 is autonomous on A's plane with no
  camera. The two blocks are named in `DELIVERY.md`.
- **One profile.** `wide_corner_m6_n4_5` and `uniform_m6_n6` are untouched
  today, so nothing here says the profile variant changes the policy visibly.
- **No repeat of F3.1.** The speed table is one pass. Its residual is a
  difference of two measurements, not a distribution.
