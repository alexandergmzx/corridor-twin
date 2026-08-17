# corridor-twin

An interview-sized digital-twin scenario for OpenUSD, ROS 2 Jazzy, and NVIDIA
Isaac Sim. Robot A navigates **autonomously** on its own lidar to deliver a
package to person B through a tapered corridor and around a corner onto the next
street. Traffic police P runs on a **separate ROS communication domain** from A
and measures A's speed from **P's own roadside camera** — a learned detector,
with a classical ArUco baseline specified but never run — crossing the domain
boundary through one allowlisted, one-way gateway. A carries no camera, and P
stays out of A's sightline.

> The paragraph above describes v2, the delivered system (ADRs 0021–0024). In
> v1 the camera was A's and the fiducials were surveyed wall markers; the v1
> figures in the milestone table below are true of the runs they describe and
> are quotable only as v1 (ADR 0022).

## Current status

**Delivered 2026-08-14 — read [`DELIVERY.md`](DELIVERY.md) first.** It maps the
three corrections from the 2026-08-04 review to their mechanisms, artifacts and
measured numbers, and lists what is deliberately not claimed.

The v2 headline: the isolation certificate is green with its mutation control
red; A delivers autonomously and touches B — contact truth-measured once, with
the approach reproducible to a 3.5 mm span across six runs while map quality
varies 4.3x; and P's learned detector recovers speed
at all five gates from pixels alone, with the estimator's own contribution to
the error measured at **+0.62%** once a newly-characterised recording-path
timing defect is accounted for.

The table below is the milestone history. **Every 2026-07 row is a v1 figure**
and ADR 0022 retires all v1 certificate numbers for v2 purposes; they remain
true of the runs they describe and are quotable only as v1.

| Date | Milestone | State | Measured result | Evidence |
|---|---|---|---|---|
| 2026-07-26 | Phase 1 baseline and first Isaac/ROS integration | **Working** | One 640×360 RGB render product, matching `CameraInfo`, and simulation `/clock` through the installed Jazzy bridge. Independent ROS probes passed headless and visible at exactly 15 Hz | [Activation record](docs/ACTIVATION.md) |
| 2026-07-27 | Geometry reconciled with the supplied diagram | **Working** | One-sided taper, a real perpendicular next street with a corner mass, and a five-piece route that turns in behind the east-wall stub to reach B in the pocket the drawing puts B in | [ADR 0010](docs/adr/0010-supplied-diagram-geometry.md), [ADR 0018](docs/adr/0018-model-the-east-wall-stub.md) |
| 2026-07-27 | Static production-camera fiducials | **Pixels valid, renderer claim invalidated** | All 15 selected frames passed, maximum station error 0.010563 m, delivered rate 14.999999 Hz, and a mirror applied to the same capture produced zero passing frames | [Static fiducials](docs/evidence/static-fiducials/NOTES.md) |
| 2026-07-27 | Live camera-only enforcement demonstration | **Working** | All four enforcement gates measured at 1.0 m/s truth, maximum speed error 0.0369 m/s, **exactly one** violation — 0.191 m/s over the 0.80 m/s corner rule at station 10.0 m — and 3354 MiB of the RTX 5070 Ti with one render product | [Live demo](docs/evidence/live-demo/NOTES.md) |
| 2026-07-31 | Police placement audit and correction | **Merged** | P moved inside the east wall behind a purpose-built corner screen; the occlusion verifier bound to the composed USD; twelve findings closed across two review rounds | [ADR 0019](docs/adr/0019-relocate-p-inside-the-east-wall-with-a-corner-screen.md), [Review log](docs/REVIEW-LOG.md) |
| 2026-08-04 | Communication-domain isolation | **Working** | A and P on separate ROS domains, crossed only by a three-topic one-way allowlist. Proved without a GPU: the police domain cannot discover A's camera topic, and forcing both probes onto one domain fails the guard | [ADR 0020](docs/adr/0020-communication-domain-isolation.md) |
| 2026-08-11 | Robot A selected by a measured gate | **Closed** | The corridor gate ran on robot2 and **failed**, so **A stays robot1** by ADR 0022's fallback clause — robot1 is the untested fallback, not a gate winner. robot2's encoder-less odometry published nothing until ~5 m in on every profile — first `odom_laser` at 4.77–5.83 m, midpoint drift 1.0 against a 0.05 bound. Not a chassis verdict: the matcher is fleet-tuned for a 4×4 m room | [ADR 0027](docs/adr/0027-robot-a-selection-outcome.md) |
| 2026-08-11 | Isolation verified live, on rendered pixels | **Green, mutation red** | Certificate green — P's observed graph equals the declared allowlist **exactly** — with the mutation control **red**. Producer gate 14.993 Hz vs 15.0 declared (ratio **0.9995**); image crossing **0.954** against a 0.95 floor at the pinned 640×360. At 1280×720 the same stream crosses at 0.926 against CameraInfo's 0.998, which records the transport ceiling | [ADR 0026](docs/adr/0026-isolation-verification.md), [crossing evidence](docs/evidence/crossing/NOTES.md) |
| 2026-08-13 | Enforcement perception is learned | **Working** | An RT-DETR (`rtdetr_r18vd`, Apache-2.0) fine-tune trained on a **3000-frame Replicator dataset** rendered from P's own mast — 1000 per profile, domain-randomized, **1581 train / 412 val after dropping the frames where A is not visible from the mast** — finds A on held-out synthetic frames at **99.3%**. Pixels only: box → bottom-centre → ray through the published `K` → ground plane → world X | [ADR 0024](docs/adr/0024-learned-enforcement-perception.md), [detector evidence](docs/evidence/detector/NOTES.md) |
| 2026-08-14 | A navigates autonomously and touches B | **Working — contact measured once, witness unproven** | Governed Nav2 on a map SLAM builds live, no authored route: A is told B's *address*, never the path. **A physically touches B in one run** (`20260814-031348`) — 0.2146 m against a 0.2175 m modelled contact, **2.9 mm past it**, truth-measured. A *separate* six-run series holds **A→B to a 3.5 mm span while map quality varies 4.3×**, because the terminal approach closes on a landmark A sees rather than on the map's opinion of where it is — but those six land at 0.2249–0.2284 m, 7–11 mm *short* of the same contact, so the band is approach reproducibility, not reproduced contact. A seventh run fell inside the band with no map figure recorded | [ADR 0023](docs/adr/0023-governed-nav2-live-slam.md), [ADR 0033](docs/adr/0033-arrival-is-contact.md), [bump evidence](docs/evidence/bump-live/NOTES.md) |
| 2026-08-14 | Speed policy pinned to A's measured profile | **Working** | v1's limits are unreachable at robot scale, so no violation could ever have arisen. Limits pinned to **0.30 / 0.25 / 0.04 m/s** by one rule applied three times. Verifying the pin found a live defect: `width_at(2.4)` returns `1.2000000000000002`, so a bare `<=` put that gate in the permissive zone and **a corner-confined violation could never have been confirmed** under any policy | [ADR 0038](docs/adr/0038-the-speed-policy-pinned-to-a-measured-profile.md), [speed profile](docs/evidence/speed-profile/NOTES.md) |
| 2026-08-14 | Camera-only speed at every gate | **Working** | **5 of 5** gates recovered from pixels alone, **exactly one** violation confirmed. Speed error −10.59% as measured, of which a newly-characterised recording-path timing defect accounts for −11.20%, **leaving the estimator's own contribution at +0.62%**. Reported, never applied. Projection-only station bias +0.023 m (sd 0.056 m, truth boxes, 412 labelled frames); through the detector, gross-failure rate **19.5%** of frames >0.5 m out, and confidence cannot tell you which | [Estimator evidence](docs/evidence/estimator/NOTES.md) |
| 2026-08-14 | Bring-up is reliable and watched | **Working — a lens verdict, not a mission verdict** | Lens deafness **2 of 4 → 0 of 18** under DDS-over-UDP-only, and the seeing gate now demands scan-count *progress* rather than a windowed rate a burst can echo through. Command → first motion **≈131 s → ≈101 s** corridor-side. The synthetic repro built to confirm the mechanism **did not reproduce it**, and that negative result is filed against the hypothesis | [ADR 0040](docs/adr/0040-corridor-sessions-are-udp-only.md), [ADR 0041](docs/adr/0041-seeing-means-progress.md), [evidence](docs/evidence/bringup-rework/NOTES.md) |

### What is deliberately not claimed

Each of these is open on purpose and recorded rather than hidden. Some are
measurements that have not been taken; the rest are measurements that came out
red and stayed red, named here rather than softened.

| Open item | Why it is not claimed | Consequence |
|---|---|---|
| No canonical **static** qualification | The recorded dwell run reported a *requested* renderer mode as measured. Its summary is preserved unmodified as `qualification-summary-v1-request-echo-invalidated.json` | Its pixel, calibration, rate and mirror-control results stand; its renderer claim does not. A fresh paired capture is required |
| ~~Pose-to-render latency uncharacterised~~ **MEASURED 2026-08-14, and far worse than the bound** | It is not a latency but a rate deficit: image content advances at 0.888x the clock on a 33 s run and 0.713x on a 92 s one | Any speed read off those header stamps is low by 11-29%. Measured, attributed, and **not corrected for** — see [the estimator evidence](docs/evidence/estimator/NOTES.md). Fixing how frames are stamped is future work |
| Every GPU figure predates ADR 0019 | The scene changed shape after the last capture: P moved sides and a `CornerScreen` was added | The figures above describe the pre-correction geometry and are pending refresh, not withdrawn |
| Host is unsupported | NVIDIA's checker rejects Linux Mint regardless of the passing hardware and live-stream gates | Recorded as a platform risk; Ubuntu 24.04 remains the fallback |

**v2 open items.** These qualify the 2026-08 rows above, and
[`DELIVERY.md`](DELIVERY.md) carries the full list with its reasoning.

| Open item | Why it is not claimed | Consequence |
|---|---|---|
| **The autonomous run and the enforcement run are not the same run** | The estimator returns the *camera's* station and the camera is now a static mast; the fleet's `sim_runner.py` carries no camera | Autonomy is evidenced on A's plane, enforcement through P's camera on both planes. Unifying them is real work, not a formality |
| **`DELIVERED_CONFIRMED` has never been reported and cannot be** | Contact is physically demonstrated and truth-measured, but the *confirmation witness* is not. The offline bench that appears to prove it feeds `robot.truly_stationary` — ground truth — and says so in its own comment | Runs report `ARRIVED_UNPROVEN` ([ADR 0033](docs/adr/0033-arrival-is-contact.md)). No drive-effort or current topic exists in the twin |
| **Mission-level reds are open, and the bring-up result does not touch them** | 0-deaf-of-18 is a *lens* verdict. On the ten-run acceptance batch every run still carried a red mission criterion — six the EKF-output-gap family (up to **1.438 s** against a 0.4 s bound), the rest unproven contact | Gate-green runs were 4 of 10, against 0 of 5 at baseline — no regression, but not a green mission |
| **ADR 0029's fusion anomaly is open** | The EKF reports **23.4×** the rotation its own IMU input contains, unexplained, and the fix is outside this repository | [ADR 0029](docs/adr/0029-map-divergence-at-the-corner.md), [fusion notes](docs/evidence/robot-a-gate/NOTES-fusion-anomaly.md) |
| **SLAM divergence is characterised, not solved** | Six samples spanning 0.52–2.24 m with no ordering: heavy-tailed or bimodal, not a trend. A trend reported after four runs was refuted by the next two and is withdrawn where it was claimed | It does not affect delivery accuracy — A→B spans 3.5 mm across the same runs — it affects transit |
| **ADR 0024's ArUco baseline has never been run** | The classical control is an ArUco plate on A; the A/B against the learned path was cut for delivery | "Learned beats classical" is **unmeasured and not claimed**. Speed is reported once, not twice |
| **One profile, one pass per speed** | `nominal_m6_n3` only | The other `(m,n)` profiles are unmeasured for v2 |
| **+0.62% is a residual, not a certified accuracy** | One run, and the difference of two measurements each carrying its own error | Quotable as an attribution, not as an error bar |
| **No v1 certificate number is quotable for v2** | [ADR 0022](docs/adr/0022-robot-a-selection-gate.md) retires them | The 2026-07 rows above are true of the runs they describe and are v1-only |

## The visibility constraint, reinterpreted (2026-08-04)

The supplied task says *the robot cannot see the traffic police, but the police
can read the data from the robot*. This repository first read that as a statement
about **what A's camera can image**, and built the geometry to match: continuous
occlusion certification over the turn, a purpose-built corner screen, and the
placement chain in ADR 0011, 0017, 0018 and 0019.

In the technical interview on 2026-08-04 the interviewer clarified the intended
meaning. The constraint was about **ROS 2 / DDS communication-domain isolation**:
A and P were meant to run on separate communication planes with no topic-level
visibility between them. Not occlusion.

That is a reinterpretation, not a discovery, and it is recorded as one. The
geometric reading was a reasonable construction of an ambiguous English sentence,
it was implemented thoroughly, and it was wrong about the author's intent.

**What changed.** A runs on ROS domain 42, P on domain 43. DDS discovery does not
cross a domain boundary, so P cannot discover, list, or subscribe to anything A
publishes — including topics nobody thought to forbid. One gateway
(`src/corridor_gateway`) relays exactly three topics one way, A to P: the camera
image, its calibration, and `/clock`. Simulator truth stays on A's domain and is
not on that list, so P's inability to read truth is now a property of the
transport rather than a rule the observer promises to obey.

**What did not change.** The occlusion work stands. P really is hidden behind a
wall, the certificate still proves it, and its gate still passes. It is now
understood as *physical-scenario realism* rather than as the implementation of
the assignment's constraint — both claims are true of the shipped system, and
they are separate claims. Nothing in ADR 0001–0019 was edited; ADR 0020 amends a
single row of ADR 0011's concept table by new record, as the immutability rule
requires, and supersedes nothing.

**Why it is not just a naming convention.** The isolation is proved, without a
GPU or Isaac, by standing a node up in each domain and asking what it can see
([`test/test_domain_isolation.py`](test/test_domain_isolation.py)). Every
negative is paired with a positive control in the same environment and skips
rather than passes when discovery is unavailable, so a container without
multicast cannot report an isolation it never tested. Forcing both probes onto
one domain fails the guard.

## Architecture at a glance

```mermaid
flowchart LR
    Config["corridor.yaml"] --> Build["scene.build"]
    Build --> USD["corridor.usda"]
    Build --> Manifest["corridor.manifest.json"]
    USD --> Proof["Occlusion certificate"]
    Manifest --> Proof

    subgraph RobotDomain["ROS domain 42 &mdash; A"]
        USD --> Isaac["Isaac Sim 5.1"]
        Isaac --> Lidar["A's navigation lidar<br/><i>contract sensor, never evidence</i>"]
        Lidar --> Nav["SLAM + governed Nav2<br/>+ docking creep"]
        Nav --> Wheels["A's wheels"]
        Isaac --> Contract["<b>/p_cam</b> Image + CameraInfo + /clock<br/><i>P's instrument, rendered here, in transit</i>"]
        Synthetic["Synthetic camera"] --> Contract
        Truth["<i>simulator truth</i><br/>ground-truth pose &middot; speed"]:::blocked
    end

    Contract ==> GW["<b>corridor_gateway</b><br/>allowlist &middot; one way"]

    subgraph PoliceDomain["ROS domain 43 &mdash; P"]
        Observer["police_observer<br/><i>live: ArUco-fiducial estimator</i>"] --> Output["Speed estimate / violation"]
        Bag["recorded frames"] -.-> Infer["<i>offline</i>: tools/p_cam_infer.py<br/>learned detector &rarr; station &rarr; speed<br/><i>ADR 0024's ArUco-on-A baseline: never run</i>"]
    end

    GW ==> Observer
    GW ==> Bag
    Manifest --> Observer
    Infer --> Output
    Truth -. "not on the allowlist" .-x GW

    classDef blocked fill:#5c1f1f,color:#ffffff,stroke:#ff6b6b,stroke-width:2px;
```

The observer is prohibited from reading simulated pose, odometry, TF-derived
robot position, or synthetic ground truth — and since ADR 0020 it is also unable
to: those producers live on A's domain and are absent from the gateway allowlist,
so they never appear in P's graph. Its measurement timestamp comes from the image
header. The manifest reaches P as a file, not a topic, so it crosses no boundary.

## Repository layout

- `src/corridor_scene`: `usd-core` authoring, marker assets, manifest generation,
  stage validation, and CPU geometric occlusion verification.
- `src/corridor_interfaces`: timestamped speed-estimate and violation messages.
- `src/police_observer`: pure perception core, ROS adapter, synthetic frame
  generator, and synthetic publisher.
- `src/corridor_gateway`: the one-way allowlisted domain bridge, and the only
  participant in both domains.
- `config/robot1`: A's Nav2, SLAM, and corridor launch configuration.
- `tools/build_corridor_arena.py`: composes the robot-scale arena — the authored
  corridor plus the twin and its lidar — that every 2026-08 milestone was
  measured on. Runs in Isaac's Python.
- `tools/corridor_profile_run.sh`: the autonomous delivery run — Isaac, then the
  lens immediately after it, then the preconditions, SLAM, Nav2, and the
  docking creep.
- `tools/lens/`: the corridor lens, the instrument the run is *watched* on. It
  refuses a run it cannot see, it outlives the run and freezes, and the next run
  replaces it ([ADR 0035](docs/adr/0035-the-lens-is-the-first-instrument.md)).
- `tools/corridor_dock.py`, `tools/landmark_detector.py`: the terminal approach —
  A perceives B for itself and closes on contact.
- `tools/train_p_cam_detector.py`, `tools/p_cam_infer.py`: the learned
  enforcement detector, trained offline on Replicator frames.
- `tools/p_cam_render_lag.py`, `tools/p_cam_station_bench.py`: the probes that
  measured the render-lag defect and the estimator's station accuracy.
- `docs/adr`: architecture decisions and consequences.
- `docs/DESIGN.md`: versioned system design and hardware budget.
- `docs/SENSOR-FEED.md`: the robot-to-observer interface contract.
- `test`: repository and end-to-end contract tests.
- `tools/isaac_5_1_ros_camera.py`: installed-version camera/clock adapter, kept
  outside the CPU-testable packages.
- `tools/ros_camera_contract_probe.py`: simulator-independent live ROS contract
  probe.
- `tools/ros_aruco_capture.py` and `tools/aruco_render_gate.py`: production-feed
  capture plus truth-isolated static station qualification.
- `docs/evidence`: curated measured results and provenance; bulk runs remain
  under ignored `out/evidence`.

## Build and run

### 1. Read the reconciled scenario

The supplied task and drawing are versioned at [`docs/ROBO_TASK.pdf`](docs/ROBO_TASK.pdf).
It is a plan view with no scale bar and widths labelled only as `m` and `n`, so
the project separates what it fixes from what the project chooses:

| From the source | A project choice |
|---|---|
| `m >= n`, narrowing toward the corner | Corridor length, 12.0 m |
| One straight face, one tapering face | Next-street width 6.0 m, length 10.0 m |
| A perpendicular next street with real walls | Turn radius 2.0 m |
| B along that street, P at its corner | B at 8.0 m along the street |

`src/corridor_scene/config/corridor.yaml` publishes this as
`topology: reconciled_with_supplied_diagram` and
`metric_scale: demo_assumption`. The speed limit is a demonstration-only
piecewise rule, because the task states none. See
[ADR 0010](docs/adr/0010-supplied-diagram-geometry.md).

### 2. Review the demo GPU qualification

The installed RTX 5070 Ti reports 16303 MiB with driver 580.173.02. The complete
activation evidence and repeatable commands are in `docs/ACTIVATION.md`.
Headless and visible loaded-stage snapshots used 916 MiB and 871 MiB total,
respectively. With the one camera render product and ROS bridge active, measured
totals were 2,494 MiB headless and 2,591 MiB visible. Every run used real-time
`RaytracedLighting`; none used path tracing. The unsupported Mint result and
IOMMU warning remain known risks; Ubuntu 24.04 is the fallback rather than an
unrecorded last-minute change.

The v1 static rendered-fiducial gate measured one headless product at 3,024 MiB
while delivering 57 synchronized pairs. See the
[accepted evidence](docs/evidence/static-fiducials/NOTES.md).

> **Every figure in this step is v1** and is not quotable for v2
> ([ADR 0022](docs/adr/0022-robot-a-selection-gate.md)): they predate both the
> ADR 0019 placement correction and the v2 architecture, so the scene they
> measured is not the scene that ships. The v2 equivalent measured during the
> ship-day enforcement pass is **3,313 MiB headless**
> ([ship-day evidence](docs/evidence/ship-day/NOTES.md)); no v2 static
> qualification exists, and none is estimated.

### 3. Create the development environment

Keep ordinary OpenUSD/ROS development separate from Isaac's Python environment:

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3.12 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-dev.txt
python -m pip install --no-deps -e src/corridor_scene -e src/police_observer
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
  --os=ubuntu:noble
```

The demonstration needs the upstream domain bridge, which `corridor_gateway`
declares as a dependency. On Ubuntu the `rosdep` line above installs it; this
host runs Linux Mint, which `rosdep` refuses (`Unsupported OS: mint`), so install
it directly:

```bash
sudo apt install ros-jazzy-domain-bridge
```

Without it the demonstration cannot bridge the camera stream to P and the
integration test skips. The isolation proof itself does not need it and still
runs. (In v2 that stream is `/p_cam/*`, P's own instrument rendered on A's
plane and in transit across the gateway — not A's camera. A is camera-less.)

Do not install pip `usd-core` into Isaac Sim's bundled Python. Do not source two
different ROS installations into the same shell.

### 4. Generate and prove the scene

```bash
source .venv/bin/activate
python -m scene.build --m 6.0 --n 3.0 --out out/corridor.usda
python -m scene.occlusion \
  --stage out/corridor.usda \
  --manifest out/corridor.manifest.json \
  --out out/occlusion-certificate.json
```

The build creates the USDA, `corridor.manifest.json`, and marker PNG files, and
rejects any authored profile whose layout would put P in a wall or in the road,
or whose turn would not fit the junction.

The occlusion command exits nonzero if any trajectory interval or composed-mesh
audit ray is uncertified. It proves the binding requirement that A's camera
never images P, and the stronger reciprocal claim that an opaque wall does the
hiding. Straight route intervals use their exact endpoint segment; turn
intervals use a conservative rectangle derived from the circular arc's exact
extrema, so an arc is never replaced by its chord. Current result for the
nominal profile: 5 covered intervals, 404 audit rays with 0 failures, nearest
blocking surface 5.366 m, `EastBuilding` the sole blocking prim. Since ADR 0017
put P east of the junction, one plane of constant X separates it from the whole
route, so the proof needs five intervals rather than the seventy-eight the
previous placement required.

Check the other authored profiles too, since each one moves P:

```bash
python -m scene.occlusion --stage out/corridor.usda \
  --manifest out/corridor.manifest.json \
  --profile wide_corner_m6_n4_5 --out out/occlusion-wide.json
python -m scene.occlusion --stage out/corridor.usda \
  --manifest out/corridor.manifest.json \
  --profile uniform_m6_n6 --out out/occlusion-uniform.json
```

### 5. Build and test ROS

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --symlink-install
source install/setup.bash
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
pytest -q
colcon test
colcon test-result --verbose
```

Or run the same sequence, including lint, with `bash tools/check_workspace.sh`.
The explicit `PYTHONPATH` is needed because `colcon test` launches the system
Python even when the calling shell has an active venv; it keeps both `pxr` and
the NumPy version used by ROS/OpenCV deterministic on this host.

### 6. Run the live demonstration

**There are two runs, and they are deliberately not the same run.** The
autonomous delivery is evidenced on A's own plane; the enforcement pass drives A
at the speed A was *measured* driving, through P's real camera on both planes.
Two blocks prevent unifying them today: the estimator returns the *camera's*
station and the camera is now a static mast, and the fleet's `sim_runner.py`
carries no camera. This is listed among the things
[`DELIVERY.md`](DELIVERY.md) does not claim.

**The autonomous delivery** — governed Nav2 on a live SLAM map, no authored
route, watched by the lens:

```bash
bash tools/corridor_profile_run.sh --robot robot1 --profile nominal_m6_n3 \
  --allow-contract-fail --corridor-slam
```

**Both flags are disclosed rather than quietly carried, because both fire on
every run behind the evidence above.**

`--allow-contract-fail` is an **override of a pre-existing twin defect**, not a
lowered threshold. robot1's Isaac twin publishes `/scan` off the 12.0 Hz its
contract declares — 13.4–15.1 Hz across the last twelve runs, and one run came
up at 8.6 Hz — so without the flag the runner classifies the precondition as
INFRASTRUCTURE and refuses to start. With it the check still runs, still fails,
and every artifact carries `PRECONDITION FAILED (recorded, overridden)`.

`--corridor-slam` is **not** the shipped SLAM configuration. It selects
`config/robot1/slam_robot1_corridor.yaml`, whose own header opens "NOT IN USE,
kept as a record" — it differs from the fleet canonical by turning loop closing
off, that hypothesis was falsified by measurement, and the runner defaults back
to the canonical. The flag stays in the printed command only because it is the
arm every run cited above actually used.

The lens comes up immediately **after `simctl start`, before every
precondition** — [ADR 0035](docs/adr/0035-the-lens-is-the-first-instrument.md)
asked for lens-before-simulator and was rolled back by its own clause when
lenses created before Isaac went deaf
([ADR 0037](docs/adr/0037-the-banner-means-seeing.md)). It refuses the run if it
cannot see, it is asked twice — at the lens and again before the robot moves
([ADR 0039](docs/adr/0039-the-lens-is-asked-twice.md)) — and the gate demands
scan-count *progress* rather than a windowed rate a burst can echo through
([ADR 0041](docs/adr/0041-seeing-means-progress.md)). The cost is stated rather
than hidden: the Isaac load ahead of it is unwatched, and a refusal throws it
away. Watch it. A phantom
landmark detection once re-aimed an entire mission while B's real post stood
five metres away, and every number in the resulting JSON looked defensible — on
the lens it is two circles far apart.

**The enforcement pass** — F3.1, the run the speed table and the capture come
from. It needs the composed robot-scale arena, built from step 4's stage in
**Isaac's** Python (not the venv — see the environment discipline above):

```bash
~/isaac/env_isaaclab/bin/python tools/build_corridor_arena.py \
  --profile nominal_m6_n3
```

The pass itself is recorded **from domain 43**, so every frame in the speed
table crossed the gateway:

```bash
STAGE=out/arena_corridor_robot1_nominal_m6_n3.usd \
MANIFEST=out/corridor.manifest.json CORRIDOR_PROFILE=nominal_m6_n3 \
ROBOT_PRIM=/World/Robot DEACTIVATE_PHYSICS=1 SPEED_MPS=0.22 UPDATES=3000 \
EVIDENCE_DIR=out/evidence/ship-day/f3.1-violation \
bash tools/run_demo.sh --headless --no-rviz --record
```

> **A bare `bash tools/run_demo.sh` is not this run.** Its defaults open the v1
> stage and drive v1's kinematic box at 1.0 m/s, and the script's own header
> says so in capitals: it "RUNS, BUT IT IS NOT THE v2 DEMONSTRATION" — the v1
> scenario wearing v2 topic names. It remains good for the domain split, the
> gateway crossing, and the camera-only estimator, all unchanged by the rename;
> it is **not** evidence for P's camera placement, the v2 resolution or rate,
> autonomous navigation, or the learned detector. The option and profile tables
> below describe that default v1 run.

It starts all three parts of the demonstration in the environments they require
and never in one shell: the camera-only observer, the enforcement display and
RViz on system Jazzy on **domain 43**; the gateway, which is the only participant
in both domains; then Isaac Sim on its bundled Jazzy on **domain 42**, driving A
along the authored route. The observer side comes up first so the consumers have
finished discovery before the publisher starts, which is the ordering the live
camera contract in step 8 already validates.

Override the domains with `ROBOT_DOMAIN_ID` and `POLICE_DOMAIN_ID` if 42 and 43
are taken on your network. The script refuses to run them equal — that failure is
otherwise silent, because everything starts, every topic flows, and the isolation
claim is simply false.

To see the boundary rather than take it on trust, with the demo running:

```bash
ROS_DOMAIN_ID=42 ros2 topic list   # /p_cam/* at its source, and ground truth
ROS_DOMAIN_ID=43 ros2 topic list   # the bridged camera and P's output; no truth
```

The same two commands diagnose a silent run. With `--wait-for-publisher` at its
upstream default, a dead Isaac side looks exactly like working isolation: no
camera topic on domain 42 means A never published, not that the boundary held.

The default drives A at a constant **1.0 m/s**. That one unchanged speed is
legal on the wide approach at a 1.2 m/s limit and illegal once the corridor
narrows to a 0.8 m/s limit, so a single pass shows a compliant stretch and
exactly one violation episode without anyone touching a throttle.

| Option | Effect |
|---|---|
| `--headless` | no Isaac viewport; RViz still shows the camera feed and readout |
| `--no-rviz` | observer and display only, for a terminal-only check |
| `--record` | `ros2 bag record` the camera, estimate, violation and clock topics |
| `VIEW=corner` | Isaac viewport perspective; `rviz` (default) matches the RViz angle, `corner` frames the junction, `chase` follows A. GUI only; it moves Kit's own viewport camera and adds no render product |
| `SPEED_MPS=1.8` | sustained speeding: one episode that opens on the approach |
| `SPEED_MPS=0.6` | fully compliant run, no violation |
| `CORRIDOR_PROFILE=wide_corner_m6_n4_5` | a different authored `(m,n)`; **no violation at 1.0 m/s**, see below |

**Switching the corridor variant changes the policy, not just the walls.** The
speed limit is a function of local clear width, so a wider corner means a
looser rule at the same station:

| Profile | Narrowest gate | Limit there | Violation at 1.0 m/s |
|---|---:|---:|---|
| `nominal_m6_n3` | 3.50 m | 0.8 m/s | yes, one at station 10.0 m |
| `wide_corner_m6_n4_5` | 4.75 m | 1.2 m/s | **no** |
| `uniform_m6_n6` | 6.00 m | 1.5 m/s | **no** |

A variant switch therefore turns the readout green at the same commanded speed.
That is the demonstration's point — geometry drives policy — and not a fault.
To show a violation on `wide_corner_m6_n4_5`, raise the speed above its 1.2 m/s
limit; `uniform_m6_n6` has no taper at all, so it is the control case where the
rule never tightens. Only `nominal_m6_n3` has been measured live.

Logs and the commanded-pose schedule land under `out/evidence/live-demo/`. The
commanded schedule is simulator truth and is labelled as evaluator-only; it is
never an observer input.

If Isaac is unavailable, the script says so and points at the simulator-free
fallback below, which is also the recorded fallback for the demonstration.

### 7. Run the simulator-free ROS demo

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch police_observer synthetic_demo.launch.py \
  manifest:=$PWD/out/corridor.manifest.json speed_mps:=1.8
```

In another sourced shell:

```bash
ROS_DOMAIN_ID=43 ros2 topic echo /police/speed_violation \
  corridor_interfaces/msg/SpeedViolation --once --no-daemon
```

Note the echo needs `ROS_DOMAIN_ID=43`, or the shell will find nothing: this
launch runs the same two-domain topology the live path does. The synthetic
publisher stands in for A on domain 42, the observer and display sit on domain
43, and the gateway is started between them. It is the only end-to-end path that
needs no GPU, so it is where the isolation is easiest to see for yourself.

The launch deliberately sets `PYTHONNOUSERSITE=1`: this machine has a user-local
NumPy 2.2 wheel, while ROS Jazzy's OpenCV/cv_bridge binaries use the NumPy 1.x
ABI. No global package is modified. Add `use_sim_time:=true` to make the
synthetic publisher the single `/clock` source, which the gateway then relays to
P's domain.

### 8. Run the live Isaac camera contract

Start the external probe in a system-ROS terminal before starting Isaac:

```bash
source /opt/ros/jazzy/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 \
  tools/ros_camera_contract_probe.py --minimum-pairs 12 --timeout 90
```

In a second terminal, start the finite adapter. Do not source system ROS there;
the adapter re-executes with Isaac's bundled Jazzy libraries and rejects Python
path leakage from the host environment:

```bash
OMNI_KIT_ACCEPT_EULA=YES \
  ~/isaac/env_isaaclab/bin/python tools/isaac_5_1_ros_camera.py \
  out/corridor.usda --updates 420 --report-gpu-memory
```

Add `--gui` for the finite visible-viewport check. Success requires both
`ROS_CAMERA_PROBE_PASS` and `ISAAC_ROS_CAMERA_PASS`; an Isaac-only pass does not
prove the ROS interface. The adapter creates exactly one render product and no
pose, odometry, TF, depth, or test-truth publisher.

### 9. Qualify rendered fiducials through the production ROS camera

Start a dedicated capture process in the system-Jazzy environment:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 .venv/bin/python tools/ros_aruco_capture.py \
  --out-dir out/evidence/static-fiducials/manual/capture \
  --minimum-pairs 18 --timeout 240 --idle-after-minimum 3
```

In the Isaac shell, hold A at the five required world-X stations while reusing
the same camera graph:

```bash
env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u PYTHONPATH \
OMNI_KIT_ACCEPT_EULA=YES \
~/isaac/env_isaaclab/bin/python tools/isaac_5_1_ros_camera.py \
  out/corridor.usda --manifest out/corridor.manifest.json \
  --profile nominal_m6_n3 \
  --static-probe-out out/evidence/static-fiducials/manual/static-truth.json \
  --report-gpu-memory
```

Then run the offline acceptance comparison:

```bash
PYTHONPATH=src/police_observer .venv/bin/python tools/aruco_render_gate.py \
  --capture out/evidence/static-fiducials/manual/capture/capture.json \
  --truth out/evidence/static-fiducials/manual/static-truth.json \
  --manifest out/corridor.manifest.json --profile nominal_m6_n3 \
  --out out/evidence/static-fiducials/manual/aruco-gate.json
```

The capture process receives no pose or truth topic. Pixel analysis is separate
from the evaluator that reads the static schedule. Exact accepted commands and
the actual-capture mirror control are in the
[evidence notes](docs/evidence/static-fiducials/NOTES.md).

### 10. Repeat the installed Isaac 5.1 composition smoke

```bash
OMNI_KIT_ACCEPT_EULA=YES \
  ~/isaac/env_isaaclab/bin/python tools/isaac_5_1_smoke.py \
  out/corridor.usda --updates 60 --report-gpu-memory

OMNI_KIT_ACCEPT_EULA=YES \
  ~/isaac/env_isaaclab/bin/python tools/isaac_5_1_smoke.py \
  out/corridor.usda --gui --updates 240 --report-gpu-memory
```

The smoke uses explicit real-time `RaytracedLighting`, 640×360, no path tracing,
and no sensor render product. Both modes validate composition and
installed-version schemas; the finite GUI mode opens a visible viewport and
closes itself. Use the preceding live contract for camera/ROS validation.

## Deliberate GPU budget

The selected RTX 5070 Ti has 16 GB VRAM, but this demo still avoids spending that
capacity casually. The initial Isaac configuration remains:

- one 640×360 RGB camera at 15 Hz;
- one small robot and primitive static environment;
- small marker textures and simple opaque materials;
- primitive/convex colliders where possible;
- CPU physics until measurement shows a reason to change;
- no depth, radar, segmentation, RL, or extra render products;
- no interactive path tracing.

The soft operating target is below 14 GB steady-state VRAM, leaving recovery
headroom for the GUI and ROS bridge.

**Two v2 changes to this budget, both deliberate.** The single RGB render
product is now **P's enforcement camera**, not A's — A carries no camera
([ADR 0021](docs/adr/0021-police-owned-sensing-and-isolation-gate.md)) — and
**A's navigation lidar is the fleet twin's contract sensor** on A's plane. The
lidar is what makes the autonomy claim real; it is never an enforcement
evidence source, and it is not a second render product. Replicator is used
**offline**, to render the detector's 3000-frame training set
([ADR 0024](docs/adr/0024-learned-enforcement-perception.md)); it is not part
of the demonstration's runtime budget. Every v1 VRAM figure on this page
predates the v2 architecture and is not quotable for v2
([ADR 0022](docs/adr/0022-robot-a-selection-gate.md)).

## Interview talking points

**The three corrections from 2026-08-04, and how each is proved.**

- **"A cannot see P" meant communication domains, not sightlines.** The
  requirement gate is the **isolation certificate**: P's observed graph equals
  the declared allowlist exactly, with a mutation control that goes red when the
  bridge is deliberately widened. A green certificate against a check that
  cannot fail is decoration. The geometric occlusion work is *not* retracted —
  it is true of the scene and still asserted — but since
  [ADR 0021](docs/adr/0021-police-owned-sensing-and-isolation-gate.md) it is
  scenario realism rather than the assignment's constraint.
- **A navigates autonomously.** Governed Nav2 on a live SLAM map, told B's
  address and never the path, with a safety governor between planner and wheels.
  Arrival is *contact*, and the contact is truth-measured — 2.9 mm past the
  0.2175 m modelled range. The *witness* is not: the two-signal confirmation of
  [ADR 0034](docs/adr/0034-the-mask-is-the-target.md) stands as a design while
  its laser half is measured wrong, so every run reports `ARRIVED_UNPROVEN`.
- **The AI/ML is load-bearing and measured.** A learned detector on P's own
  camera, trained on synthetic Replicator frames — and, more importantly, a
  measurement of what it is worth for enforcement rather than a detection rate
  alone. 99.3% detection answers whether a box overlaps the robot, not whether
  the box puts the robot in the right metre. Measured separately: the projection
  is +0.023 m on truth boxes, but **through the detector 19.5% of frames land
  more than 0.5 m out, and confidence cannot tell you which.**

**Engineering positions this repository will defend.**

- Speed is inferred from P's **own** camera pixels, its published calibration,
  and image acquisition timestamps. No pose, odometry, TF, depth, or simulator
  truth reaches the estimate path — and since ADR 0020 the observer is not
  merely forbidden to read truth, it is unable to: those producers live on A's
  domain and are absent from the allowlist.
- USD is generated deterministically and measured from the composed stage; the
  GUI is a consumer, not the source of truth. Variants are finite named corridor
  profiles, not arbitrary numeric sliders.
- The supplied drawing is treated as evidence about topology, not about scale;
  every metric length is declared a project choice rather than dressed up as a
  survey.
- The occlusion proof reports wall occlusion separately from being merely
  off-screen, and never relabels one as the other.
- Simulated time uses one `/clock` publisher and resets estimator state on time
  jumps. Wall time may measure external latency but never enters differentiation.
- Isaac-specific code is isolated and checked against the installed version's API.

**Where the engineering is most visible is in what was measured and refused.**

- The pose-to-render gap was carried as "uncharacterised, bounded by one camera
  period" for weeks. Measuring it showed the bound wrong by three orders of
  magnitude, and that it is not a latency but a **rate deficit** — content
  advances at 0.888× the clock. It is attributed and **not corrected for**: a
  correction derived from the run it corrects is a fitted parameter wearing a
  mechanism's clothes.
- A 19.5% detector gross-failure rate is the textbook case for robust fitting.
  It is deliberately **named and not applied**, because changing the method
  after seeing the result is tuning to the answer.
- The synthetic repro built to confirm the shared-memory deafness mechanism
  **did not reproduce it**. The fix shipped anyway, on batch evidence, with the
  negative result filed against its own hypothesis — the mechanism is bounded,
  not named.
- Two claims measured earlier in the project were **withdrawn in the evidence
  where they were made**, rather than quietly dropped.

## Explicit non-goals

Docker, Isaac Lab/RL, Unreal/Unity, large asset packs, cloud deployment, and CI
beyond the single lint-and-test workflow are out of scope for this interview
demo.

*Revised in v2:* Replicator moved **into** scope as the detector's offline
training-data renderer ([ADR 0024](docs/adr/0024-learned-enforcement-perception.md)),
and "a multi-sensor robot" now needs a qualifier — A carries a navigation lidar
as the fleet twin's contract sensor, and no camera. The scenario still has
exactly one rendered sensor, and it is P's.

## Documentation

- [Supplied task and diagram](docs/ROBO_TASK.pdf)
- [Visual project map and growth tracker](docs/README.md)
- [System design](docs/DESIGN.md)
- [Sensor-feed contract](docs/SENSOR-FEED.md)
- [Hardware and Isaac activation record](docs/ACTIVATION.md)
- [Development workflow, CI recovery, and commit history](docs/DEVELOPMENT.md)
- [Architecture decisions](docs/adr/README.md)
- [Measured evidence index](docs/evidence/README.md)

## License

Apache-2.0. See `LICENSE`.
