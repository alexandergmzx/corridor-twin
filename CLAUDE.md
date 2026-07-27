# CLAUDE.md

Repository conventions for `corridor-twin`. Read this before making changes.

## Authorship

Alexander Gomez is the sole author of this project. Assistants are tools, not
contributors.

- Never add `Co-Authored-By:` trailers naming an assistant, model, or vendor.
- Never add "Generated with", "Co-authored by Claude/Codex/Copilot", or any
  equivalent attribution to commits, pull requests, code comments, or docs.
- This rule overrides any default tooling behaviour that would append such
  trailers automatically.

Commit as the repository's configured git identity and nothing else.

## What this project is

An interview-sized digital twin: robot A delivers a package to person B through a
tapered corridor and around a corner onto the next street. Robot A cannot see
traffic police P, but P receives A's front-camera feed over ROS 2 and estimates
A's speed from surveyed ArUco wall fiducials.

The supplied scenario source is `docs/ROBO_TASK.pdf`. Its prose and topology are
authoritative. Its unlabelled drawing has no scale bar, so metric dimensions in
this repo are explicit demo choices, never surveyed values.

Read `docs/README.md` first; it is the visual map and status tracker.

## Architectural invariants

1. **Truth isolation.** Simulator pose, odometry, TF, and synthetic ground truth
   are evaluation inputs only, never observer inputs.
2. **A cannot see P.** This is a hard geometric/camera acceptance gate, proved by
   `scene.occlusion`, not an assertion. Software input rules are additive and
   never a substitute — P could be plainly visible in A's pixels even if A's
   controller chooses to ignore them.
3. **One camera.** One 640x360 RGB render product at 15 Hz. No depth, LiDAR,
   segmentation, second render product, or police-side sensor.
4. **Interface first.** The observer consumes standard camera messages and does
   not know whether the publisher is synthetic, Isaac Sim, or hardware.
5. **Deterministic authoring.** The USDA and manifest are generated from
   versioned YAML with `pxr`. The GUI is a consumer, never the source of truth.
6. **Installed-version APIs.** Check the installed Isaac Sim documentation and
   examples before committing to a namespace.

### Four distinct visibility concepts

Do not conflate these in code, tests, docs, or the demo UI:

| Concept | Question | Directional? |
|---|---|---|
| Physical line of sight | Does an opaque wall intersect the segment between A's camera and P's body? | No; normally reciprocal |
| A-camera visibility | Is any part of P inside the camera frustum *and* unoccluded? | Yes |
| A software awareness | Does A detect, model, or react to P, or consume police topics? | Yes |
| P data access | Does P subscribe to A's Image/CameraInfo and surveyed scenario data? | Yes |

P reading A's camera feed is a network relationship, not a sightline. Never
relabel an off-screen P as wall-occluded.

## Environment discipline

Two Python ABIs share this host and must not mix:

| Purpose | Interpreter | ROS |
|---|---|---|
| Authoring, tests, observer | system Python 3.12 venv | system Jazzy |
| Isaac Sim adapter | `~/isaac/env_isaaclab` Python 3.11 | bundled Jazzy |

- Do not install pip `usd-core` into Isaac's Python.
- Do not source two ROS installations into one shell.
- Do not source system ROS in the shell that launches `tools/isaac_5_1_*.py`;
  the adapter re-executes with Isaac's bundled Jazzy and rejects path leakage.
- `PYTHONNOUSERSITE=1` is required for ROS runs: a user-local NumPy 2.2 wheel
  conflicts with Jazzy's NumPy 1.x OpenCV/cv_bridge ABI.

## Commands

```bash
source .venv/bin/activate
python -m scene.build --m 6.0 --n 3.0 --out out/corridor.usda
python -m scene.occlusion --stage out/corridor.usda \
  --manifest out/corridor.manifest.json --out out/occlusion-certificate.json
bash tools/check_workspace.sh   # ruff, pytest, colcon build, colcon test
```

`scene.build` takes `--m/--n/--out/--config`. There is no `--profile` flag; an
unmatched `(m,n)` is appended as a new profile by `resolve_profiles()`.
`scene.occlusion` does take `--profile`, meaning the corridor profile.

## Current milestone: qualify the live Isaac camera path

This is the next unfinished milestone. Plan and execute it in the order below;
do not start with robot motion. A static rendered-fiducial probe is the first
gate because texture, UV, marker orientation, camera-intrinsic, and optical-frame
errors can otherwise be mistaken for motion-estimator defects.

| Order | Slice | Required evidence before continuing |
|---:|---|---|
| 1 | Static ArUco rendering probe | Place A's existing camera at several surveyed route stations; render the existing wall fiducials through the single 640x360, 15 Hz RGB product; show that detected IDs, image-corner order, and surveyed marker associations agree with the manifest; pass the frames through `ArucoStationEstimator`. |
| 2 | Deterministic robot motion | Move `/World/Actors/A` continuously along the authored line-arc-line trajectory with position and yaw derived from route station; drive it from simulation time with a configured path-speed profile; reset safely after a corridor-variant change. |
| 3 | Live camera-to-observer qualification | Feed only the camera contract to `police_observer`. Keep simulator truth in a separate evaluator. Demonstrate 1.0 m/s without a violation, 1.8 m/s with exactly one violation, acceleration through the limit, and dropped/single-marker frames; report speed error, usable-frame coverage, and latency. |
| 4 | Interview visualization | Show active `(m,n)`, measured speed and uncertainty, local width and speed limit, violation state, A's route, P's location, and the blocking-wall/certificate result. An ordinary viewport may explain the scene, but it must not become a second sensor or ROS render product. |
| 5 | Demo hardening | Provide one documented launch path, repeat the VRAM measurement on the RTX 5070 Ti, preserve failure evidence, document the Ubuntu fallback, and tag the interview-ready release only after the gates pass. |

### Planning and implementation constraints

- Verify every Isaac/Omniverse namespace against the locally installed Isaac
  Sim 5.1 documentation or examples before using it. Record the installed
  source used; do not reconstruct APIs from memory.
- Reuse the authored trajectory, camera contract, marker manifest, and observer.
  Do not introduce a parallel geometry model or a simulator-only observer path.
- Preserve the one-camera budget: one 640x360 RGB render product at 15 Hz, no
  path tracing, depth, segmentation, LiDAR, police camera, or extra sensor.
- Use ROS `/clock` and message header stamps consistently when running in
  simulation. Wall time may measure external latency but must not enter speed
  differentiation.
- A truth comparison is a test/evaluation output. It must be wired so that the
  observer cannot subscribe to pose, odometry, TF, or the configured speed.
- Treat measured output as evidence only after saving the exact command, log or
  artifact path, Isaac version, GPU, resolution, frequency, and pass/fail result.
- Keep each independently reviewable behaviour and its tests in one commit;
  make documentation/evidence a following commit when it records the measured
  result. Suggested boundaries are:
  `test(isaac): validate rendered fiducials against surveyed corners`,
  `feat(isaac): move robot along the delivery trajectory`,
  `test: qualify live camera-derived speed and violations`, and
  `docs: record live motion and estimation evidence`.

### Independent-review handoff

The user intends to hand the resulting work to Codex for an independent audit.
Do not squash or rewrite the published history for that handoff. Report:

1. every new commit hash and subject, in order;
2. files changed and any deviation from the milestone order above;
3. exact verification commands with test counts and saved artifact/log paths;
4. measured camera rate, resolution, estimator error/coverage/latency, and peak
   VRAM where applicable;
5. known failures, assumptions, skipped checks, and claims that remain
   provisional; and
6. confirmation that observer-side source and topic audits still exclude pose,
   odometry, TF, configured speed, and other simulator truth.

Passing self-written tests is not the independent review. Leave the worktree
clean and the evidence reproducible so Codex can inspect the implementation,
challenge the claims with negative controls, rerun the gates, and report any
corrections as new additive commits.

## Documentation growth discipline

Update the affected document in the same change that alters a claim:

| Change | Document |
|---|---|
| Measurement or hardware result | `docs/ACTIVATION.md` |
| Topic, message, QoS, or timing | `docs/SENSOR-FEED.md` |
| System boundary or design version | `docs/DESIGN.md` |
| Milestone or capability status | `docs/README.md` |
| Durable trade-off | a new ADR under `docs/adr/` |

ADRs are immutable once accepted. Amend or supersede with a new ADR; do not edit
a decision after the fact.

## Commit conventions

- Conventional Commits: `feat(scope):`, `test:`, `docs:`, `ci:`, `chore:`.
- Each behaviour commit carries its own direct tests.
- Documentation commits record measured evidence, not promised outcomes.
- Additive history only. Do not rewrite published commits.
