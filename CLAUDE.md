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

## Interview objective and definition of done

This repository exists to support a live NVIDIA Omniverse engineering interview,
not to become a production traffic-enforcement platform. The primary deliverable
is a short, reliable, visually understandable demonstration backed by enough
evidence to defend its engineering decisions.

An interviewer should understand in one run:

1. A travels from the tapered corridor toward B on the next street.
2. The corridor narrows toward the corner and the local demonstration speed limit
   becomes stricter.
3. P is physically hidden behind the corner wall and cannot be seen by A's camera.
4. P nevertheless receives A's one permitted RGB camera stream over ROS 2.
5. Surveyed fiducials let P estimate station and speed without pose, odometry, TF,
   depth, or simulator truth.
6. The UI makes measured speed, uncertainty, local width, limit, and violation
   state obvious.
7. Changing the corridor-width USD variant visibly changes the geometry and policy
   while preserving the scenario invariants.

Interview-ready means:

- one documented command starts the demonstration;
- A moves continuously on the authored route;
- the camera-only observer demonstrates a compliant run and one continuous
  speeding episode;
- P's concealment is visible and backed by the geometric certificate;
- one camera remains the only simulated sensor;
- the RTX 5070 Ti stays within the recorded memory budget;
- a recorded fallback is available if the live run fails.

Use this decision filter for new work:

- Prioritize a working, explainable, rehearsable demo over production-scale
  generality.
- Add complexity only when it strengthens a headline interview claim or prevents
  a demonstrated failure.
- Do not create a new blocking gate for a low-severity documentation or polish
  issue; correct it additively without derailing the current milestone.
- Do not reopen accepted decisions without new contradictory evidence.
- Keep large fiducials, overlays, and other scene elements visually credible;
  numerical success alone is insufficient if the viewport looks contrived.
- Once a milestone's stated acceptance criteria pass, commit it and advance to
  the next visible capability.

## Agent roles

Claude is currently the implementation/planning agent. Codex independently
reviews completed milestone commits. Claude should not repeatedly restart the
independent audit unless the user explicitly assigns that role.

The active sequence is:

1. close confirmed correctness defects;
2. restore camera-derived enforcement coverage at the corner;
3. requalify that final static geometry on the GPU;
4. implement deterministic motion;
5. connect the live observer and visualization;
6. rehearse and harden the interview demonstration.

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

## Current handoff: restore enforcement coverage, requalify, then move A

Read [`docs/HANDOFF.md`](docs/HANDOFF.md) first. It records the exact commit
range through `d19d02f`, the current 106-repository plus 61-package test result,
review findings R1–R10, and the limits that are not yet closed.

Stage 0 is closed. The next work is enforcement coverage, not another review
cycle.

**Robot motion must not start yet.** Two things block it, and neither is a
matter of taste:

1. **There is no canonical static qualification.** The recorded run predates the
   renderer readback fix and reported a *requested* mode as measured. Its
   summary is preserved unmodified as
   `qualification-summary-v1-request-echo-invalidated.json`, and no replacement
   is published until a fresh paired run passes. Motion evidence built on that
   baseline would inherit the same defect.
2. **Enforcement coverage does not reach the corner.** Past camera x ≈ 7.5 the
   wall markers fall outside the 75° frustum, so gates 8.0 and 10.0 can never
   produce an estimate and the tightest rule — 0.8 m/s at x ≥ 10 — cannot be
   exercised from camera evidence. This is a renderer-independent FOV
   obstruction, not a rendering-quality problem.

| Status | Slice | Required evidence before continuing |
|---|---|---|
| Done | Static ArUco rendering probe | Nominal profile passes five production-pixel dwells with an actual-capture mirror control. Its renderer claim is invalidated; its pixel and station results stand. |
| Done | Renderer/camera contract correction | `5bc1c99` reads the render mode back, `0c4e9b8` gates encoding and aligns the principal point behind a 0.05 px criterion, `3d9a754` covers every rejecting branch portably. |
| Next | Restore enforcement-gate coverage | Add reference fiducials on the north-wall extension and the east building face — perpendicular planes, so combined correspondences stay non-coplanar. Split marker roles so references never become phantom gates. Re-run the occlusion certificate for all three profiles. |
| Then | Requalify on GPU | Fresh paired capture of the corrected geometry, with dwells sampling the weak two-tag band and the previously unreachable region. Only then does a canonical static qualification exist again. |
| Then | Deterministic robot motion | Move `/World/Actors/A` continuously along the authored line-arc-line trajectory with position and yaw derived from route station; drive it from simulation time with a configured path-speed profile; reset safely after a corridor-variant change. Thresholds come from the requalification, not from synthetic extrapolation. |
| Later | Live camera-to-observer qualification | Feed only the camera contract to `police_observer`. Keep simulator truth in a separate evaluator. Demonstrate 1.0 m/s without a violation, 1.8 m/s with exactly one violation, acceleration through the limit, and dropped/single-marker frames; report speed error, usable-frame coverage, and latency. |
| Later | Interview visualization | Show active `(m,n)`, measured speed and uncertainty, local width and speed limit, violation state, A's route, P's location, and the blocking-wall/certificate result. An ordinary viewport may explain the scene, but it must not become a second sensor or ROS render product. |
| Last | Demo hardening | Provide one documented launch path, repeat the VRAM measurement on the RTX 5070 Ti, preserve failure evidence, document the Ubuntu fallback, and tag the interview-ready release only after the gates pass. |

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

The current incoming reviewer is Claude. `docs/HANDOFF.md` is the operative
checklist. Do not squash or rewrite history for this handoff. Report:

1. every new commit hash and subject, in order;
2. files changed and any deviation from the milestone order above;
3. exact verification commands with test counts and saved artifact/log paths;
4. measured camera rate, resolution, estimator error/coverage/latency, and the
   exact VRAM sampling method where applicable;
5. known failures, assumptions, skipped checks, and claims that remain
   provisional; and
6. confirmation that observer-side source and topic audits still exclude pose,
   odometry, TF, configured speed, and other simulator truth.

Passing self-written tests is not the independent review. Leave the worktree
clean and the evidence reproducible so Claude can inspect the implementation,
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
| Rendered frame, log, or measured artifact | `docs/evidence/<topic>/` |

ADRs are immutable once accepted. Amend or supersede with a new ADR; do not edit
a decision after the fact.

### Evidence artifacts

- Generate bulk, intermediate, and nondeterministic output under
  `out/evidence/<topic>/`; tools must not overwrite committed evidence by
  default.
- Promote only representative frames and stable summaries into
  `docs/evidence/<topic>/`, in the documentation commit that records the
  measured result.
- Every topic directory under `docs/evidence/` must contain `NOTES.md` with the
  exact command, Isaac version, GPU, resolution, frequency, settings, and
  pass/fail result. The evidence index itself is exempt.
- Curate artifacts. A frame without provenance is decoration, and an unbounded
  log dump is not a reviewable result.

## Commit conventions

- Conventional Commits: `feat(scope):`, `test:`, `docs:`, `ci:`, `chore:`.
- Each behaviour commit carries its own direct tests.
- Documentation commits record measured evidence, not promised outcomes.
- Additive history only. Do not rewrite published commits.
