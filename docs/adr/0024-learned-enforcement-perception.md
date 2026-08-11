# ADR 0024: Enforcement perception is a synthetic-data-trained detector with a fiducial baseline

- Status: Accepted
- Date: 2026-08-11
- Source: interview feedback of 2026-08-04 (AI-usage correction);
  [`docs/v2-plan.md`](../v2-plan.md).
- Builds on [ADR 0021](0021-police-owned-sensing-and-isolation-gate.md): the
  perception problem is *detect and track robot A in P's frames*.
- **Supersedes the estimation method of
  [ADR 0002](0002-camera-only-speed-observation.md)** — surveyed wall markers
  → camera station → gate-crossing interpolation — completing the transition
  0021 began when it moved camera ownership to P. 0002's discipline (evidence
  is camera pixels only; truth is an evaluation input) is retained by both
  pipelines below.
- **Inverts the placement decision of
  [ADR 0013](0013-size-fiducials-from-delivered-camera.md) and supersedes its
  render-product contract**: the fiducial moves from the walls to robot A's
  body, sized by the same delivered-camera method, now for P's camera; and
  0013's decision to keep one 640×360 render product at 15 Hz — including its
  rejection of a resolution increase — is superseded by decision 5 below,
  which makes resolution a measured parameter. 0013's sizing method is
  retained, and its rejection of a *second* camera is upheld (0021 keeps
  exactly one render product).
- Extends [ADR 0007](0007-speed-policy-and-violation.md) /
  [ADR 0014](0014-violation-episode-semantics.md): evidence and episode
  semantics unchanged.

## Context

The task author's third correction was direct: the team is an AI/ML team, AI
is to be used actively, and v1's enforcement pipeline contained none —
fiducials and geometry only. ADR 0002's method was explicitly classical
(ArUco corners, PnP, surveyed gates), so satisfying this correction is a
supersession of that method, not an extension.

v2's enforcement runs on P's own camera (ADR 0021), which makes the
perception problem *detect and track robot A in P's frames* — a problem with
a natural learned solution and a natural classical baseline, on the platform
whose synthetic-data tooling is the target team's stated stack.

## Decision

1. **Learned detector, synthetic-first.** P's perception is a detector for
   robot A fine-tuned on a dataset rendered from **this scene** via
   Replicator: domain-randomized lighting, materials, robot pose along the
   route envelope, distractor props. Dataset generation scripts and the
   dataset manifest are committed evidence. Before any dataset code is
   written, the Replicator namespaces are verified against the installed
   Isaac Sim 5.1 documentation and examples, per the repository's
   installed-version rule; the verified source is recorded with the scripts.
2. **Permissive licensing only.** Detector family from YOLOX / RT-DETR /
   NVIDIA TAO `[to pin at training spike]` — and because this record is
   immutable once accepted, the pin lands as its own short record with the
   spike evidence, never as an edit here. Ultralytics-YOLO is rejected for
   this artifact on AGPL grounds.
3. **Classical baseline retained for A/B.** An ArUco plate mounted on robot
   A, sized from P's delivered camera intrinsics by the ADR 0013 method; pose
   via PnP. Every speed figure is reported twice — learned vs baseline —
   against evaluation-plane truth.
4. **Speed from track, scale from known geometry.** Image-plane track →
   metric speed via known target geometry (marker edge for the baseline;
   chassis dimensions or keypoints for the detector) plus gate-crossing
   timing. Monocular, no depth sensor — scale from a known object, which is
   also the honest answer to the depth question on record.
5. **P camera resolution is a measured parameter.** 640×360 is expected to be
   insufficient at far gates; resolution is chosen by measured detection
   range per gate, bounded above by the crossing's measured throughput
   ceiling from the ADR 0026 session, and the VRAM budget is re-measured at
   the chosen setting. Both numbers enter the v2 requalification table.

## Consequences

- The enforcement report gains a model card: dataset size and randomization
  ranges, training configuration, mAP on held-out synthetic, and per-gate
  speed-error deltas vs the baseline.
- The wall plates of 0013/0015 are retired from the evidence path; their
  disposition as scenery is a scene-change note, not a new decision.
- A real-image eval set is out of scope until a physical robot A exists; the
  synthetic-to-real gap is named in the report rather than papered over.
- The v1 estimator's gate-crossing and episode logic (0007/0014/0016
  semantics) remains the auditable frame both pipelines feed; the learned
  component does perception, not policy.

## Alternatives considered

- **Classical-only pipeline.** Rejected: it is the corrected miss.
- **End-to-end learned speed regression.** Rejected: unverifiable against the
  episode semantics of 0007/0014; detector + geometry keeps the violation
  logic auditable while the learned component does the perception.
- **Real-photo training data.** Rejected for v2 scope: no physical robot A
  exists in-window, and synthetic-first is precisely the Omniverse story the
  artifact exists to tell.
- **Ultralytics-YOLO for convenience.** Rejected on license grounds alone;
  capability was not the question.
