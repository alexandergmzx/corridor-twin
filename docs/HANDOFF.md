# Incoming review handoff

| Field | Value |
|---|---|
| Handoff version | 2.0.0 |
| Prepared | 2026-07-27 |
| Branch under review | `audit/reconcile-docs-and-decisions` |
| Published base | `f992470` on `origin/main` |
| Range to audit | `origin/main..origin/audit/reconcile-docs-and-decisions` — enumerate it, do not trust a count written here |
| `main` | Untouched at `f992470` until this review passes |
| Portable gate | `bash tools/check_workspace.sh`. Last clean run 2026-07-27: ruff pass, 124 repository tests plus 1 skipped without the colcon-generated interfaces, 3 packages built, 96 ROS package tests |

> **Read [`REVIEW-LOG.md`](REVIEW-LOG.md) before raising a finding.** Two audit
> rounds have already run. Three findings were resolved differently from what
> the audit prescribed — in one case the prescription would have broken the
> build — and eight items are deliberately open. Disagreeing with a recorded
> disposition is welcome; re-deriving one is the waste that log exists to
> prevent.

## What this branch does

It responds to two independent audits. Neither found a broken engineering
invariant: the occlusion gate, the one-camera budget and truth isolation all
held under scrutiny. What they found was documentation that had drifted out of
agreement with the code, and then a cluster of real defects around the speed
policy.

The documentation drift ran in the unusual direction — the repository was
**underclaiming**. Three of the four entry points still said robot motion was
blocked and live estimation unimplemented, several commits after both shipped
with measured evidence.

The code defects were not cosmetic. The demonstration's most hand-edited value,
the speed policy, sat on its least-protected path: reversing its rule list — a
semantically identical set — silently deleted the corner rule and produced a run
with zero violations and no error.

## Commits under review

```bash
git log --reverse --format='%h %s' origin/main..origin/audit/reconcile-docs-and-decisions
```

| Commit | Boundary | Why it is separate |
|---|---|---|
| `5a3a083` | Decisions | ADR 0015 (reference fiducials) and ADR 0016 (the owner-approved 3.5 → 4.0 m policy boundary). Written before the documents that cite them, so no commit forward-references a missing file |
| `43db0fc` | Status reconciliation | Every claim corrected against evidence, plus a test so an evidence topic cannot go unlisted again |
| `ab09aff` | Truth-isolation guard | Structural, not textual. See M1 in the review log — the audit's own example did not hold and its prescribed fix would have failed the build |
| `b5194bf` | Policy correctness | Normalization and validation on both sides of the manifest. The highest-severity behavioural change in the range |
| `55eb69e` | Coverage reporting | A field that could only ever read false becomes a detector that moves |
| `dbcf57e` | Supporting tools | Three small independent defects, each with a direct regression |
| `4567d78` | This handoff | The review record, the rewritten handoff, and a test so its header cannot go stale a third time |

For context, the twelve commits from `a416e47..f992470` already on `main` are
the demonstration itself — motion, the RViz enforcement view, the one-command
launch, and the measured live run in
[`evidence/live-demo/NOTES.md`](evidence/live-demo/NOTES.md).

## Audit checklist

### 1. Establish the exact tree

```bash
git status --short --branch
git log --reverse --format='%H %s' origin/main..HEAD
git diff --check origin/main..HEAD
git diff --stat origin/main..HEAD
```

Do not push, rebase, squash, or amend as part of the audit. Confirm every commit
uses the repository's configured Alexander Gomez identity and carries no
assistant attribution trailers.

### 2. Re-run the portable gate

```bash
bash tools/check_workspace.sh
```

The ament run prints deprecation warnings about implicit `None` returns. They
are not failures, and they should not be relabelled as a clean stderr stream
either.

### 3. Challenge this range specifically

Against implementation, not prose:

1. **Two policy validators, one invariant.** `scene.model._validate_speed_policy`
   and `police_observer.estimator.normalized_speed_rules` implement the same
   rule because `corridor_scene` cannot import `police_observer`.
   `test_speed_policy_validation_agrees_across_packages` is the only thing
   holding them equal — is its case list strong enough, and can you construct an
   input where the two disagree?
2. **Normalization versus rejection.** Rules are sorted rather than required to
   be written ascending. Does sorting hide a configuration error a reader would
   want to see?
3. **Fail-at-construction.** `MarkerMap.assert_policy_covers_the_corridor`
   checks entry, corner and every gate width. Is there a reachable width it
   misses — and can `limit_at` still raise from inside `_on_frame`?
4. **Subscription enumeration.** `_constructed_subscriptions` parses
   `create_subscription` and `message_filters.Subscriber`. Is there a third way
   this package could subscribe that the walk would not see?
5. **The coverage flag.** It must be able to read false. Break a gate and
   confirm the test fails.
6. **The ADRs against the code.** ADR 0015's four load-bearing properties and
   ADR 0016's boundary should each be checkable in `corridor.yaml` and the
   estimator.
7. **Reconciled documents in the other direction.** Round 1 fixed
   underclaiming. Do any of them now *overclaim*? The static requalification and
   the pose-to-render latency must still read as unclaimed everywhere.
8. **The staleness test.** `test_handoff_header_matches_the_actual_tree` exists
   because this header went stale twice. Does it actually fail when the header
   drifts?

### 4. Report before continuing

Report findings first, ordered by severity, with file and line references. Treat
committed tests and evidence as claims to challenge, not as proof. If a defect
is confirmed, correct it in a new focused commit with a direct regression; do
not rewrite or squash existing history.

Preserve the invariants in [`../CLAUDE.md`](../CLAUDE.md) throughout — one
camera, truth isolation, and the geometric proof that A cannot see P.

## Next implementation slice

The **paired static requalification**. There is still no canonical static
qualification: the recorded dwell run predates the renderer readback fix and
reported a requested mode as measured. Its summary is preserved unmodified as
`qualification-summary-v1-request-echo-invalidated.json`. The live demonstration
does not replace it — a paired dwell capture with its own mirror control is a
different measurement.

## Known open

Listed once, with reasons, in [`REVIEW-LOG.md`](REVIEW-LOG.md#known-open--please-do-not-re-raise-as-new).
Summarised: no canonical static qualification; uncharacterised pose-to-render
latency; R17 (marker 84 half behind the corner mass on the default profile);
R11 (no runtime profile reload); the violation's lack of redundancy; live
coverage limited to one profile at one speed; and C4, C5, L4, M2.
