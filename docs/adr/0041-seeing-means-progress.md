# ADR 0041: The seeing gate demands progress, not a rate

- **Status:** Accepted
- **Date:** 2026-08-14
- **Amends:** [ADR 0037](0037-the-banner-means-seeing.md) decision 2 (the
  banner's gate) and the second call pinned by
  [ADR 0039](0039-the-lens-is-asked-twice.md). Both records' conclusions
  stand: the banner still means seeing, and the lens is still asked twice.
  What changes is what one ask reads.

## Context

The gate passed on `/healthz` reporting `rates.scan > 0`, and `rates` is a
10 s trailing window (`_lens_core.RateWindow`). A lens that heard a burst
and went deaf keeps reporting a non-zero rate for the remainder of the
window, and the gate returns on its first success.

Run `20260814-133559` is the proof it matters: its `lens.json` freeze latch
places the last message it ever received at **t≈0.25 s**, and it cleared
the seeing gate at t≈6 s — the window echoing a burst from a lens already
deaf. The run then spent its full 136 s bring-up and was refused by the
second check (ADR 0039), which is exactly the expensive path the first
gate exists to prevent.

Since the 2026-08-14 rework the lens's `/healthz` carries `counts` —
cumulative per-topic message totals, monotonic by construction.

## Decision

`lens_is_seeing` reads `/healthz` **twice, 0.6 s apart**, and passes only
if the monotonic scan count **increased** between the reads. At the
contract's ~12 Hz that is ~7 messages; a deaf, frozen, or not-yet-sampled
lens has no moving count and fails naturally, with no special cases.

Try budgets keep the old wall-clock envelopes: 18 tries (~20 s) at the
lens gate, 3 tries (~3.5 s) at the pre-mission check.

## Consequences

- A burst can no longer buy a banner. The 133559 shape is refused at the
  lens phase, before SLAM and Nav2 are spent on an unwatched run.
- A healthy lens passes in ~0.6 s — faster than the old form's first
  poll-and-sleep cycle.
- The gate now depends on the `counts` field added by the instrumentation
  commit; the seam test runs the runner's extracted snippet against the
  lens's own `healthz_payload` pairs, so the shell/Python halves cannot
  drift apart silently.
