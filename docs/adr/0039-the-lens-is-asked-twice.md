# ADR 0039: The lens is asked twice, because the placement does not prevent deafness

- **Status:** Accepted
- **Date:** 2026-08-14
- **Extends:** [ADR 0037](0037-the-banner-means-seeing.md) decision 3 — a lens
  that cannot see refuses the run. The rule is unchanged; it is applied at a
  second point. **Corrects 0037's decision 4 rationale**, which is a factual
  correction rather than a reversal: the placement stands, its stated
  justification does not.

## Context

ADR 0037 was accepted this morning on a correlation. Two of six lenses created
*before* `simctl start` served `/healthz` for a whole run and resolved nothing;
no lens created *after* it had ever been observed to do that, across roughly 90
archived sessions. The record was explicit that the mechanism was unidentified
and that the correlation was what there was.

**Run `20260814-125254` is the counterexample.** Its lens was created 71 s
after `simctl start`, passed the seeing gate, printed its banner, and went deaf
within seconds: 300 samples over 60.2 s of a 250 s run, every metric column
null, frozen from the 60 s idle rule onward.

The run was otherwise good. The handoff fired, A reached 0.2251 m from B —
inside the 3.5 mm spread of the six runs before it — and the two gate failures
it recorded (EKF gap 1.063 s, map-frame goal error 370 mm) are known open items
untouched by anything in this session. **The delivery was fine and unwatchable.**

So the placement is not protective. What caught this was the `lens_resolved_frac`
covariate added hours earlier, which printed **THE LENS SAW NOTHING** and
stamped a `manifest_error` — after the run, which is when a covariate speaks.

## Decision

**The seeing check is one function, `lens_is_seeing`, called twice: when the
lens starts, and again immediately before the transit recorder.**

The second call is a refusal on the same terms as the first, naming `--no-lens`.
Six polling attempts rather than forty: by that point the instrument has already
proved it can hear, so this is a liveness question and not a startup race, and
a 20 s poll before every mission would tax every healthy run.

**One definition, asserted to be one.** Two copies of a predicate like this
drift, and the run then gets judged by whichever is laxer. A test pins the
single definition and the two call sites.

### Why this point

Bring-up is roughly 130 s of the run. Everything before the transit recorder is
setup; everything after it is the thing worth watching. A lens that dies just
after passing its gate has been dead for all of bring-up and will be dead for
all of the mission, and asking once more at the last moment before the robot
moves converts that from a post-run covariate into a refusal — at the cost of
the bring-up already spent, and nothing else.

## What this does not do

**It does not stop the lens going deaf.** The mechanism remains unidentified.
This makes the run refuse instead of quietly producing an unwatchable success,
which is the property the operator asked for; it is not a fix for the
underlying DDS behaviour, and it should not be quoted as one.

**It does not close the general case either.** A lens that dies *during* the
mission still passes both checks. `lens_resolved_frac` in `run.json` remains
the backstop for that, and it remains a covariate — a broken instrument must
not destroy good navigation evidence, which is the ruling ADR 0037 already made
and the reason both blind runs of the morning were allowed to complete.

## Consequences

Two refusal points instead of one, so a marginal lens costs a bring-up rather
than a whole run. The failure mode this introduces is a false refusal — a lens
briefly starved of scans at exactly the wrong moment — and the six-attempt
window over three seconds is what stands between that and a lost run. If false
refusals ever appear, the window is the thing to widen, not the check to remove.

**0037's decision 4 keeps its conclusion and loses its reason.** The lens still
starts after `simctl start`, because the seeing gate cannot be asked before
`/scan` exists and that argument is untouched. What is retired is the claim
that the position itself avoids the failure. Anyone re-reading 0037 for
guidance on *where* the problem comes from should read this record instead.
