# ADR 0037: The banner means the lens is seeing, not that it is serving

- **Status:** Accepted
- **Date:** 2026-08-14
- **Supersedes:** [ADR 0035](0035-the-lens-is-the-first-instrument.md)
  **decision 1 only** (the lens starts before the simulator), by way of that
  record's own rollback clause. Decisions 2, 3, 4 and 5 stand unchanged and
  unloosened; decision 3's refusal is extended rather than weakened.

## Context

ADR 0035 removed a class of faux launch: a run that died in bring-up used to
write no `lens.log` at all, and the banner printed a literal port
unconditionally. Six runs on 2026-08-14 confirmed the fix on its own terms —
six announced, six served, zero refusals — and the evidence notes said so.

**That measurement was of the wrong thing.** Read back from the artifacts:

| run | rows | span | samples with a resolved fit |
|---|---|---|---|
| `20260814-085419` | 1150 | 232.9 s | 695 |
| **`20260814-085821`** | **500** | **100.4 s** | **0** |
| `20260814-090216` | 1100 | 222.8 s | 696 |
| **`20260814-090613`** | **500** | **100.4 s** | **0** |
| `20260814-093434` | 1100 | 223.1 s | 695 |
| `20260814-093830` | 2000 | 405.5 s | 1553 |

**Two of six lenses resolved nothing for the entire run.** Both served
`/healthz` throughout, both were announced with a green banner, and both are
identical in shape: 500 samples, 100.4 s, every metric column null. They stop
at 100 s because `is_frozen` latched — which means each did hear *something*
early, around 40 s in, and then heard nothing for the 60 s idle window. The
runs themselves were fine: both handed off and both delivered, at 0.2263 m and
0.2262 m, inside the 3.5 mm spread of the other four. Nothing was lost except
the ability to watch them, which is the whole reason the instrument exists.

So the claim "zero faux launches" measured **serving**, and the operator was
asking about **seeing**. `/healthz` answering `ok` is the most a bound socket
can honestly claim, and the runner was reading it as proof a run was watched.

This is the worse of the two classes. The first failed loudly — no log, no
URL. This one is indistinguishable from success at the moment it matters.

**The mechanism is not identified, and this record does not claim it.** What
exists is a correlation and a boundary: lenses created *before* `simctl start`
went blind on 2 of 6 runs; lenses created *after* it are 0 of roughly 90
archived sessions. The novel churn in that window is `simctl`'s own
`sim_target` and `probe_topics` participants, which are created and
`os._exit(0)`-hard-exited every ~10 s while the lens is already joined to the
domain. Diagnosing DDS discovery is not a ship-day activity.

## Decision

### 1. `/healthz` reports whether the lens is seeing

The endpoint returns JSON from a pure `healthz_payload(state)`: `ok`, `t`,
`frozen`, and the per-topic `rates` already carried in the sampler's state.
The rates are the seeing signal — a deaf lens reports 0.0 on every topic, and
a lens with no sample yet reports `rates: null`, because *absent* and *zero*
are different facts and only one of them is a fault.

It adds no key to `build_state()`, so the stub's bidirectional key contract is
untouched. **`lens_stub.py` deliberately keeps its flat `ok`**: the stub can no
longer green-light a real run, which is the correct relationship between the
two.

### 2. The banner is gated on a non-zero scan rate

The runner polls `/healthz` for up to 20 s and prints the URL only inside the
branch where a scan rate above zero was observed. That deadline is ~240 scans
at the measured 12–15 Hz, so a lens that is merely slow to subscribe clears it
in the first second.

### 3. A deaf lens is restarted once, and then refuses the run

One retry, because the deafness has no identified mechanism and a restart is
far cheaper than a lost Isaac load. Attempt 1's `lens.log` is copied to
`lens-attempt1.log` before the retry overwrites it — it is the only artifact
the failure leaves. A second deafness is an INFRASTRUCTURE refusal naming
`--no-lens`, the same shape as 0035's never-served refusal.

### 4. The lens therefore starts after `simctl start`

Not a preference — a consequence. The gate asks whether the lens hears `/scan`,
and that question is only askable once `/scan` exists. `simctl start` has
already waited for it to publish before returning, so a lens that hears nothing
at that point is deaf rather than early. The correlation above points the same
way, and ADR 0035's rollback clause instructed exactly this move.

**It still starts before the contract precondition**, and therefore before
SLAM, Nav2, the mission, and every `rerun()` exit that made bring-up
unwatchable. That half of 0035 is the reason the block did not simply go back
where it came from, and a test pins both bounds.

## Alternatives rejected

**Diagnose the DDS discovery failure and keep the earlier placement.** The
right answer eventually, and the wrong one today. It is an open-ended
investigation into participant discovery timing, and the delivery is today.
The placement is reversible in one block if the mechanism ever turns out to
be something the earlier position was not causing.

**Gate on `/map` instead of `/scan`.** `/map` arrives when SLAM gets there,
which on a slow bring-up is tens of seconds after the gate would ask. It would
convert a real class of healthy run into refusals.

**Move the block without adding the gate** — the brief's own safe ordering.
It would have prevented both observed failures, and it would have left the
banner still meaning "a socket answered". The next deaf lens by any other
mechanism would be just as invisible, and the operator's condition was that
the launch itself become trustworthy.

**Keep `/healthz` flat and have the runner subscribe to `/scan` itself.** It
measures a different process's view of the domain, which is precisely the thing
that was wrong. The question is whether *the lens* hears, so the lens must be
the one answering.

## Consequences

**The ~61 s Isaac load is now unwatched, and a lens refusal throws it away.**
Paid deliberately. All three recorded bring-up deaths are after the load;
`simctl start` fails loudly on its own and classifies itself; and a blind lens
does not merely fail to help — it decorates a run with a page that looks alive.

**The previous run's lens now lingers through `simctl start`** rather than
being reaped before it, because `reap_previous_lens()` moved with the block.
Harmless: it is frozen by then, it publishes nothing, and its dump is already
complete. Its `lens.json` may acquire a bounded tail of post-run samples, which
is the same property 0035 already accepted.

**The gate is a check at one moment, not a guarantee for the run.** A lens that
hears scans at t=5 s and goes deaf at t=40 s passes it. Both observed failures
would have been caught, because both were deaf from the start of the window
that matters — but the general case is not covered, and this record does not
pretend otherwise. The page shows `frozen` and the dump shows the null columns,
so the failure is *visible* afterwards; making it *loud* afterwards is a
separate, additive change on its own evidence.

## Validation

Both directions, live, before any Isaac time was spent:

| | |
|---|---|
| Real lens, empty domain 69 | `{"ok": true, "t": 3.245, "frozen": false, "rates": {"scan": 0.0, ...}}` — the runner's own extracted snippet exits **1**, refusing |
| Same lens, fed 12 Hz of `LaserScan` | `"scan": 12.062` — the snippet exits **0**, accepting |

The 12.062 against a published 12.0 also confirms the rate window the gate
reads is accurate to 0.5%, which matters because the gate's threshold is a
comparison against zero and a broken window would read zero forever.

The shell/Python seam is tested the way the announcement seam is: the runner's
snippet lifted out of the script and executed against the lens's own
`healthz_payload` output for the seeing, deaf, frozen and no-sample-yet cases.
Neither side's unit tests can see that contract, and drift in it means either
refusing every healthy run or accepting every deaf one.
