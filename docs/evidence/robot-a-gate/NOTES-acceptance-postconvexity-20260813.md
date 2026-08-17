# Four consecutive deliveries, after the decoy was closed

**This is correction 2's evidence, and it is not a gate.** Read the caveats
before the numbers; two of them are load-bearing.

## Command

    bash tools/diagnostics/run_batch.sh 3 nominal_m6_n3
    bash tools/diagnostics/run_batch.sh 2 nominal_m6_n3 wide_corner_m6_n4_5 uniform_m6_n6

which invokes, per run:

    bash tools/corridor_profile_run.sh --profile nominal_m6_n3 --robot robot1 \
      --domain 67 --allow-contract-fail

Summarised by `tools/diagnostics/batch_summary.py`. Promoted artifact with
per-file sha256: [`acceptance-postconvexity-20260813.json`](acceptance-postconvexity-20260813.json).

| | |
|---|---|
| Isaac | 5.1.0.0, RTX 5070 Ti |
| Robot | robot1 (ADR 0027), domain 67 |
| Code | `b915f81`, convexity fix `2a4e706` in |
| Profile | `nominal_m6_n3` |
| Lens | up on every run, `http://127.0.0.1:8765/` |

## Result

| run | closest approach | walked away | refinements | docked on |
|---|---|---|---|---|
| 184725 | 0.0592 m | 0.000 | 1 | B |
| 185113 | 0.1332 m | 0.000 | 1 | B |
| 190257 | **0.0458 m** | 0.000 | 1 | B |
| 190648 | 0.1335 m | 0.000 | 1 | B |

**Four consecutive nominal runs, all within 0.15 m of the delivery standoff,
all staying put, all docked on the real B.**

`docked_on` is **derived, not asserted**: the detected range at DOCKED is
compared against the true distance from A's final position to B and to the
`EastWallStub` decoy, and the nearer match must agree within 0.15 m. Residuals
were 26.4, 12.2, 1.2 and 1.7 mm — the laser and the evaluation plane agree to
within a couple of centimetres on every run.

For contrast, the same measurement across the *whole* of 2026-08-13: before the
convexity fix, **5 of 10 docked runs latched the decoy** and parked 0.45–0.63 m
from B. After it, **8 of 8 docked runs across all three profiles** were on B.
See [`NOTES-the-eastwallstub-decoy-20260813.md`](NOTES-the-eastwallstub-decoy-20260813.md).

## What this is not

- **Not a gate.** Every run used `--allow-contract-fail`, which is the runner's
  intended corridor invocation — `/scan` runs 14–16 Hz against a declared 12 in
  the stock yahboom arena too — but it means each artifact carries the caveat and
  none of this is gate evidence.
- **Not measured against a pinned criterion.** ADR 0033 was unwritten when these
  ran. The 0.15 m used here is *proposed*, and the proposal itself has an open
  problem: identical dock ranges (0.6124–0.6200 m across these four) produce
  closest approaches from 0.0458 to 0.1335 m, because `closest_approach_m` is a
  minimum over the whole run and so partly reflects how near A's trajectory
  swung to the standoff *point* rather than where it stopped. Whether that is
  the right criterion is ADR 0033's question.
- **Not the whole population.** These four are the consecutive passes. The same
  day also produced two post-convexity nominal runs at 0.1703 and 0.1761 m —
  just over — and one non-start. Selecting the consecutive four is the standard
  the brief asked for, not the average.
- **Map-frame error is reported and ungated**, per the ADR 0029 diagnosis: it is
  1.2–2.3 m on these runs and it is not a delivery measurement. It is in the
  JSON as `map_frame_failures_ungated`.

## Why these four and not the earlier three

An earlier three-consecutive set exists (0.1147, 0.1007, 0.0354 m) and is
quoted in the delivery-close handback. **Those predate the convexity fix.** They
are genuine deliveries onto B, but they ran on code that latched the decoy half
the time and happened not to on those three. The four here ran on the code that
ships.
