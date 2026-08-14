# T1: ten runs, zero deaf lenses — ADR 0040 accepted on this batch

**The deafness is gone under UDP-only: 10/10 lens-healthy** against a
baseline of 2 deaf in 4 recent runs (2/6 on the morning placement).
Operator raised the batch floor to ≥10 before leaving; the rung's 4/4
extended to 10/10 with no rung change.

## Environment

- 2026-08-14, 15:17–15:56 CST. RTX 5070 Ti host, Isaac Sim 5.1 backend via
  fleet `simctl` (robot1, corridor arena `nominal_m6_n3`), ROS 2 Jazzy,
  `rmw_fastrtps_cpp`.
- Command, per batch (from the symlinked fleet path):
  `BATCH_RUN_FLAGS="--corridor-slam" bash tools/diagnostics/run_batch.sh 4 nominal_m6_n3`
  then `… 6 nominal_m6_n3` — 10 sequential full runs, each a cold Isaac
  session under `/tmp/fleet-isaac.lock`.
- Configuration under test = commits `c838e9c…02451c9` on
  `bringup-rework-2026-08-14`: UDP-only transport session-wide
  (`ground_station/fastdds_udp_only.xml`, both env names — ADR 0040),
  `--no-rviz` default, progress-based seeing gate (ADR 0041),
  matched-event logging, per-phase durations + `/dev/shm` census in
  `run.json` (schema 1.1.0).

## The ten runs

Artifacts: `out/evidence/robot-a-gate/20260814-<id>-robot1-nominal_m6_n3/`.

| run | lens frac | restart / refusal | shm pre/post/teardown | simctl (s) | goal at (s) | odom_laser Hz | EKF Hz | gate.json failures |
|---|---|---|---|---|---|---|---|---|
| 151747 | 0.858 | none | 2/2/0 | 63 | 116 | 13.69 | 9.99 | EKF gap 1.438 s |
| 152148 | 0.843 | none | 0/0/0 | 53 | 104 | 13.95 | 9.98 | EKF gap |
| 152508 | 0.859 | none | 0/0/0 | 53 | 105 | 13.66 | 9.99 | — |
| 152850 | 0.887 | none | 0/0/0 | 53 | 108 | 13.75 | 10.0 | — |
| 153316 | 0.873 | none | 0/0/0 | 53 | 108 | 13.78 | 9.99 | EKF gap |
| 153717 | 0.868 | none | 0/0/0 | 53 | 105 | 13.73 | 9.99 | EKF gap |
| 154057 | 0.865 | none | 0/0/0 | 53 | 109 | 13.51 | 9.99 | — |
| 154449 | 0.872 | none | 0/0/0 | 63 | 120 | 13.76 | 9.99 | EKF gap |
| 154902 | 0.866 | none | 0/0/0 | 53 | 107 | 13.75 | 9.99 | — |
| 155252 | 0.873 | none | 0/0/0 | 53 | 106 | 13.59 | 9.99 | EKF gap |

**T1 verdict: 0 deaf of 10.** Every lens passed the ADR 0041 progress gate
first try, no `lens-attempt1.log` exists anywhere, no refusal fired, and
resolved fractions sit in the healthy band (0.843–0.887; the deaf shape is
0.000). The matched-event log recorded 10–14 (un)match events per run.

## Rate cost of UDP-only: none — odom_laser is faster

| rate | baseline (default transport, runs 131949/133922) | UDP-only (10-run range) | pinned floor |
|---|---|---|---|
| odom_laser | 10.57 / 10.61 Hz | **13.51–13.95 Hz** | 6.0 |
| EKF output | 9.99 / 9.95 Hz | 9.98–10.0 Hz | 9.0 |
| scan (gate basis) | 12.0 | 12.0 | 12.0 |
| first motion after goal | 1.37–2.72 s (18-run baseline) | 1.49 s (spot-read, 152508) | ≤30 |

Consistent with fleet OI-13's measurement ("worst-case inter-arrival gaps
BETTER under UDP-only"). `/dev/shm` carries **zero** fastrtps entries for
the whole session — the census columns are the structural proof that no
corridor participant touches shared memory any more.

## What stayed red, named so nobody reads 10/10 as a mission verdict

Every run classified `result` with a red **mission-level** criterion:
6 of 10 the pre-existing EKF-output-gap family (present on 3 of 5 baseline
runs at up to 1.063 s; ADR 0029 fusion territory, excluded from this
session's scope), and the rest the unproven-contact docking outcome
(`ARRIVED_UNPROVEN`, ADR 0033 family). **Gate-green runs: 4 of 10 vs 0 of
5 at baseline.** No new failure family appeared under UDP-only.

## Latency side-effects already visible (pre-U8)

`simctl start` 63 → 53 s on 8 of 10 runs (`--no-rviz` removes the RViz
load and its fixed 8 s sleep; the two 63 s runs are the first-of-batch
shape). Median goal-send at ~107 s vs the 129 s baseline median; median
command → first motion ≈ **109 s** (baseline ≈ 131 s) before any U8 work.

## Consequence

ADR 0040 accepts on this batch (its stated criterion was 8/8; measured
10/10). The ratified conditional grant for the three fleet simctl edits
fires. The churn repro's negative result stands recorded in
`docs/evidence/lens-deafness/NOTES.md` — the mechanism is bounded, the fix
is empirical.

## U8 confirm batch: the waits became their events (T2, corridor side)

Same command, 4 runs, 16:05–16:20 CST, after commit `892f8b7` (contract
sampled in parallel and joined before Nav2; the 8 s settle polls the lens's
`counts.map` with the old pause as ceiling; the nav lifecycle manager gated
on `nav_ready_waiter` instead of a 5 s timer).

| run | lens frac | nav attempts | simctl (s) | goal at (s) | first motion after goal (s) |
|---|---|---|---|---|---|
| 160531 | 0.960 | 1 | 53 | 85 | 1.49 |
| 160905 | 0.981 | 1 | 63 | 99 | 1.51 |
| 161254 | 0.955 | 1 | 63 | 100 | 1.50 |
| 161643 | 0.983 | 1 | 64 | 101 | 1.51 |

- **T2 (corridor-side): median command → first motion ≈ 101 s** — target
  ≤105 met — vs ≈109 s over the ten-run batch pre-U8 and ≈131 s at the
  2026-08-14 baseline. Post-`simctl` serial time fell from ~54 s to
  ~32–37 s.
- **The A/B guard held**: 4/4 single-attempt nav bring-ups, zero aborts
  (the settle's replacement risk), `nav_ready_waiter` saw all four
  `get_state` services in **0.1 s** on every run (the timer it replaced
  was 5 s), and the settle's event fired instantly ("first map seen at
  +0s" ×4 — SLAM starting ~27 s earlier means the map exists before the
  settle is reached).
- **Deafness spot-check under the new phase shape: 0 of 4** (14 of 14
  today), with the four highest lens coverages ever recorded
  (0.955–0.983) — the earlier SLAM start gives the lens more to resolve.
- The remaining fixed cost is inside `simctl start` (median 63 s: ~14 s
  discovery dwell, 8 s post-safety sleep, ~10 s poll floor around the
  genuine ~30 s Isaac readiness). Those are the fleet-side edits under
  the fired conditional grant; measured justification for the dwell
  number: `discovery-convergence.json` in `out/evidence/bringup-rework/`
  (fresh participant sees an existing publisher in ≤0.05 s, n=20, under
  Isaac load, UDP-only).

## Fleet-edit spot-check: two runs, all three simctl edits live

Fleet branch `yahboomcar-ros2 simctl-events-2026-08-14` (commits
`cfa1220` dwell/`--trust-clean`, `39b47d6` wait_for check-first,
`47a8f95` safety event), corridor `--trust-clean` wired. robot2 2d smoke
first: **green** — "[3/3] both topics advertised **(1 s)**" (the old
wait carried a ≥5 s floor), clean teardown
(`out/evidence/bringup-rework/robot2-smoke-2026-08-14.log`).

| run | lens frac | nav attempts | simctl (s) | goal at (s) | motion after goal (s) |
|---|---|---|---|---|---|
| 162722 | 0.971 | 1 | **40** | **74** | 1.44 |
| 163036 | 0.968 | 1 | **39** | **75** | 1.51 |

- `simctl start` 63 → **39–40 s**: dwell 14→4 s (`--trust-clean`,
  silence budget only), post-safety pause → governor+deadman
  discoverable at **4.0 s** measured (the event halved the old fixed 8),
  check-first poll floors gone.
- **Command → first motion ≈ 76 s** — beats the ≤85 s
  with-fleet-delegation target; corridor-side-only was already ≤105 at
  ~101 s. Baseline: ≈131 s.
- Lens healthy on both → **0 deaf of 16 runs today**.
- **Two runs are a spot-check, not a batch** (gate discipline): quoting
  ≤85 s as a standing number should wait on a full 8-run batch under the
  merged fleet branch — parked as a morning decision. The corridor's
  descriptive `phase_typical_s`/header numbers deliberately still say
  ~63 s / ~100 s (the corridor-only shape) until the fleet branch passes
  review and a full batch re-measures.
