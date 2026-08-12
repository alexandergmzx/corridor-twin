# Close the autonomy gate, open Phase 3

> **APPROVED 2026-08-12 17:18 CST. UNATTENDED** — Alexander is away, so the hard
> rules bind in full: git is local-only and `git push` does not exist tonight;
> history is append-only; a commit is a green checkpoint or it does not happen;
> Isaac is single-occupancy under `/tmp/fleet-isaac.lock`; judgment calls are
> **parked, not decided**.
>
> **Budget 5 h — started 17:18, ends 22:18. No new unit starts after 21:48.**
> `date` between units. Branch `gate-green-2026-08-12` from
> `corridor-stabilization-2026-08-12` (`9867360`). The handback is this
> document's final section and is written even if the session fails early.

## Live status

| Unit | State | Notes |
|---|---|---|
| P0 session plan | **DONE** 17:20 | this file |
| P1 the startup circle, fixed | pending | 90 min hard timebox, then the skip-edge |
| P2 landmark containment | pending | radius re-derived at scale, not copied |
| P3 scan path, wide_corner + uniform | pending | nominal already green |
| P4 acceptance runs | pending | nominal + wide_corner gated, uniform reported |
| P5 P-camera candidates | pending | runs even if P4 is red |
| P6 paper debt | pending | no Isaac, parallel-safe |

## Session start state

Host clean at 17:18: GPU 607 MiB of 16303, RAM 19 G free, disk 120 G free,
domain 67 carries no un-namespaced ROS nodes. A **stale** Isaac lock from my own
16:47 run was removed — PID 2001914 dead, owner `corridor-profile-run
nominal_m6_n3`, the run the watchdog killed.

## What the stabilization session hands over

Green: ruff clean, 387 passed / 1 skipped, colcon 140 tests 0 failures.

Working that was not before: the arena and the plan are the same scenario
(hash-checked before every run), the gate divides by the observed window, the
lens runs, evidence is session-scoped with a result/rerun/**crash**
classification, the domain is preflighted and teardown escalates, the scan
filter accepts the corridor from scan 1, and the wheel radius is calibrated —
midpoint drift **0.159 → 0.0097**.

Two things it deliberately left open, and both are P1/P2's inputs:

1. **The startup circle is measured, named and NOT fixed.** All three prior
   hypotheses are falsified by log: nothing commands A before the goal
   (`commands_before_goal: 0`, three runs of three), and the first recovery
   `Spin` fires at t = 173 s when A is already parked at the far end. **The
   recovery-Spin brief is dead and is not to be implemented.** What ground truth
   shows is 253° of turning over 1.06 m in the first 60 s, ending 0.2 m *behind*
   spawn, then **56 s stationary**, then a transit that works and reaches
   0.0625 m from the delivery standoff.
2. **A drives 3.4 m past B and stops**, because with docking off the arrival
   gate is a map-frame goal the drifting map never satisfies (ADR 0029).

## P1 — the startup circle, fixed. **90 min hard timebox.**

The culprit is the local controller, commanding rotation *after* the goal while
the map is still nearly empty. The leading mechanism, and the first thing to
measure, is that **A is chasing a map frame that is still moving**: SLAM's
`map→odom` correction jumps as its first scans arrive, A's *believed* heading
swings without the robot moving, and the controller physically rotates to
correct an error that only exists in the frame.

**Diagnosis first, offline, no GPU.** Both of this afternoon's bags carry `/tf`,
`/odom` and `/sim/ground_truth`. Compare map-frame yaw against truth yaw over
the first 60 s and plot the `map→odom` correction beside the commanded rotation.
If the correction moves and truth follows it, the mechanism is confirmed and the
fix is a readiness gate, not a controller parameter.

**Fix, if confirmed:** hold the goal until the map frame has settled — a bounded
wait on `map→odom` being stable within a threshold over a window — rather than
sending it the instant `bt_navigator` reports ACTIVE. Runner-side, no Nav2
parameter touched.

**Acceptance** (ground truth, three consecutive runs): cumulative pre-transit
rotation **< 45°**, forward progress begins **< 30 s after goal-send**, and
transit accuracy does not regress — closest approach **≤ 0.15 m** of the
delivery standoff.

**Skip-edge at 90 min:** ship the measured mitigation that gets tonight's runs
green, and write the proper fix up as a parked unit with its evidence. Do not
spend P4's budget here.

## P2 — landmark containment, re-derived at the committed scale

Per Alexander's 14:52 decision, U3's numbers are re-derived rather than carried:
the arming window is **derived from the scaled manifest**, not the briefed 3.0 m.
The as-run route is **7.380 m** at nominal (pinned in
`test_scenario_as_run.py`), against 24.601 m authored — so a 3.0 m window
authored at the old scale is 0.9 m here. Compute it from the manifest at run
time; do not write a literal.

Arming requires **all** of: travel-integrated distance (EKF, A's own estimate,
never truth) ≥ route − window; map-frame goal proximity ≤ window; detection
bearing within **±60°** of the goal bearing. Spawn-region negative control in
the tests, paired with a positive guard so it cannot pass vacuously. **Docking
stays OFF in every gate run.**

## P3 — scan path for wide_corner and uniform

Nominal is green: 349 passed / 10 dropped, no fail-open. The two constants are
already manifest-driven and per-profile, so this is a verification unit: one
smoke each, acceptance **accepts from scan 1, zero fail-open**.

## P4 — the acceptance runs

nominal + wide_corner **gated**; uniform runs and is **REPORTED, never gated**
(ADR 0022). Masked-oracle map ≤ 0.20 m, gate JSON pass at pinned thresholds,
P1's startup criterion, zero landmark events, dock off, lens up, session-scoped
artifacts with arena and manifest hashes.

Reds are committed findings, in bold, with their artifacts. **No tuning past
this list.** If green, one additional nominal pass with `--dock` enabled as a
separate, clearly-labelled demo-candidate run.

## P5 — Phase 3 opens: P-camera candidates

**Runs even if P4 is red.** 2–3 poses computed from the SCALED manifest, each
with per-gate line-of-sight, distance and incidence angle to all four
enforcement stations; one 640×360 frame each through the ADR 0009 adapter in a
single Isaac session; saved under `docs/evidence/p_cam_candidates/` with a
one-page memo: the table, and what each pose costs the detector.

**CHOOSE NOTHING.** The memo is for Alexander.

## P6 — paper debt (no Isaac, parallel-safe, separate commits)

- **ADR 0030** pinning the committed scale constant and the robot-scale build
  default, its derivation, the oracle-mask decision, and the 0.12 m screen
  margin — declaring every prior scale value superseded.
- **ADR 0029**: rename to the house register; scope the "odometry calibration
  acquitted" row to its bench-sweep conditions.
- Verify `do_loop_closing` in the committed slam config matches the
  falsified-hypothesis revert.

## Rules for tonight

Two attempts at any failing command for the same reason, then record and move
on. Every gate run writes JSON; thresholds printed and enforced from one
constant; infrastructure failures are reruns, twice at most, classified
explicitly. Fleet touches only if a P1 fix genuinely requires one — single
commit, listed separately. **Nothing pushed.**

## Handback

*(written at session end — the Biswal scoreboard)*
