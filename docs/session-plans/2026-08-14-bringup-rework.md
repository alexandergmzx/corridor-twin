# Session plan — bring-up rework: reliable, then fast (2026-08-14)

Branch: `bringup-rework-2026-08-14` off `8928de8` (tip of
`bringup-lens-2026-08-14`). Approved plan:
`~/.claude/plans/plan-mode-bring-up-linear-cookie.md` (research, hypothesis
table, ratified decisions). Operator present at start; batch stretches run
autonomously, so this plan binds per CLAUDE.md's long-session rules.

**Wall-clock budget: 14:15 → 22:00 CST. No new units after 21:30.**

Targets:
- **T1**: lens deafness 0 over ≥8 runs with the chosen fix
  (baseline 2/4–2/6 across three sessions).
- **T2**: command → first motion, median ≤105 s corridor-side
  (baseline ≈131 s); ≤85 s only if the ratified conditional fleet grant
  fires (T1 = 8/8 first).

Scope guard: run 4's abort (yaw 1.1353, drift 0.355) is ADR 0029 fusion
territory — untouched here.

Ratified (2026-08-14, operator): batches run this session; `--no-rviz`
becomes the profile-run default; fleet simctl edits only after T1 = 8/8
under the standing conditional grant (one edit per commit in robot-fleet,
event-derived not shortened, T2 phase table in each message, robot2 smoke),
else OI notes to the fleet backlog.

## Inventory (file:line)

- Lens: `tools/lens/corridor_lens.py` (subs :290-299, healthz :194-229,
  sampler :551-578, node :241-301); `tools/lens/_lens_core.py`
  (RateWindow :227-265, is_frozen :208-224, FREEZE_IDLE_S :205).
- Runner: `tools/corridor_profile_run.sh` (phase() :240-253, lens block
  :822-890, lens_is_seeing :784-797, settle :1093, teardown :541-597,
  simctl invocation :765-768, budget header :315-333,
  phase_typical_s :228-238).
- Manifest: `tools/run_manifest.py` (SCHEMA_VERSION :54).
- Nav launch: `config/robot1/robot1_nav_corridor_launch.py`
  (TimerAction :148-151).
- Profile: `robot-fleet/ground_station/fastdds_udp_only.xml` (OI-13).
- Tests to extend: `test/test_corridor_lens.py`,
  `test/test_the_lens_is_up_before_the_run.py`,
  `test/test_the_wait_is_acknowledged.py`.
- Jazzy rclpy verified: `SubscriptionEventCallbacks(matched=…)` and
  `create_timer(clock=…)` exist (event_handler.py:163, node.py:1750-1756).

## Unit queue (timeboxes; status updated after each unit)

| # | Unit | Timebox | Status |
|---|---|---|---|
| U1 | This plan on disk | 10 m | DONE |
| U2 | Lens instrumentation: matched-event logging, executor-liveness timer, healthz counts/matched/exec fields (observation-only; NO new publisher — the zero-publisher invariant of ADR 0035 §2 stands, so H3's discriminator is a node timer, not a self-ping) + tests | 45 m | DONE — commit `c838e9c`; live-smoked on domain 69 (exec_tick_age_s 0.236) |
| U3 | run.json covariates: per-phase durations from phases.log, /dev/shm fastrtps census ×3, schema 1.1.0 + tests | 45 m | DONE (this commit) |
| U4 | DDS churn repro, no Isaac, domain 69, SHM vs UDP-only A/B; two realism escalations max, then batch decides | 60 m | DONE — **negative result, filed in bold**: three arms (plain os._exit churn ×448; + victim kill-and-replace ×19 generations; + SIGKILL-mid-traffic, peak 68 segments) all CLEAN. H1's synthetic form bounded, not confirmed; ADR 0040 context corrected; batch is the decider. Commits `7c80e10` + evidence commit. Tool itself landed early inside `02451c9` (over-broad add, recorded in 7c80e10's message) |
| U5 | Rung (a): UDP-only export (both env names, D5 resolution) + `--no-rviz` default + ADR draft + tests | 30 m | DONE — `68bce8a` |
| U6 | T1 batch: 4 runs → extend to 10 (operator raised the floor to ≥10 before leaving); rate-cost table vs robot1 contract numbers; evidence commit | 150 m | DONE — **T1 MET: 10/10 lens-healthy** (frac 0.843–0.887, 0 restarts, 0 refusals, shm 0/0/0, odom_laser FASTER under UDP-only: 13.5–13.9 vs 10.6 Hz baseline, EKF unchanged). ADR 0040 → Accepted. Gate-green 4/10 vs 0/5 baseline; remaining reds are pre-existing ADR-0029/0033 families. Median command→motion already ≈109 s pre-U8. **Conditional fleet grant FIRES.** Evidence: docs/evidence/bringup-rework/NOTES.md |
| U7 | Progress-based seeing gate (count-delta across two polls) + second-checkpoint same + tests | 30 m | DONE — `02451c9` + ADR 0041 |
| U8 | P2 corridor-side: lens ∥ simctl, contract ∥ SLAM, settle→event, TimerAction→get_state-event; update phase_typical_s/header/tests; measured against the U6 batch | 90 m | pending |
| U9 | Fleet edits ONLY if the conditional grant fires (T1 = 8/8): dwell skip, wait_for check-before-sleep, safety sleep→event; one commit each in robot-fleet + robot2 smoke | 60 m | pending (conditional) |
| U10 | Handoff doc + morning decisions | 30 m | pending |

Skip-edges: U4 SHM non-repro after 2 escalations → U5 anyway (batch
decides). U6 any deafness under UDP-only → matched logs discriminate → gate
hardening rung (c) grows a participant self-restart before more batches.
T1 short of 8/8 → U9 skipped, OI notes instead.

Delegated / not delegated: fleet-repo writes ONLY under the ratified
conditional grant (U9), committed separately in robot-fleet. Nothing else
outside this repo. No pushes (local-only). Isaac under
`/tmp/fleet-isaac.lock` discipline; domains: repro on 69, runs on 67;
deny-list 20/42/43/44/66/68 untouched.

## Handoff (final section, written at session end)

*(empty until the session ends — this section records only measured
outcomes, never promised ones)*
