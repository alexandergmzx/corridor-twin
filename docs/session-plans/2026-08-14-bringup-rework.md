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
| U8 | P2 corridor-side: contract ∥ SLAM, settle→event, TimerAction→get_state-event; phase_typical_s/header/tests updated; lens ∥ simctl PARKED (decision filter: target met without the spine restructure) | 90 m | DONE — `892f8b7` (code) + `a757216` (measured: confirm batch 4/4, median command→motion ~101 s, zero nav aborts, waiter 0.1 s, settle instant, frac 0.955–0.983) |
| U9 | Fleet edits under the FIRED grant: dwell `--trust-clean` (4 s silence budget, convergence-justified), wait_for check-first, safety sleep→event; one commit each in robot-fleet + robot2 smoke + corridor spot-check | 60 m | DONE — fleet `simctl-events-2026-08-14`: `cfa1220`/`39b47d6`/`47a8f95`; robot2 2d smoke GREEN ("(1 s)" wait); corridor `--trust-clean` wired; **spot-check: simctl 39–40 s, command→motion ≈76 s, 2/2 lens-healthy (16/16 today)** |
| U10 | Handoff doc + morning decisions | 30 m | DONE — this section |

Skip-edges: U4 SHM non-repro after 2 escalations → U5 anyway (batch
decides). U6 any deafness under UDP-only → matched logs discriminate → gate
hardening rung (c) grows a participant self-restart before more batches.
T1 short of 8/8 → U9 skipped, OI notes instead.

Delegated / not delegated: fleet-repo writes ONLY under the ratified
conditional grant (U9), committed separately in robot-fleet. Nothing else
outside this repo. No pushes (local-only). Isaac under
`/tmp/fleet-isaac.lock` discipline; domains: repro on 69, runs on 67;
deny-list 20/42/43/44/66/68 untouched.

## Handoff (final section — measured outcomes only)

Written 16:40 CST, inside budget (14:15→22:00; last unit closed 16:40).

**Both targets met, both beaten.**

- **T1 (deafness 0 over ≥10): MET at 0 of 16.** Ten-run batch under
  UDP-only (ADR 0040, Accepted) + four U8-confirm + two fleet-edit
  spot-checks — no restart-once, no refusal, `lens_resolved_frac`
  0.843–0.983, `/dev/shm` at zero fastrtps entries throughout.
- **T2 (command → first motion): baseline ≈131 s → ~101 s corridor-side
  (target ≤105, 4-run confirm) → ≈76 s with the fleet edits (target ≤85,
  2-run spot-check — a spot-check, NOT a quotable batch).**

**Corridor branch** `bringup-rework-2026-08-14` (13 commits, additive, all
tests green — 502 passed): instrumentation (`c838e9c`), covariates
(`3827a8a`), UDP-only + `--no-rviz` (`68bce8a`), progress gate + ADR 0041
(`02451c9`), repro escalations (`7c80e10`), repro negative-result evidence,
T1 evidence + ADR 0040 acceptance (`d7f56af`), U8 events (`892f8b7`), U8
measurement (`a757216`), `--trust-clean` wiring, CLAUDE.md banner-claim
update. **Fleet branch** `yahboomcar-ros2 simctl-events-2026-08-14`
(3 commits under the ratified grant, one edit each, T2 tables in messages).
Nothing pushed anywhere (git local-only).

**Known state / caveats**

- Mission-level reds are UNTOUCHED and still there: EKF-gap (6/10 of the
  T1 batch; up to 1.438 s vs 0.4 bound; also 3/5 at baseline) and
  unproven-contact `ARRIVED_UNPROVEN` — ADR 0029/0033 territory, excluded
  by this session's scope guard. 10/10 is a LENS verdict, not a mission
  verdict (gate-green 4/10 vs 0/5 baseline, so no regression).
- The churn repro did NOT reproduce the deafness (three clean arms) —
  the mechanism is bounded, not named; the fix is empirical
  (`docs/evidence/lens-deafness/NOTES.md`).
- `yahboomcar-ros2` had a pre-existing dirty file (`slam_debug.rviz`),
  found and left untouched on its previous branch's working tree.
- The U7 gate commit (`02451c9`) accidentally swept in the repro tool
  (over-broad `git add`); recorded in `7c80e10`'s message. History
  additive throughout.
- Descriptive bring-up numbers (`phase_typical_s`, "~100 s" header) state
  the corridor-only shape; they move again only after the fleet branch
  merges and a full batch re-measures.

**Morning decisions**

1. **Fleet branch review/merge** (`simctl-events-2026-08-14`, 3 commits).
   If merged: run the full 8-run batch and only then quote ≈76 s; also
   update `phase_typical_s`/header to the re-measured shape.
2. **Lens ∥ `simctl start` overlap** (last ~5–7 s): parked — restructures
   the runner's phase spine and touches ADR 0037's placement decision;
   targets were met without it.
3. **Demo-path UDP-only** (`run_demo.sh`, domains 42/43): requires
   re-measuring ADR 0026's crossing ratios before quoting them; not done
   this session by design (ADR 0040 scope note).
4. **Fleet `os._exit(0)` probe children**: correct long-term fix is a
   clean shutdown path; harmless to corridor under UDP-only. Fleet
   backlog / OI note — writing the fleet ledger row is beyond the grant.
5. **OI-13's REMAINING item** ("bake the export into the standard
   ground-station env"): corridor sessions are now baked; the
   machine-wide bake is a fleet decision.
6. **ADR-0029-family reds** remain the mission blockers for the next
   session (EKF gap + docking contact proof).
