# ADR 0022: Select robot A by a measured corridor-odometry gate

- Status: Accepted
- Date: 2026-08-11
- Source: interview feedback of 2026-08-04 (autonomy correction); fleet
  session-6 evidence (2026-08-08); assumptions verified against fleet code in
  [`docs/v2-plan.md`](../v2-plan.md) §1 and §6.
- Extends [ADR 0004](0004-corridor-profile-variants.md): the `(m,n)` variant
  set becomes the selection instrument.
- The outcome will be recorded as ADR 0027; this record decides the procedure
  and is not edited by the result.

## Context

The task author's second correction: A must deliver **autonomously** from A to
B; v1's authored line and waypoints were read, correctly, as a level
indicator. v2 therefore needs a real robot model with a real autonomy stack,
and two candidate twins exist in the fleet with materially different odometry
physics:

| Candidate | Odometry | Verified state |
|---|---|---|
| robot2 — RaspTank twin (`rasptank-ros2`) | Laser scan matching → EKF; **no encoders by fleet decision D-05**, enforced by an executable contract check | Governed Nav2 goal SUCCEEDED in sim (session 6). Two conflicting recorded error figures exist — **82 mm** (rasptank README) and **131 mm** (fleet research plan) — different runs of the same gate, neither with a machine-readable artifact; both under the 150 mm tolerance |
| robot1 — Yahboom twin (`yahboomcar-ros2`) | Wheel encoders 11 Hz + IMU → EKF (`pn-fix`, sim-validated only) | Drive/map validated; Nav2 goal-reaching sim-only with its accuracy figure formally retracted; hardware-measured items are the deadman, watchdog absence, and stand A/B — not navigation |

The corridor is a canonical degeneracy environment for scan-matching
odometry: parallel walls constrain the lateral axis and say little about
progress along the travel axis. The scenario's own variant set spans the
question — `nominal_m6_n3` narrows continuously (wall distance encodes
longitudinal position), `uniform_m6_n6` is the degenerate stress case. One
honest caveat is recorded up front: the C1 lidar's 12 m range [vendor claim]
covers much of the 12 m corridor, so end-wall and corner returns partially
re-constrain the travel axis; the study reports covariance against station,
not a single verdict, and its degeneracy claim is strongest mid-corridor.

robot2 carries the stronger narrative (the robot the task author saw, autonomy
with zero odometric sensors onboard); robot1 carries geometry-independent
odometry. Choosing a priori would replace a measurable question with an
opinion.

## Decision

1. **Selection is by gate, not preference.** Run robot2 through the corridor
   scene on all three profiles: a corridor drive-and-map gate followed by one
   governed Nav2 goal A→B per profile.
2. **The gate is a fork, not a flag.** The fleet's `robot2_sim_gate.py`
   exposes only `--seconds`, knows nothing of arenas, and drives an open-room
   polygon that would fight corridor walls through the governor. Following
   the fleet's copy precedent, the corridor gate is a fork with a
   straight-pass drive schedule and the same JSON-report shape. Its
   measurement scripts export their own `ROS_DOMAIN_ID`; nothing exports it
   for them.
3. **Acceptance, evaluated on `nominal_m6_n3` and `wide_corner_m6_n4_5` only**
   (`uniform_m6_n6` is a stress report, never a gate). Thresholds pinned from
   the derivations in `docs/v2-plan.md` §6:
   - Nav2 action SUCCEEDED **and** map-frame goal error ≤ **0.15 m** — the
     fleet's one pinned tolerance (`xy_goal_tolerance: 0.15` in
     `nav2_robot2.yaml`). The measuring script must *enforce* this bound; the
     fleet's `test_nav_governed.py` currently enforces 0.30 m while printing
     "tolerance was 150 mm", and the corridor variant corrects that.
   - The scan matcher never withholds more than **5 consecutive** updates —
     ≈0.5 s at the measured 10.4–10.8 Hz matcher rate, ≤0.11 m blind travel
     at the 0.22 m/s cap, under the goal tolerance. Measured as the maximum
     publication-stamp gap on `/robot2/odom_laser`.
   - Longitudinal drift versus simulator truth ≤ **5 % of distance
     travelled** at the corridor midpoint — ≈0.30 m at the ~6 m midpoint,
     consistent with the only enforced end-to-end bound in the fleet today.
     Truth is consumed by the evaluation plane only, per ADR 0020/0021.
4. **Pass → robot A = robot2. Fail → robot A = robot1**, with the one-sentence
   bridge in the delivery note (the odometry needs of the autonomy
   correction). Either way the three-profile run is published as the
   degeneracy study — measured exploratory work.
5. **All v1 certificate numbers are retired for v2** regardless of outcome:
   the 0.0369 m/s worst error, the station-10.0 m episode, the 24.601 m /
   24.62 s route figures, ray counts, and the VRAM figure. v2 quotes only v2
   runs, produced by whichever robot this gate selects.

## Consequences

- The corridor scene must load as a fleet arena before the gate can run
  (ADR 0025 is the prerequisite, and the arena-composition preconditions
  verified in `docs/v2-plan.md` §6 apply: the C1 lidar authored by the shared
  `author_lidar` function with `minDistBetweenEchosM = 0.05`, the
  scan conditioner in the loop, the contract check invoked with
  `--imu-hz 60` against the Isaac twin, absolute `/robot2/map`, and
  `bond_timeout: 0.0`).
- Route length, duration, and violation numbers become emergent properties of
  an autonomy stack, not scripted constants; the v2 requalification table
  replaces the v1 invariants wholesale.
- A failed gate is a publishable negative result, in the fleet's
  negative-results-in-bold tradition, not a discarded run.
- The gate runs contend for the machine-wide single Isaac slot, which is an
  honor-system protocol, not a code lock; sessions acquire and release it
  manually.

## Alternatives considered

- **Pick robot1 a priori.** Rejected: forfeits the zero-odometric-sensors
  story and the degeneracy study, and robot1's navigation evidence is weaker
  than its reputation — its Nav2 accuracy figure is retracted, and its
  corrected EKF is stand-validated only.
- **Pick robot2 a priori.** Rejected: unmeasured risk exactly where the
  scenario is hardest for its odometry class.
- **Build both to completion.** Rejected for the Aug 24 window: doubles the
  Phase 2–3 requalification for no additional interview claim. Two robots,
  one corridor remains future work.
- **Reuse the fleet gate unmodified.** Rejected: it cannot select an arena,
  cannot pin a domain, and its drive pattern is wrong for a corridor; a fork
  with corridor-shaped drive and thresholds is the fleet's own precedent for
  divergence.
