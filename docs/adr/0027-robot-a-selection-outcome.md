# ADR 0027: Robot A selection outcome — the corridor gate failed; A stays robot1

- Status: Accepted
- Date: 2026-08-11
- Source: three-profile live measurement, recorded in
  [`docs/evidence/robot-a-gate/NOTES.md`](../evidence/robot-a-gate/NOTES.md)
  with per-profile JSON artifacts beside it.
- Executes the procedure decided in
  [ADR 0022](0022-robot-a-selection-gate.md), including its fallback clause.
  0022 is not edited by this record; it decided *how* to choose, and this is
  what the choosing produced.

## Decision

**The RaspTank fleet twin (robot2) did not pass the corridor gate. Robot A
remains robot1, per ADR 0022's fallback clause.**

Both gated profiles failed, on the same cause, with the same signature:

| Profile | Gated | Moved | First `odom_laser` | Withheld | Midpoint drift | Nav2 | Verdict |
|---|---|---|---|---|---|---|---|
| `nominal_m6_n3` | yes | 8.17 m | 5.83 m | 506 (limit 5) | 1.000 (limit 0.05) | ABORTED, 18.37 m | **FAIL** |
| `wide_corner_m6_n4_5` | yes | 8.30 m | 5.41 m | 459 | 1.000 | ABORTED, 17.59 m | **FAIL** |
| `uniform_m6_n6` | reported | 8.38 m | 4.77 m | 391 | 1.000 | not measured | finding |

## What actually failed

Not the chassis, the governor, or SLAM. The twin drove ~8.3 m on every profile
and built a map of 1361–1682 occupied cells. **Odometry is what failed.**

`odom_laser` publishes nothing until the robot has already travelled 4.8–5.8 m.
Until then the EKF has no laser input, its pose stays at the origin, and
estimated travel at the midpoint is 0.0 m — which is why the drift fraction is
exactly 1.000 on all three profiles rather than merely large. Nav2 then aborts
against a pose that never moved.

## The degeneracy study

The primary artifact is the covariance-against-station trace in each
`gate-*.json` (466–599 rows per profile). At the midpoint:

| Profile | `cov_xx` (along corridor) | `cov_yy` (across) | Anisotropy |
|---|---|---|---|
| `nominal_m6_n3` | 4.58e-04 | 2.61e-05 | 17.6× |
| `wide_corner_m6_n4_5` | 4.73e-04 | 2.52e-05 | 18.8× |
| `uniform_m6_n6` | 4.63e-04 | 2.51e-05 | 18.4× |

The along-corridor axis is ~18× less constrained than the cross-corridor axis,
stable across a 6→3 m taper, a 6→4.5 m taper, and an untapered 6 m corridor.
This is the textbook corridor degeneracy, measured rather than asserted.

Acquisition station and withheld count both order monotonically with taper
(5.83/506, 5.41/459, 4.77/391). **Three points do not establish a mechanism and
none is claimed here.**

## Honest scope

These bound how far the result may be carried:

- **This is not a verdict on the chassis.** The scan matcher is the fleet's
  in-house one, tuned and measured in a 4×4 m room (fleet OI-04 keeps its
  performance on C1 data open). Whether the corridor behaviour is a platform
  limit or a tuning limit is *not resolved by this measurement*. The gate asked
  whether robot2 delivers in the corridor today; the answer is no, and the
  reason is a component that could plausibly be retuned.
- **C1's 12 m range partially re-constrains the travel axis** through end-wall
  returns, so the corridor is not a pure degeneracy case. The measured
  anisotropy is what survives that help, not a worst case.
- **Pivot-yaw slip of 32.2 % [measured, fleet G5] is uncompensated** and would
  bite hardest at the corner arc, which none of these runs reached — the robot
  never got past ~8.4 m of a ~24.6 m route.
- **The lidar forward offset `x=0.08` is still [estimate]** (hand tape confirmed
  only the 0.10 m scan height, 2026-08-11), so the sensor's along-axis position
  carries an unmeasured error on precisely the axis that is degenerate.
- **`robot_radius` is pinned at 0.12 m against a measured circumscribed
  0.128 m** — nav2 believes the robot is 8 mm smaller than it is, and that
  matters most where the corridor is narrowest. Deliberately not changed:
  moving an inflation parameter mid-gate would invalidate the runs measuring it.
- **Nav2's abort is not fully attributed.** The goal sat ~18 m away in largely
  unmapped space, so "aborted because localization failed" and "aborted because
  the goal was unreachable in the map built so far" are not separated here.
- **Twin sensor rates are variable.** The `--imu-hz 60` precondition failed on
  two of five sessions (41.01 Hz, and 37.46 Hz with `scan` at 7.06 Hz) and
  passed on the others. Both failures were rerun as infrastructure, never
  recorded as results, and nothing was tuned to make a run pass.

## A measurement defect worth recording

The withholding metric originally counted only gaps *between* `odom_laser`
messages, so the interval from drive start to the first message was invisible. A
run in which the matcher produced nothing whatsoever for 5.9 m scored **"1
consecutive withheld update"** — comfortably inside the limit of 5. The same run
scores 506 once the initial silence is counted.

**A gate measuring only the gaps it could already see would have passed this
stack as healthy.** The defect is recorded because the corrected metric is the
one this decision rests on, and because the failure mode — an instrument blind
to the absence of a signal rather than to its degradation — is general.

## Consequences

- Robot A remains **robot1** for the corridor demonstration. The v2 autonomy
  narrative is delivered on robot1, not the RaspTank twin.
- The corridor gate tooling, the arenas, and the fleet membership all stand and
  are reusable: the twin is a working simulation that fails a specific
  localization gate, not a dead end.
- **Reopening this is cheap and legitimate.** A matcher retuned for corridor
  geometry, or a measured lidar `x` offset, would justify re-running the same
  three profiles against the same thresholds. This record is an outcome, not a
  closed door; a re-run that passes supersedes it with a new ADR.
- ADR 0022's procedure is now discharged.
