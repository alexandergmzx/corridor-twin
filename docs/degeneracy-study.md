# Corridor scan-match degeneracy — study

Companion to [ADR 0027](adr/0027-robot-a-selection-outcome.md). That record
decided robot A; this one keeps the measurement that produced the decision and
says carefully what it does and does not support.

> **Draft, 2026-08-11.** Robot2's three profiles are measured and committed.
> Robot1's contrast is the open half; this document is written so it can be
> extended rather than rewritten when those runs land.

## The phenomenon

A corridor is the textbook scan-matching degeneracy: two long parallel walls
constrain the cross-corridor direction tightly and the along-corridor direction
barely at all, because sliding the robot along the corridor produces a scan that
looks almost identical.

On robot2 this was not a subtle accuracy loss. On every profile the matcher
published **nothing at all** until the robot was ~5 m in, and robot2 has no
wheel encoders (fleet D-05), so the EKF had no other odometry source. Its pose
stayed at the origin, and Nav2 aborted against a robot it believed had not
moved.

| Profile | Acquisition station | Max consecutive withheld | Midpoint drift | Nav2 |
|---|---|---|---|---|
| `nominal_m6_n3` | 5.83 m | 506 | 1.000 | ABORTED |
| `wide_corner_m6_n4_5` | 5.41 m | 459 | 1.000 | ABORTED |
| `uniform_m6_n6` | 4.77 m | 391 | 1.000 | not measured |

Artifacts: `docs/evidence/robot-a-gate/gate-*.json`, each carrying the full
covariance-against-station trace (466 / 476 / 599 rows).

## The signature: ~18x anisotropy, and it does not move

At the midpoint of each run:

| Profile | `cov_xx` (along) | `cov_yy` (across) | Anisotropy |
|---|---|---|---|
| `nominal_m6_n3` | 4.58e-04 | 2.61e-05 | 17.6x |
| `wide_corner_m6_n4_5` | 4.73e-04 | 2.52e-05 | 18.8x |
| `uniform_m6_n6` | 4.63e-04 | 2.51e-05 | 18.4x |

**The most informative result here is a non-result.** Anisotropy is the one
quantity that does *not* order with anything: 17.6, 18.8, 18.4 across a 6→3 m
taper, a 6→4.5 m taper, and an untapered 6 m corridor. The degeneracy's
*magnitude* looks like a property of "being in a corridor at all" rather than of
how sharply that corridor narrows.

## Ordering analysis, and what is circular in it

`tools/degeneracy_analysis.py` computes candidate predictors of the acquisition
station (artifact: `degeneracy-analysis.json`). Results:

| Predictor | Ordering vs acquisition station | Usable? |
|---|---|---|
| taper (m width per m length) | decreases | **yes** — a profile property, fixed before the run |
| local clear width at acquisition | decreases | weakly — partly determined by where acquisition happened |
| end-wall distance at acquisition | decreases | **no — circular** |
| max consecutive withheld | increases | **no — restates acquisition** |
| midpoint anisotropy | no monotonic ordering | yes, and it is the interesting one |

Two rows are arithmetic, not evidence, and are kept only so nobody re-derives
them as findings later. End-wall distance at acquisition is
`11.50 − station`, so it *must* order inversely with station. Withheld count is
the same quantity as acquisition expressed in scan periods.

That leaves taper as the only non-circular predictor that orders — over
**three points**, with taper covarying with local width by construction. No
mechanism, fit, or correlation coefficient is claimed. ADR 0027 explicitly
refuses that claim and this study does not quietly acquire it.

## The robot1 contrast (open)

Robot1 is expected to behave differently for an architectural reason rather than
a geometric one: its EKF does not consume the scan matcher at all.
`ekf_sim_pnfix.yaml:138-146` removed the laser-pose input as "measured
HARMFUL", leaving wheel encoders (`odom0: /odom_raw`, vx only, `:117-122`) and
IMU yaw-rate (`imu0: /imu/data`, `:150-155`). Meanwhile `simctl:665` still
launches `yahboomcar_localization laser_odometry`, so `/odom_laser` is published
and measurable **beside** a healthy EKF.

If that holds, the corridor degeneracy is still present on robot1 and simply no
longer load-bearing: the matcher may withhold exactly as it did on robot2 while
`/odom` never stops and Nav2 never loses its pose. That is the contrast worth
recording — same corridor, same instrument, different dependency graph.

### A confound that must not be glossed

Robot1 and robot2 do not carry the same lidar:

| | Robot | Max range | Can range the end wall from spawn? |
|---|---|---|---|
| C1 | robot2 | 12.0 m | yes, from station 0 |
| MS200 | robot1 | **8.0 m** | **no — not until station 3.50 m** |

The corridor's end wall stands 11.50 m from A's spawn on every profile
(`build_arena.py:54`; `build_rasptank_arena.py:46`). So any robot1-vs-robot2
comparison of *acquisition station* compares two sensors as well as two odometry
architectures, and cannot be attributed to either alone. The comparison that
survives the confound is the **dependency** one: whether localization continues
when the matcher does not.

## The scale finding: the corridor was 37x the robot

**Added 2026-08-11, and it reframes everything above.**

The authored corridor is 12 m long and 6 m wide. The robot is 0.20 x 0.16 m. The
drawing carries no scale bar, so those metres were always demo choices -- and
they were chosen before any robot existed. A corridor 37 times the robot's width
does not merely exaggerate the degeneracy; it *is* the degeneracy. The walls sit
3 m away, the end wall 11.5 m away, and robot1's MS200 (8.0 m) cannot range that
wall at all until station 3.5 m.

Scaling every length by 0.2 -- preserving every ratio, and therefore every
geometric argument in the scenario -- gives a 2.4 m corridor, 1.2 m at the entry
and 0.6 m at the corner, which is 3.8x the robot's width. The end wall then
stands 2.3 m from A's spawn, inside lidar range from station 0.

The effect on the same robot, same stack, same instrument:

| | 6 m corridor | robot-scale corridor |
|---|---|---|
| First `odom_laser` at station | **9.66 m** | **0.03 m** |
| `odom_laser` messages / rate | 12 / 0.13 Hz | **1052 / 11.69 Hz** |
| Max consecutive withheld | 936 | 8-35 |
| Worst EKF gap | 1.46 s | 0.36-0.54 s |
| Nav2 map-frame goal error | 18.46 m | **3.17 m** |

The matcher acquires essentially immediately and then runs at its full rate.
**The corridor degeneracy measured across this study is, to a large extent, an
artifact of a scene built at the wrong scale for its robot** -- not a property
of corridors, and not a property of either chassis.

That does not retract the robot2 measurements: they are true of the scene as
authored, and ADR 0027's decision was taken against that scene. It does mean the
~18x anisotropy is a measurement of a 6 m corridor, and a re-run of robot2 at
robot scale is the obvious next experiment. It has not been done.

What did NOT come right with scale: Nav2 still aborts. The planner reaches
"Failed to create plan" through the 0.6 m corner rather than the earlier
"outside bounds", so the remaining blocker is path feasibility through a narrow
turn, which is a nav-tuning question and not a localization one.

Drift also did not settle: 0.125 on one small-scale run and 0.049 on another,
against a 0.05 bound. **One run passing and one failing is not a pass**, and n=2
does not establish which is representative.

## Methods note: two instrument defects

Both were found by running the gate, not by reading it, and both would have
produced confidently wrong evidence.

**The withholding metric was blind to absence.** It measured gaps *between*
`odom_laser` messages, so the interval from drive start to the first message was
invisible. A run in which the matcher produced nothing whatsoever for 5.9 m
scored **"1 consecutive withheld update"** against a limit of 5 — comfortably
green. Counting the initial silence scores the same run 506. An instrument blind
to the absence of a signal rather than to its degradation is a general hazard,
and it very nearly passed this stack as healthy.

**The drift metric manufactured drift from sampling.** It compared the estimate
truncated at the midpoint *time* against half the total *distance*. Truth
crosses the halfway mark partway through a sample interval, so a perfectly
tracking estimate was charged a whole sample's travel as drift. Both tracks are
now truncated at the same instant.

Every figure in this study is computed by a module-level function with unit
tests (`test/test_corridor_sim_gate.py`), specifically so the numbers ADR 0027
rests on can be checked without standing up a GPU session.

## What is not claimed

- No mechanism for acquisition station. n=3, collinear predictors.
- No statement that robot2's chassis is unsuitable. The matcher is fleet-tuned
  for a 4x4 m room (OI-04 keeps its C1 performance open); a retune justifies
  re-running the same thresholds.
- No hardware result. Everything here is Isaac Sim 5.1.0.0 on an RTX 5070 Ti.
- Nav2's aborts are not separated from "the goal was unreachable in the map
  built so far".
