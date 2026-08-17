# The startup circle: what it actually was, and three clean starts

**2026-08-12 17:26–17:56.** Follows
[`NOTES-startup-circle.md`](NOTES-startup-circle.md), which measured the
behaviour and named the local controller. That attribution was **wrong**, and
this records what the bag says instead.

## It was the health check

`check_isaac_contract.py` says so in its own docstring: it commands **0.12 m/s**
*"so that /cmd_vel is proven to actually move the robot"*, for the first half of
its window, at **0.3 rad/s** — a 0.4 m-radius arc. The corridor runner calls it
as a precondition, before SLAM and before Nav2.

From the bag of run `20260812-164717`, which settles it:

| topic | first message |
|---|---|
| `/cmd_vel` (governor's OUTPUT) | **moving at t = 16.31 s** |
| `/cmd_vel_raw` (every legitimate driver) | **t = 86.05 s** |

802 moving commands at a constant `(0.120, 0.300)` from t = 16.31 s to
t = 31.28 s, integrating to **182° and 1.27 m**. Published straight to
`/cmd_vel`, bypassing the governor, **seventy seconds before Nav2 published
anything at all**. Ground truth turns 253° over 1.06 m and ends 0.2 m behind
spawn.

Three sessions read this as a Nav2 recovery, a stale `behavior_server`, and a
DWB critic. It was the precondition doing exactly what it says it does.

### Two hypotheses died on the way, and both are recorded

- **A chasing a moving map frame.** `map→odom` is *identity* — 0.000 m, 0.0° —
  for the first 88 s of the run, with SLAM's first correction at t = 48.5 s and
  its first non-zero one after t = 88 s. A was not correcting for a frame that
  moved.
- **An early recovery `Spin`.** The behaviour-tree log puts the first `Spin` at
  **t = 173 s**, after `FollowPath → FAILURE`, when A is already parked at the
  far end of the street.

### Why the instrument missed it

`corridor_startup_probe.py` watched `/cmd_vel_raw` only, and so reported
`zero_rotation_before_goal: true` on three runs while the robot was physically
turning 253°. It was reading the right topic for a well-behaved driver and the
wrong one for this. It now watches both, and reports
`moving_on_cmd_vel_directly` and `ungoverned_rotation_deg`.

## The fix, and what it costs

robot1's corridor runs pass `--speed 0.0 --turn 0.0`. The precondition still
measures every rate; it no longer drives.

**Stated rather than hidden:** the run no longer proves `/cmd_vel` moves the
robot. That proof exists twice elsewhere and closer to the metal — the
composer's forward-sign gate commands 0.2 m/s and measures ground truth on every
arena build, and the transit itself moves A seven metres — and the corridor
overrides this checker's verdict on every run anyway (`--allow-contract-fail`,
scan runs 14–16 Hz against a declared 12), so what it contributes here is a rate
report, and a rate report does not need motion.

## Acceptance: three consecutive runs, ground truth

Measured by `tools/startup_acceptance.py`, artifact `startup-acceptance.json` in
each run directory. Pre-goal rotation from `/sim/ground_truth`; goal-send taken
as the first `/cmd_vel_raw` message.

| run | pre-goal rotation | progress after goal-send | closest approach to the standoff |
|---|---|---|---|
| `20260812-172604` | **0.03°** | **1.59 s** | 0.0605 m |
| `20260812-173705` | **0.56°** | **1.50 s** | 0.0661 m |
| `20260812-174825` | **0.03°** | **1.47 s** | 0.1285 m |
| limit | < 45° | < 30 s | ≤ 0.15 m |

Three of three, on every criterion. Before the fix the same measurement read
**78.8°**.

The third run's 0.1285 m is the closest to its limit and the spread across the
three (0.06–0.13 m) is worth watching: the criterion is met, not comfortably.

## What this does NOT fix

A still drives **3.3–3.5 m past B** and stops there. With docking off the
arrival gate is a map-frame goal the drifting map never satisfies — ADR 0029's
open blocker, untouched by this. The map's duplicate-wall reading on these runs
is 1.00–1.56 m against a 0.20 m limit, scored on a masked map whose oracle floor
is 0.000.
