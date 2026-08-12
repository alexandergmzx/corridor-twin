# The startup circle, measured — and it is not what any of us thought

**2026-08-12, three live runs**, `nominal_m6_n3`, robot1, domain 67, docking off,
lens up. Artifacts under `out/evidence/robot-a-gate/20260812-16*-robot1-nominal_m6_n3/`.

The instrument is `tools/corridor_startup_probe.py`: `/cmd_vel_raw` and
`/behavior_tree_log`, recorded live with the goal-send moment marked. Nothing in
either repository had ever subscribed to the second one.

## Three hypotheses, all falsified by the same log

| hypothesis | verdict | evidence |
|---|---|---|
| a stale `behavior_server` left on the domain drives it | **no** | `commands_before_goal: 0` on every run, with the domain preflighted clean |
| Nav2's stock recovery `Spin` fires early into empty costmaps | **no** | first `Spin` at **t = 173 s**, after `FollowPath → FAILURE`, when A is already parked at the far end |
| unspecified motion before the goal is active | **no** | zero commands of any kind before the goal, three runs out of three |

**Nothing commands A before it has a goal.** The brief's framing — motion before
the transit goal — is wrong, and the fix it implied (remove `Spin` from the
recovery set) would not have touched what the operator sees.

## What A actually does, from ground truth

Run `20260812-164701`, first 120 s, `/sim/ground_truth`:

| t (s) | x | y | yaw |
|---|---|---|---|
| 0.0 | 0.014 | 0.002 | 7.1° |
| 16.2 | 0.025 | 0.003 | 7.1° |
| 24.2 | 0.214 | 0.390 | **141.1°** |
| 32.3 | −0.184 | 0.322 | **−100.2°** |
| 40–80 | −0.19 | 0.29 | −100.1° *(stationary)* |
| 88.5 | −0.144 | 0.162 | −46.6° |
| 96.6 | 1.195 | −0.013 | 7.3° |
| 112.8 | 2.932 | 0.189 | 6.9° |

**253° of turning over 1.06 m of travel in the first 60 s**, ending 0.2 m
*behind* the spawn — then **56 seconds standing still** — then the real transit,
which works: A reaches within **0.0625 m** of the delivery standoff at t = 110 s.

So the circle is real, it is A's first commanded motion, and it costs roughly
90 seconds of a 255 s window. It is the controller's, issued after the goal, in
the window where the map is still nearly empty. The 56 s stall is DWB commanding
near-zero while the behaviour tree re-plans — `commands_total` keeps climbing
throughout, so it is not a stalled node.

## What was fixed underneath it, and what it bought

The lidar filter was starving SLAM for exactly this window.
`scan_frame_relay` validated every revolution against a hardcoded 4 × 4 m room
and, on the corridor, dropped essentially everything for ~21 s before disabling
itself for the rest of the run — measured in 56 of 62 sessions.

Two constants, both closed-room numbers, both now caller-supplied from the
corridor's own manifest:

| | before | after |
|---|---|---|
| wall model | stock 4 × 4 m room | 24 segments from the manifest, per profile |
| `MIN_VALID_BEAMS` | 200 of 360 | 120 *(this scene's median is 175; 96.5% clear it)* |

Replaying a run's own scans against the corridor's walls, **1.1%** are genuinely
impossible, median impossible-fraction **0.000** — the geometry test was always
capable; the beam count threw the scans away before it ran.

Live result on run three: `dropped 10 (349 passed)` and **no fail-open at all**.
The filter stays on for the whole run for the first time.

**It did not remove the circle.** 253° before, 253° after. The starvation was
real and is fixed; it is not the cause.

## Where that leaves it

The named culprit is the local controller, not the recovery set and not an
orphan. That is a different fix from the one U2 specified, and it is not made
tonight on a measurement this young: what is established is *which node* and
*when*, which is what U2 asked for.

Two other things improved on the same runs and are recorded where they belong:
midpoint drift **0.159 → 0.0097** after the wheel-radius calibration
(`NOTES-odometry-scale.md`), and the map's duplicate-wall reading is now scored
against a masked map whose oracle floor is 0.000.

The transit itself is sound. A gets to within 6.3 cm of B and then drives 3.4 m
past it, because with docking off the arrival gate is a map-frame goal the map
never satisfies — which is ADR 0029's open blocker, unchanged.
