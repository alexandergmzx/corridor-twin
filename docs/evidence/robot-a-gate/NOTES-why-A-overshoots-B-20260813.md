# A does not ignore B. It arrives, is told it is 1.25 m short, and keeps driving

**2026-08-13, offline from six session bags plus 22 recorded runs, no GPU.** Watched live on the lens
first: A runs past B and out of the corridor. The artifacts had been reporting
`closest_approach_m 0.03` and being read as success.

```bash
python3 <scratch>/at_closest.py <bag>     # what Nav2 believed at arrival
python3 <scratch>/axis.py <bag>           # the error, along vs across
```

## The mechanism

**A reaches the delivery standoff to within 3–12 cm on every run.** It is not
ignoring B and it is not failing to navigate. At that exact moment, SLAM's pose
is **1.2–2.2 m wrong**, so Nav2's goal checker sees a position error of over a
metre and keeps commanding forward motion. A physically overshoots. As it
overshoots it leaves the mapped corridor, the pose error grows further, and the
goal recedes faster than A closes on it. It ends against the street's far wall.

| run | physically from standoff | SLAM pose error there | goal-check xy | goal-check yaw |
|---|---|---|---|---|
| `20260813-102650` | 0.032 m | **1.332 m** | 1.361 FAIL | 76.9° FAIL |
| `20260813-000546` | 0.054 m | **2.201 m** | 2.246 FAIL | 64.0° FAIL |
| `20260813-015009` | 0.029 m | **1.453 m** | 1.482 FAIL | 58.7° FAIL |
| `20260812-184944` | 0.110 m | **1.717 m** | 1.632 FAIL | 63.1° FAIL |
| `20260813-035938` | 0.123 m | **1.242 m** | 1.149 FAIL | 62.5° FAIL |
| `20260813-110947` | 0.124 m | **0.798 m** | 0.691 FAIL | 59.8° FAIL |

Six runs, one mechanism, no exceptions. Tolerances are
`xy_goal_tolerance: 0.15`, `yaw_goal_tolerance: 0.6`
(`config/robot1/nav2_robot1_corridor.yaml:66-67`).

### Where it stops is the wall, not the error

Across **22 recorded `nominal_m6_n3` runs**, `walked_away_m` is bimodal, not
proportional:

| outcome | runs | final position | walked away |
|---|---|---|---|
| drove to the street's far wall | **14** | (5.0 – 5.1, −5.6 … −5.9) | 3.18 – 3.51 m |
| stopped near B | 4 | (4.1 – 4.9, −1.6 … −2.9) | 0.30 – 1.03 m |
| never moved (bring-up failures) | 4 | ≈ spawn | 0.00 m |

The street's south end is y = −6.0. **The majority case is A driving until the
geometry stops it**, which is why those fourteen all land within 0.4 m of each
other: the distance is set by the wall, not by the size of the pose error.

So the pose error explains *why A keeps going*; the wall explains *where it
ends up*. The one suggestive datum on magnitude is that the run with the
smallest measured pose error (`20260813-110947`, 0.798 m) is also the one with
the smallest walk-away (0.298 m) — **n = 6 measured, so that is a hint, not an
established relationship**, and it is not claimed as one.

## The error is ALONG the corridor, and it is born in the first two metres

Decomposed onto the corridor axis (map +x is A's spawn heading), run
`20260813-102650`:

| A has driven | error ALONG | error ACROSS |
|---|---|---|
| 0.73 m | **−0.563 m** | 0.022 m |
| 1.54 m | **−0.875 m** | 0.054 m |
| 2.16 m | **−1.264 m** | 0.075 m |
| 2.6 → 4.1 m | −1.19 … −1.40 m | 0.07 … 0.16 m |
| 4.17 m (out of the corridor) | −1.114 m | 1.247 m |

Three things follow, and each is worth stating separately:

1. **The error is longitudinal.** Across-corridor it stays under 0.16 m for the
   whole transit — the two walls constrain that direction well. Along the axis
   it reaches −1.26 m. The sign is negative: **SLAM places A behind where it
   is**, which is exactly the error that makes a robot keep driving.
2. **It is established in the first ~12 s of motion**, over the first 2.2 m of a
   7.4 m route, in two discrete steps (+0.41 m at 0.73 m travelled, +0.32 m at
   2.16 m). Then it holds flat at ≈ −1.25 m for the rest of the corridor. Steps,
   not creep.
3. **The correction is the error.** `map→odom` tracks the pose error almost
   exactly (1.24 vs 1.266, 1.312 vs 1.313 …), and the EKF's own longitudinal
   drift over the same run is **0.0044** — 2.3 cm over 5.3 m. **The odometry is
   good and SLAM is "correcting" it by 1.25 m in the wrong direction.**

This is the corridor's own degeneracy — a long, straight, near-featureless
passage gives the scan matcher nothing to constrain travel *along* it. It is the
subject of [`docs/degeneracy-study.md`](../../degeneracy-study.md), predicted
before it was measured.

**It amends ADR 0029's location claim.** That record says *"The corridor is
clean… The failure is at the far end. A reaches the delivery standoff and the
map degrades around and after that point."* Measured here, the failure is
established **before A is a third of the way down the corridor** and is flat
thereafter. The far-end growth is real but it is the second act.

## And a second, independent defect the first one was masking

The counterfactual, computed on the same samples: **if SLAM had been perfect, the
goal still would not have completed.**

| | with the measured map | with a perfect map |
|---|---|---|
| xy check | 1.361 m → FAIL | 0.032 m → **PASS** |
| yaw check | 76.9° → FAIL | **81.1° → FAIL** |

`corridor_nav_gate.py:301` sends `goal.pose.pose.orientation.w = 1.0` — a goal
yaw of **zero**, i.e. "arrive facing along the map's +x axis", which is back up
the corridor. A arrives facing −51° to −74° (south, having turned onto the
street). With `stateful: true` the checker tests position first, so the xy
failure has been hiding this the whole time: **the delivery goal has never
carried a reachable orientation.**

Cheap, corridor-side, and independent of the map. It does not fix the overshoot
on its own — the xy check still fails by 1.36 m — but leaving it in place means
the arrival gate cannot go green even if the map problem is solved.

## What this changes

- **D1 is the right fix and this is its justification.** The landmark
  measurement is taken in the laser frame and is the only quantity in the system
  immune to a 1.25 m pose error. The map-frame containment test D1 deletes fails
  for precisely this reason — 2812 rejections on a run where A's own laser was
  measuring B correctly at 0.63 m with a fitted radius of 0.1244 against an
  authored 0.12.
- **The arrival gate has two blockers, not one.** Both need naming in whatever
  ADR closes it; fixing the map alone would leave a 81° yaw failure behind.
- **No `slam_toolbox` or Nav2 parameter was touched** — ADR 0029's standing law
  holds. This is a diagnosis.

## Scope and limits

- Six bags analysed in depth, all `nominal_m6_n3`, robot1, domain 67; the
  population table covers 22 recorded runs of that profile. The along/across
  decomposition is computed on ONE bag — its shape is not shown to be
  universal, only its endpoint is, via the six-run arrival table.
- The world→map transform used here is a pure rotation by A's spawn heading,
  valid because map and world share the spawn origin. If a future scenario
  spawns A elsewhere this analysis needs the translation too.
- The two jump magnitudes are read at 2 s sampling, so their timing is ±2 s and
  their number is a lower bound.
- **Why** the matcher slides backwards specifically — the taper, the scan
  filter's 0.12 m minimum, the 12 m range against a 3.6 m corridor — is not
  answered here. That is the degeneracy study's question and it is still open.
