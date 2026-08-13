# A does not ignore B. It arrives, is told it is 1.25 m short, and keeps driving

**2026-08-13, offline from ten session bags across all three profiles, plus 22 recorded runs.** Watched live on the lens
first: A runs past B and out of the corridor. The artifacts had been reporting
`closest_approach_m 0.03` and being read as success.

```bash
source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
BAGS=~/Development/MicroROS/MicroROS-assets/bags
PYTHONNOUSERSITE=1 python tools/diagnostics/at_closest.py        $BAGS/20260813-102705-isaac-d67
PYTHONNOUSERSITE=1 python tools/diagnostics/travel_registered.py $BAGS/20260813-102705-isaac-d67=nominal_m6_n3
PYTHONNOUSERSITE=1 python tools/diagnostics/axis_all.py          $BAGS/20260813-102705-isaac-d67=nominal_m6_n3
```

See [`tools/diagnostics/README.md`](../../../tools/diagnostics/README.md) for
which script answers which question.

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
| drove to the street's far wall | **13** | x 4.88 – 5.12, y −5.59 … −5.89 | 3.18 – 3.51 m |
| stopped near B | 4 | x 4.15 – 4.94, y −1.61 … −3.59 | 0.30 – 1.03 m |
| never moved (bring-up failures) | 5 | ≈ spawn | 0.00 m |

The street's south end is y = −6.0. **The majority case is A driving until the
geometry stops it**, which is why those thirteen all land within 0.30 m of each
other: the distance is set by the wall, not by the size of the pose error.
Recomputed from the artifacts rather than counted by eye — an earlier version of
this table said 14 / 4 / 4 and was wrong in two cells.

**It is not a `nominal` behaviour.** The one `uniform_m6_n6` run driven for this
diagnosis (`20260813-113843`) did the same thing: closest approach 0.112 m,
walked away **3.346 m**, final position (5.090, −5.796) — the same wall, within
0.1 m of where the `nominal` runs stop.

So the pose error explains *why A keeps going*; the wall explains *where it
ends up*. The one suggestive datum on magnitude is that the run with the
smallest measured pose error (`20260813-110947`, 0.798 m) is also the one with
the smallest walk-away (0.298 m) — **n = 6 measured, so that is a hint, not an
established relationship**, and it is not claimed as one.

## The fault, in one number: SLAM sees a fifth of the motion

Over the **first 2 m of travel**, how far each estimator thinks A went:

| profile | bags | EKF / truth | **SLAM / truth** |
|---|---|---|---|
| `nominal_m6_n3` | 6 | 0.986 – 1.050 | **0.365 – 0.692** |
| `wide_corner_m6_n4_5` | 2 | 0.991 – 1.021 | **0.324 – 0.723** |
| `uniform_m6_n6` | 2 | 0.940 – 0.985 | **0.128 – 0.198** |

**The odometry is right and the matcher is not.** The EKF measures A's first two
metres to within 6% on all ten bags. The scan matcher registers between
**13% and 72%** of the same motion — and SLAM then "corrects" the good
odometry toward its own estimate. That is the whole mechanism: everything below
is its shape.

`uniform_m6_n6` — the profile with **no taper at all** — is the worst on both
its bags, below every `nominal` and `wide_corner` value. That is consistent with
a tapering wall giving the matcher a distance-dependent signature that parallel
walls do not, so the taper *helps*. With n = 2 for uniform and the other two
profiles overlapping heavily (0.32–0.72), **an ordering by profile is a
supported hypothesis, not an established result**, and it is not claimed as one.

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

**And it is universal.** Over the straight approach leg, across **10 bags and
all three profiles** (`tools/diagnostics/axis_all.py`):

| profile | peak ALONG | peak ACROSS | along/across | half-error reached by |
|---|---|---|---|---|
| `nominal_m6_n3` ×6 | −0.76 … −1.91 m | −0.23 … +0.12 m | 7 – 34× | 0.59 – 1.92 m |
| `wide_corner_m6_n4_5` ×2 | −1.17, −1.50 m | +0.03, −0.02 m | 39×, 72× | 0.97, 1.06 m |
| `uniform_m6_n6` ×2 | −1.71, −2.11 m | +0.018, +0.030 m | 97×, 70× | 1.00, 1.38 m |

Every bag: the along-axis error is **negative** and between 7× and 97× the
across-axis error, and half of it is present within two metres of travel. The
across-axis figure is computed on the approach leg only — past the corner most
runs leave the corridor and it explodes, which measures the excursion rather
than the corridor.

Three things follow, and each is worth stating separately:

1. **The error is longitudinal.** Across-corridor it stays under 0.16 m — the
   two walls constrain that direction well. Along the axis it reaches −1.26 m
   here and −2.11 m at worst. The sign is negative: **SLAM places A behind
   where it is**, which is exactly the error that makes a robot keep driving.
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

- **Ten** bags, robot1, domain 67: six `nominal_m6_n3`, two
  `wide_corner_m6_n4_5`, two `uniform_m6_n6`. The 22-run population table is
  `nominal` only.
- The time-resolved trace (two discrete jumps) is from ONE of those ten
  (`20260813-102705`). The endpoint and the axis decomposition are confirmed on
  all ten; **the two-jump shape is not**.
- `uniform_m6_n6` has n = 2. Its being worst is consistent and unexplained,
  not established.
- The world→map transform used here is a pure rotation by A's spawn heading,
  valid because map and world share the spawn origin. If a future scenario
  spawns A elsewhere this analysis needs the translation too.
- The two jump magnitudes are read at 2 s sampling, so their timing is ±2 s and
  their number is a lower bound.
- **Why the matcher under-registers** is not answered here. Worth noting
  against the obvious guess: the corridor is 3.6 m long at the committed scale
  and the MS200 reports 0.12–8.0 m, so the far wall IS in range throughout —
  "no longitudinal feature within range" does not survive contact with the
  contract check's own output. That makes the cause more interesting, not less,
  and it is the degeneracy study's question.
