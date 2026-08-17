# The linear scale, measured on thirteen bags instead of one window

**2026-08-12, offline.** No GPU, no simulator, nothing driven.

```
python3 tools/odometry_scale_audit.py \
  --bags ~/Development/MicroROS/MicroROS-assets/bags/20260812-*-isaac-d67 \
  --out out/evidence/robot-a-gate/odometry-scale-20260812.json
```

Artifact: [`odometry-scale-20260812.json`](odometry-scale-20260812.json).
Sources: `/odom_raw` (wheel), `/odom` (EKF, `pn-fix`), `/sim/ground_truth`.

## The −15.9% does not reproduce

It came from one run's `midpoint_drift`: 0.6491 m of estimate against 0.7719 m
of truth, over **0.77 m of travel** on a run whose total was 1.5 m — a ratio of
two short, noisy path-length sums, taken in an arena that was the wrong scale
anyway. It is not carried forward.

Measured properly — distance accumulated in 1 s windows, windows where truth
moved less than 0.02 m dropped, median of the per-window ratios:

| source | median of bag medians | min | max | spread |
|---|---|---|---|---|
| wheel `/odom_raw` | **0.9463** | 0.848 | 1.144 | 0.296 |
| EKF `/odom` | 0.9554 | 0.863 | 1.112 | 0.250 |

**About −5%, and the run-to-run spread is six times the bias.** A single
straight-line transit cannot verify a ±3% target against a measurement that
scatters by ±0.15 — the verification U4c asks for needs several runs or a longer
one, not one more.

## Separating turning from straight settles what kind of error it is

Wheel odometry on a four-wheel skid-steer slips most while turning; a drive
conversion error does not care whether the robot is turning. Splitting the
windows at 5° of heading change per second:

| source | all windows | **straight only** | straight spread | bags |
|---|---|---|---|---|
| wheel | 0.9463 | **0.9366** | 0.094 | 7 |
| EKF | 0.9554 | 0.9068 | 0.282 | 7 |

On straight driving the wheel channel under-reports by **6.3%, on every one of
the seven bags long enough to have straight windows** — maximum 0.946, so not
one of them reaches parity — and the spread tightens from 0.296 to 0.094.

That is the signature of a **systematic conversion error, not slip**. Slip makes
wheels over-report (they turn further than the body travels); this under-reports
while driving straight, consistently, which is a rolling radius that is too
small.

| bag | truth (m) | wheel, all | wheel, straight |
|---|---|---|---|
| `20260812-090428` | 17.012 | 0.848 | 0.852 |
| `20260812-092904` | 17.770 | 0.914 | 0.937 |
| `20260812-094008` | 11.562 | 0.937 | 0.928 |
| `20260812-104057` | 9.159 | 0.924 | 0.913 |
| `20260812-122616` | 13.035 | 0.941 | 0.942 |
| `20260812-123839` | 10.028 | 0.947 | 0.946 |
| `20260812-130918` | 2.745 | 0.946 | 0.940 |

The six short bags (1.3–2.4 m of travel) have no straight windows at all and
their all-window ratios scatter from 0.898 to 1.144. **Short runs cannot measure
this**, which is what the single-window −15.9% was.

## Where the number lives, and what it implies

`sim_runner.py:69` — `WHEEL_R = 0.0458`, the "effective rolling radius", against
a geometric `WHEEL_R_GEOMETRIC = 0.0245` at `:51`. It is used twice: to
integrate `/odom_raw` from joint velocities (`:720`) and, as a copied constant
at `build_corridor_arena.py:140`, to command wheel speeds.

At a measured 0.9366 the implied effective radius is **0.0489 m**, +6.8% on the
configured 0.0458.

Two consequences, and the second is not about odometry at all:

1. The odometry under-reports distance by 6.3%.
2. The same constant converts a commanded velocity into wheel speed, so **A
   drives about 7% faster than Nav2 asks it to.** A speed policy pinned against
   commanded speed would be pinned against the wrong number.

## Applied, and independently confirmed by a measurement it did not design

`WHEEL_R` 0.0458 → **0.0489**, fleet-side, one commit
(`yahboomcar-ros2` `52bf989`), listed separately for separate review.

The composer's own forward-sign gate is an open-loop straight-line transit
against ground truth — command 0.2 m/s for 2 s, no Nav2, no governor, no filter
— and it has been running on every arena build all along:

| | nominal | wide_corner | uniform | vs the 0.400 m commanded |
|---|---|---|---|---|
| at `WHEEL_R = 0.0458` | 0.418 m | 0.417 m | 0.418 m | **+4.4%** |
| at `WHEEL_R = 0.0489` | 0.391 m | 0.392 m | 0.392 m | **−2.1%** |

Same direction as the bags and a different method: both say the true rolling
radius is larger than the constant, by 4.4% (the 2 s gate) and 6.8% (the
thirteen bags). They do not agree exactly, and the honest reading is that they
bracket it — the gate implies 0.0478, the bags 0.0489. The bag-derived value is
the one applied, because it is measured over hundreds of seconds of governed
driving rather than one 0.4 m open-loop push on a chassis that is still
settling.

**The bar was ±3% and the post-change direct measurement is −2.1% on all three
profiles**, so the calibration stands rather than being reverted. A governed
transit re-measures it in U5.

## What this does NOT say

- **Nothing about yaw.** The fusion anomaly (`NOTES-fusion-anomaly.md`) is a
  rotation problem and this measurement does not touch it. Wheel and EKF linear
  ratios track each other (0.9463 / 0.9554), so the linear channel is not where
  the filter misbehaves.
- **Not a chassis verdict.** 0.0458 is itself an empirical constant whose 1.87×
  ratio over the geometric radius is unexplained fleet-side; this measurement
  says that constant is 6.8% off, not why it is 1.87× in the first place.
- **The correction is not applied here.** `WHEEL_R` is fleet-side code that
  every robot1 Isaac session uses, not corridor configuration. Changing it is a
  fleet decision with fleet-wide blast radius, and it is parked for one.
