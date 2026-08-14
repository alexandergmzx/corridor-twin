# A's measured speed profile, and the policy pinned from it

**Date** 2026-08-14. **ADR 0038.** Owner-ratified per ADR 0007.

## Command

```bash
source /opt/ros/jazzy/setup.bash
PYTHONNOUSERSITE=1 python3 tools/measure_speed_profile.py \
  ~/Development/robot-fleet/src/MicroROS/MicroROS-assets/bags/20260814-{085441,085843,090238,090635,093458,093853}-isaac-d67 \
  --out out/evidence/speed-profile/measured-profile.json
```

| | |
|---|---|
| Runs | six delivery runs, `nominal_m6_n3`, robot1, domain 67 |
| Source | `/sim/ground_truth`, 3128 samples per run, recorded by `simctl` |
| Truth basis | **secant over a ±0.30 m window of travel** |
| Scenario | scale 0.30 (ADR 0030), gates at 0.6/1.2/1.8/2.4/3.0 m |
| Isaac | 5.1, RTX 5070 Ti |
| Result | **PASS** — the pinned policy produces exactly one violation on the mean, slowest and fastest cases |

The runs are the same six as `../lens/first-instrument/`. Two of them were
watched by a blind lens (ADR 0037); that affected the instrument, not the
navigation, and the ground truth in the bags is unaffected.

## The profile

| gate | clear width | mean | slowest run | fastest run |
|---|---|---|---|---|
| 0.6 m | 1.65 m | 0.1967 | 0.1731 | 0.2066 |
| 1.2 m | 1.50 m | 0.1960 | 0.1891 | 0.2023 |
| 1.8 m | 1.35 m | 0.1689 | 0.1399 | 0.1818 |
| 2.4 m | 1.20 m | 0.1285 | 0.1125 | 0.1420 |
| 3.0 m | 1.05 m | 0.0807 | 0.0555 | 0.1028 |

All values m/s. Six crossings per gate, all six runs crossed all five gates.

**A is not a constant-speed vehicle.** It decelerates by a factor of 2.4
between the first gate and the last, which is Nav2's controller slowing into
the goal, so "A's cruise speed" is not one number and a single-figure limit
cannot be derived from one.

## Two facts worth recording

**`/sim/ground_truth` publishes zero twist.** Isaac's publisher fills the pose
and leaves `twist.twist.linear.x` identically 0.0 on all 18,768 samples across
the six runs. The `twist_*` columns of the artifact are that zero, not a
measurement, and any speed truth in this project must be differentiated from
position. Kept in the tool's output rather than dropped, so the next reader
sees the zero and its explanation together instead of rediscovering it.

**The v1 limits were unreachable, and scaling them does not help.** Robot1's
band is 0.056–0.207 m/s. v1's tiers are 0.8/1.2/1.5; scaled by 0.30 they are
0.24/0.36/0.45. Every tier of either sits above everything the robot does.

## The pin, and its verification

Widest tier first: **0.30 / 0.25 / 0.04 m/s**, zone boundaries untouched.
One constant, `PINNED_LIMITS_MPS` in `tools/scale_scenario.py`; printed by the
generator; stamped into the derived scenario's header; asserted against the
manifest the observer loads.

Verified by `src/police_observer/test/test_pinned_policy_against_the_measured_profile.py`,
which runs the shipped `ViolationDetector` over the profile rather than
re-deriving the verdicts:

| check | result |
|---|---|
| Zone membership unchanged by the pin | strict = {2.4, 3.0}, as ADR 0016 decided |
| Every permissive gate clears the **fastest** run | 0.207 < 0.30, 0.202 < 0.25, 0.182 < 0.25 |
| Two-gate floor on the **slowest** run | 0.1125 and 0.0555 both > 0.04 |
| Strict limit is not the governor's creep clamp | 0.04 ≠ 0.05 |
| Violations, mean / slowest / fastest | **1 / 1 / 1**, each confirmed at gate 3.0 |
| Episode open at route end | emits nothing further; compliance still rearms |

Uncertainty is not in these margins. Every verdict assumes a perfect
estimator; what the real per-gate σ does to them is measured in
`../estimator/`.

## **A defect this found**

Running the checker rather than trusting the table found that **ADR 0016's
two-gate floor had not held since ADR 0030, and no test saw it.**

`MarkerMap.width_at(2.4)` returns `1.2000000000000002`, so a bare
`width <= 1.2` put that gate in the permissive zone. The strict zone held one
gate, `consecutive_estimates` is 2, and **a corner-confined violation could
never have been confirmed on the as-run scenario** — under this policy or any
other. The demonstration's central claim would have produced zero events with
nothing to point at.

Scale-dependent, which is why it hid: at v1's authored metres the same
expression, `6.0 + (8.0 / 12.0) * (3.0 - 6.0)`, is exactly `4.0`. ADR 0016's
arithmetic was correct when written; ADR 0030's 0.30 scaling broke it silently
while every v1 test stayed green.

Fixed as a one-nanometre tolerance on the boundary comparison
(`covered_by`), with a regression test at both scales. The threshold does not
move — that is ADR 0016's decision, and it is not reopened.

## Artifact

`measured-profile.json` — per-run per-gate crossings (time, x, twist, secant,
window travel and span) and the six-run summary. Generated under
`out/evidence/speed-profile/`; this note and the summary are what is promoted.
