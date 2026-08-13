# The autonomy acceptance runs, 2026-08-12 evening

**RED on both gated profiles.** The reds are recorded, not tuned around, and
they are the same two ADR 0029 named: the map diverges, and the fusion's yaw
scale wanders. What is new is that everything else in the transit now passes.

```
bash tools/corridor_profile_run.sh --profile <p> --robot robot1 \
  --gated --no-dock --allow-contract-fail
```

robot1, domain 67, docking **off**, lens up, arena and manifest hash-checked
against each other before every run. Per-run artifacts under
`out/evidence/robot-a-gate/<stamp>-robot1-<profile>/`; the whole session is
tabulated in [`session-runs-20260812.json`](session-runs-20260812.json).

## The two gated profiles

| | `nominal_m6_n3` | `wide_corner_m6_n4_5` | gate |
|---|---|---|---|
| run | `20260812-184944` | `20260812-185907` | |
| classification | result | result | |
| **closest approach to the standoff** | **0.110 m** | **0.020 m** | ≤ 0.15 m ✅ |
| midpoint drift | 0.0137 ✅ | 0.0606 ❌ | ≤ 0.05 |
| **yaw scale** | **1.166** ❌ | **1.108** ❌ | 1.0 ± 0.1 |
| worst EKF gap | 0.242 s ✅ | 0.200 s ✅ | ≤ 0.4 s |
| **duplicate wall** | **0.840 m** ❌ | **0.780 m** ❌ | ≤ 0.20 m |
| startup criterion | pass (0.61°, 1.84 s) ✅ | pass (0.04°, 1.50 s) ✅ | < 45°, < 30 s |
| scan filter | 349/10, no fail-open ✅ | 560/10, no fail-open ✅ | accepts from scan 1 |
| **verdict** | **FAIL** | **FAIL** | |

## What passes now, and did not this morning

- **A arrives.** Closest approach 0.020–0.178 m across every run that drove,
  median ~0.11 m, six of seven inside ADR 0022's 0.15 m tolerance. This morning
  the same measurement read 5.754 m, because the plan and the arena were
  different scenes.
- **A leaves cleanly.** The startup criterion passed on every run that measured
  it — pre-goal rotation 0.03–0.61° against a 45° limit, forward progress within
  1.5–1.8 s of goal-send. Before the fix: 78.8°.
- **The lidar filter holds.** 560 passed / 10 dropped on `wide_corner`,
  349 / 10 on `nominal`, and **no fail-open on any run** since the wall model
  and beam count came from the manifest. Every corridor run before today ran
  with filtering disabled after the first ~21 s.
- **Longitudinal drift is small**: 0.0024–0.0384 on nominal, against 0.159 this
  morning and a 0.05 gate.

## What fails, and it is one thing wearing two hats

**Duplicate wall, 0.78–0.84 m against 0.20**, on a masked map whose
perfect-SLAM oracle reads 0.000 (ADR 0030). This is real divergence, not the
scene's own geometry.

**Yaw scale 1.11–1.17 against 1.0 ± 0.1**, and across the night's eight
nominal runs the ratio ranged **0.808 – 1.248**.

That band is itself a finding. ADR 0029 recorded **0.14× – 23.4×** across seven
runs — a filter claiming ten revolutions the robot never made. Tonight it is
0.81–1.25. Still failing the gate, and a different order of fault; the wheel
radius calibration (`NOTES-odometry-scale.md`) is the only change between the
two measurements. **No claim is made that it caused the improvement** — nothing
here was an A/B, and the fusion anomaly's own record shows it varies run to run.

`wide_corner`'s drift of 0.0606 is the one number that got worse than nominal's,
and one run is not a profile comparison.

## uniform — the transit gate PASSES

Reported, never gated (ADR 0022): it is the degeneracy study and is expected to
struggle. It did not.

| | `uniform_m6_n6`, run `20260812-191347` | gate |
|---|---|---|
| **transit gate** | **PASS — no failures at all** | |
| closest approach | 0.083 m | ≤ 0.15 ✅ |
| midpoint drift | **0.0041** | ≤ 0.05 ✅ |
| **yaw scale** | **1.060** | 1.0 ± 0.1 ✅ |
| worst EKF gap | 0.200 s | ≤ 0.4 ✅ |
| first `odom_laser` station | 0.0001 m | matcher alive from the start |
| startup criterion | pass (0.34°, 1.47 s) | ✅ |
| scan filter | 673 passed / 10 dropped, no fail-open | ✅ |
| map | **not scored — the save failed** | — |

**This is the first fully green transit gate of the session**, and it is the
profile ADR 0027's degeneracy study expected to be the hardest. The untapered
corridor is the easiest case for the scan matcher, so this is not a surprise so
much as a confirmation that when the matcher is healthy every other number in
the gate falls into place — including the yaw ratio, which is inside ±0.1 here
and outside it on both tapered profiles.

The run is still marked FAIL because `map_saver_cli` died with *"Failed to spin
map subscription"* after 20 s, so no map existed to score. That is an
infrastructure failure of the SAVE, not a statement about the map: **2 of 16
runs tonight lost their map this way.** Worth its own look; it is not the
divergence.

Its first attempt (`20260812-190623`) is a `rerun`: bt_navigator never reached
ACTIVE in two tries. Both attempts' artifacts are kept.

## What was not run

**Two of sixteen runs lost their saved map** to `map_saver_cli` failing to spin
its subscription, which is how a green transit ends up recorded as FAIL. Not
chased tonight.

**No docked demo-candidate run.** The brief made it conditional on the gates
being green, and they are not. The containment that would make that run safe
exists and is tested (`NOTES-startup-fixed.md`, ADR 0030), but it has not been
exercised live.

## Reading these runs honestly

**Thirteen runs: six results, six reruns, one that never reached a verdict.**
That ratio is a finding in itself — roughly half the Isaac sessions on this box
produce no statement about the robot. The reruns break down as three watchdog
kills at the old 420 s cap, two nav-stack bringup races (bt_navigator never
ACTIVE, or ACTIVE then rejecting), and one nav gate that crashed on a defect
introduced earlier the same evening.

One run — `20260812-183327` — carries `result` and should be a `rerun`. It moved
0.129 m because the goal was refused by an inactive action server, and it
predates the rule that reads `ground_truth_distance_m` to tell that case from a
lost acceptance response. History is append-only, so its record stands and this
says so.
