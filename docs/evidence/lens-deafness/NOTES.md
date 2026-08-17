# The deafness repro that did not reproduce — three clean arms

**Finding, in bold because it is negative: the synthetic churn does NOT
reproduce the deaf lens.** Three escalating arms, 1,346 hard-killed
participants, 39 subscriber replacements, zero deafness. Hypothesis H1's
synthetic form is bounded, not confirmed; the mechanism claim in ADR 0040
rests on fleet precedent (OI-13) and the upstream Fast-DDS record, and the
empirical decider is the T1 batch.

## Environment

- No Isaac involved; no GPU; scratch domain **69** (deny-list untouched).
- ROS 2 Jazzy system install, default `rmw_fastrtps_cpp`, default transport
  (SHM enabled) in all three arms — the exact configuration every corridor
  run used before ADR 0040.
- Host also carried one unrelated `ros2-daemon` (domain 43); the cleanup
  sweep removes only segments no live process maps (`_dds_shm.py` rule).
- Tool: `tools/diagnostics/dds_churn_repro.py` at this commit. One 12 Hz
  360-beam `/scan` publisher (sensor QoS), one victim subscriber wearing
  the lens's matched-event instrumentation, churn children that subscribe
  `/scan` and die without cleanup, mimicking `simctl`'s
  `sim_target`/`probe_topics` helpers (`simctl:287,1056`).

## Commands and verdicts (2026-08-14, 14:47–15:16 CST)

All run from the symlinked fleet path with
`CORRIDOR_FLEET_SRC=/home/alexmint/Development/robot-fleet/src`,
`PYTHONNOUSERSITE=1`, `/opt/ros/jazzy/setup.bash` sourced. Raw verdicts
with full timelines: `out/evidence/lens-deafness/<arm>/repro.json`.

| arm | escalation | command (suffix) | churn | outcome |
|---|---|---|---|---|
| `shm-arm` | none: children `os._exit(0)` at rest every 2 s | `--out out/evidence/lens-deafness/shm-arm --duration 900` | 448 children | **clean** — pub 10,395 / sub 10,402, shm peak 59 |
| `shm-replace-arm` | + victim SIGKILLed and replaced every 45 s (the `reap_previous_lens` shape; Fast-DDS #5053's deaf party is the *restarted* subscriber) | `… --victim-replace-every 45` | 449 children, 19 replacements | **clean** — every generation heard promptly |
| `shm-kill9-arm` | + churn children SIGKILLed mid-traffic at ~1 s age | `… --churn-kill9 --victim-replace-every 45` | 449 children, 19 replacements, peak 68 segments | **clean** — final sweep removed 53 stale segments |

Deafness criterion (pinned in the tool and its tests): victim count static
≥15 s while the publisher advanced ≥60 messages over the same window —
silence alone convicts nobody. Every arm exits by sweeping only segments no
live process maps; `/dev/shm` ended at 2 fastrtps entries.

## What this does and does not say

- It does **not** kill H1: the real deaf runs involved Isaac's participant
  (megabyte-class image/scan segments — the #2790 exhaustion shape, not
  reached by kilobyte scans), simctl's real timing, and a 5-subscription
  tf2-carrying lens. The synthetic form tested the churn *pattern*, and the
  pattern alone is insufficient on this host and Fast DDS version.
- It does bound the fix's burden of proof: since the cheap repro cannot
  arbitrate SHM vs UDP-only, the T1 batch (≥8 real runs under ADR 0040's
  export) is the decider, and the matched-event log + shm census now in
  every run will classify any deafness that survives as matched-but-silent
  (transport) vs never-matched (discovery) on the spot.
- Two runs of the plan's escalation budget were spent as designed; the
  third combined both escalations. No further synthetic arms: per the
  session plan's skip-edge, the batch decides.
