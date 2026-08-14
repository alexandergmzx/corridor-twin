# ADR 0040: Corridor sessions run DDS over UDP only

- **Status:** Proposed (accepts on the T1 batch: 8/8 lens-healthy runs)
- **Date:** 2026-08-14
- **Relates to:** [ADR 0035](0035-the-lens-is-the-first-instrument.md),
  [ADR 0037](0037-the-banner-means-seeing.md),
  [ADR 0039](0039-the-lens-is-asked-twice.md) — this record supplies the
  mechanism 0039 left open ("It does not stop the lens going deaf. The
  mechanism remains unidentified."). Fleet decision **OI-13**
  (`robot-fleet/docs/architecture.md`) owns the profile and decided
  UDP-only machine-wide; this ADR finishes its REMAINING item ("bake the
  export") for corridor sessions.

## Context

Across three sessions the lens went deaf on 2 of 4 recent runs (2 of 6 on
the morning placement): it served `/healthz`, passed or nearly passed its
seeing gate, and then received nothing for the rest of the run. Since ADR
0037 a twice-deaf lens refuses the run, so each deafness costs a full
Isaac load. ADR 0039 hardened the gate but could not name the mechanism.

The 2026-08-14 rework's research converged on one hypothesis with an
unusual amount of independent support:

1. **The failure signature matches Fast DDS's documented SHM-after-churn
   family.** Every deaf lens heard *something* very early (last message at
   t≈0.25–40 s, read from the freeze latch in each `lens.json`) and then
   nothing forever, including from publishers that started later.
   eProsima/Fast-DDS #5053: after process churn a subscriber "successfully
   discovers and matches with the publisher topic, but receives no data
   messages. UDP transport does not exhibit this problem." rclcpp #1831 is
   the same shape from the ROS side, and Fast DDS's own docs say a process
   that dies without cleanup leaves zombie segments, lock files and
   mutexes that `fastdds shm clean` exists to remove.
2. **The churn is real and identified.** ADR 0037 already named it:
   simctl's `sim_target`/`probe_topics` helpers are short-lived rclpy
   participants that subscribe and then `os._exit(0)` every ~10 s — the
   no-cleanup exit, by design. Run `133559`'s teardown removed **85**
   stale segments after one run.
3. **The fleet has measured the end state before.** OI-13 (2026-08-08):
   35 accumulated `/dev/shm/fastrtps*` segments made new participants on
   the sim domain *completely blind*; the UDP-only profile
   (`ground_station/fastdds_udp_only.xml`) fixed it at a measured cost of
   <1% on every contract rate with *better* worst-case inter-arrival gaps.
   Its remaining item — bake the export — was still open, and every
   corridor run to date ran default transport with SHM enabled (verified:
   no executable export of `FASTDDS_DEFAULT_PROFILES_FILE` existed
   anywhere in the run path).
4. **The no-Isaac repro reproduces it.**
   `tools/diagnostics/dds_churn_repro.py`: one 12 Hz `/scan` publisher,
   one victim subscriber carrying the lens's matched-event
   instrumentation, one churn loop of subscribe-then-`os._exit(0)`
   children on scratch domain 69. *(Verdict artifacts under
   `out/evidence/lens-deafness/`; the numbers are recorded in the
   evidence NOTES and quoted here at acceptance.)*

## Decision

1. **Every corridor session runs DDS over UDP only.**
   `tools/corridor_profile_run.sh` exports both
   `FASTDDS_DEFAULT_PROFILES_FILE` and `FASTRTPS_DEFAULT_PROFILES_FILE`
   (the XML's own compatibility note) pointing at the fleet's
   `ground_station/fastdds_udp_only.xml`, before any participant is
   created, so every child — simctl's stack, `sim_runner` under Isaac's
   python, SLAM, Nav2, the lens, the recorder, every probe — inherits it.
   The profile is OI-13's file, referenced, never copied.
2. **`CORRIDOR_DDS_PROFILE` overrides; empty disables.** The empty form
   exists for exactly one purpose: the A/B control arm of a measurement.
   A run's `run.json` records `dds_profile`, so every artifact says which
   transport it measured.
3. **The scope is corridor sessions.** The P-plane demo (`run_demo.sh`,
   the gateway crossing) is *not* switched by this ADR: ADR 0026's
   crossing ratios were measured under default transport, and quoting
   them for a UDP-only demo without re-measuring would violate the
   evidence discipline. That re-measure is a named follow-up, not a
   side effect.

## Consequences

- The deafness mechanism stops being folklore: matched-event logs
  (in the lens since this rework) plus the repro's A/B give it a
  reproduction, a discriminator, and a kill.
- Corridor runs no longer share a failure domain with every other DDS
  process on this host through `/dev/shm` — the repro deliberately
  manufactures the poison and cleans up only what no live process maps.
- Loopback UDP replaces SHM for same-host delivery. OI-13 measured the
  cost on the fleet's stack (<1% on rates); the corridor batch records
  robot1's own contract rates per run, so the corridor-specific cost is
  measured, not assumed.
- simctl's `os._exit(0)` children remain unfixed (fleet-owned code); under
  UDP-only their churn is harmless to corridor participants. Fixing the
  exit path is a fleet backlog item, not a corridor gate.

## Validation (filled at acceptance)

- The churn repro's two verdict files (SHM arm, UDP-only arm).
- The T1 batch: ≥8 consecutive runs with zero deaf lenses (no
  restart-once firings, no lens refusals, `lens_resolved_frac > 0`),
  with per-run `shm_fastrtps_*` censuses and robot1 contract rates
  against their pinned values.
