# Session plan — the lens comes up first, and bring-up stops failing invisibly

**Branch** `bringup-lens-2026-08-14`, from `00183e5`.
**Started** 2026-08-14 04:30 CST. **Budget:** stop starting new units at 07:30,
handback by 08:00. `date` between units.

Unattended hard rules bind: append-only, green-checkpoint commits, Isaac
single-occupancy, park-don't-decide, **nothing pushed**.

**Process-check discipline, restated because it drew blood:** `pgrep`/`pkill`
patterns match their own command line. This cost four self-matches last session
and the last one killed my own shell mid-cleanup. Every occupancy or liveness
check in this session uses a `/proc` scan, or builds its pattern from a variable
so the literal never appears in the command line.

---

## Why this session exists

The previous session made the bump work — A touched B, measured (truth
0.2146 m against a 0.2175 m contact). But **only 2 of 5 runs reached the
docking phase**, and the operator could not watch any of them. The complaint
was "faux launches": handed a lens URL for a run that is already dead, or given
no lens at all.

The complaint is literally true in the code:

- The lens starts at `corridor_profile_run.sh:955`, **after** SLAM and Nav2.
  **Ten of the twelve `rerun()` exits precede that line.** A run that dies in
  bring-up writes no `lens.log` at all — `20260814-022725`, `-023029`, `-025555`.
- `:968` prints the URL **unconditionally**. The health poll breaks on success
  *or* on the lens dying, and only ever checks 8765 while the lens walks
  8766-8770 on `EADDRINUSE`.
- The lens is killed 5th in `teardown()` (`:411`), before `map quality` and the
  startup criterion. Measured serving window: 127 s and 116 s of ~6-min runs.

None of it is required: the lens is read-only by construction, every payload is
optional, every TF lookup is zero-timeout, and `/healthz` is served
independently of ROS. The position at line 955 arrived in one commit with no
rationale and no ADR.

## Two findings that reshaped the bring-up half

**The orphan CAUSES the SLAM double-failure.** Run `025555`: attempt 1 logged
`failed to send response to /slam_toolbox/change_state (timeout)`; the runner
TERMs only the `ros2 launch` pid — which `:429-437` already records as not
propagating — so attempt 1's node survived and **attempt 2 burned its full
110 s talking to attempt 1's orphan**. Reaping must land before faster
detection, or better detection only makes the orphan fresher.

**A log oracle for SLAM is perfect over 82 archived launches**: `Managed nodes
are active` → 78/78 activated; the change_state WARN → 3/3 failed; one silent
hang (the orphan). Zero false either way. Healthy activation is **1.26 s**
against a 110 s deadline.

**A map-divergence gate would be actively harmful.** All five runs exceeded
`MAX_DUPLICATE_WALL_M = 0.20` (0.76-1.58 m), and the *worst* map produced the
*best* approach — `031348`, 1.580 m, the run that touched B. It becomes a
recorded covariate, never a gate.

## Ratified this session

| | ruling |
|---|---|
| **D1** | **The lens outlives the run.** Starts before the simulator, serves through teardown, freezes ~60 s after data stops, next run reaps and replaces it. `corridor_lens.py` leaves `residents()`. Precedent: `simctl:1043-1048` already calls the lens "left open across sessions by design" — the corridor regressed that. |
| **D2** | **A lens that cannot serve refuses the run** (`rerun`), naming `--no-lens`. CLAUDE.md:439-441 already makes it mandatory equipment. |
| **D3** | Full scope, including a second handoff trigger on Nav2 SUCCESS (ADR 0036). |

D1 + D2 force the placement: the lens starts **before** `simctl start`, or a
refusal throws away 70 s of Isaac load.

---

## Queue

| # | unit | box | status |
|---|---|---|---|
| V0 | Branch + this document | 15 m | **DONE** |
| L1 | Dump written as measured, not as a farewell | 30 m | |
| L2 | Windowed rates + history covering a session | 45 m | |
| L3 | Lens before the sim; banner = verified port; absent lens refuses | 60 m | |
| L4 | Lens outlives the run + freeze + ADR 0035 | 60 m | |
| B1 | Phase labels (SLAM, map save, preconditions) | 20 m | |
| B2 | Failed attempts reaped as a process group | 60 m | |
| B3 | SLAM poll reads the log (oracle replay over 82 logs) | 45 m | |
| B4 | A run that never hands off says so | 45 m | |
| B5 | Map score as a `run.json` covariate | 20 m | |
| B6 | Nav2 SUCCESS as a second handoff trigger + ADR 0036 | 60 m | |
| B7 | Evidence + handback | 45 m | never skipped |

**Ordering is load-bearing twice.** L1/L2 before L3: an early lens whose dump is
a farewell gift and whose rates are lifetime averages would lose the very
evidence the move exists to capture, and would lie in the footer while doing it.
B2 before B3: faster detection alone makes the orphan fresher.

## Log

| unit | outcome |
|---|---|
| V0 | branch `bringup-lens-2026-08-14`, this document |

## Handback

*(written at session end, or on early failure — whichever comes first)*
