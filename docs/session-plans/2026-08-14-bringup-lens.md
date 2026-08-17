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
| V0 | Branch + this document | 15 m | **DONE** `d07ba85` |
| L1 | Dump written as measured, not as a farewell | 30 m | **DONE** `bece230` |
| L2 | Windowed rates + history covering a session | 45 m | **DONE** `0718218` |
| L3 | Lens before the sim; banner = verified port; absent lens refuses | 60 m | **DONE** `b2c0b7a` |
| L4 | Lens outlives the run + freeze + ADR 0035 | 60 m | **DONE** `d267852` |
| B1 | Phase labels (SLAM, map save, preconditions) | 20 m | **DONE** `8657118` |
| B2 | Failed attempts reaped as a process group | 60 m | **DONE** `fdffafd` |
| B3 | SLAM poll reads the log (oracle replay over **85** logs) | 45 m | **DONE** `454d07d` |
| B4 | A run that never hands off says so | 45 m | **DONE** `bd254c8` |
| B5 | Map score as a `run.json` covariate | 20 m | **DONE** `3c054b2` |
| B6 | Nav2 SUCCESS as a second handoff trigger + ADR 0036 | 60 m | **DONE** `76a0fac` |
| B7 | Evidence + handback | 45 m | **DONE** |

**Ordering is load-bearing twice.** L1/L2 before L3: an early lens whose dump is
a farewell gift and whose rates are lifetime averages would lose the very
evidence the move exists to capture, and would lie in the footer while doing it.
B2 before B3: faster detection alone makes the orphan fresher.

## Log

| unit | outcome |
|---|---|
| V0 | branch `bringup-lens-2026-08-14`, this document |
| L1-L4 | the lens half. Contract tests were the binding constraint throughout: the bidirectional state-key match forced `frozen` into `lens_stub` in the same commit, and the constant-parsing tests forced `HISTORY_LEN` to stay a bare literal |
| B1-B6 | the bring-up half. The SLAM oracle was re-measured before being trusted: the plan said 82 archived logs, the box now holds **85**, and the separation held — 81/81 ready, 3/3 failed, 0 ambiguous |
| B7 | validation run `20260814-051901` and evidence |

## Handback

### What was delivered

**Eleven commits, each gated green before committing.** Final gate: ruff clean,
**pytest 544 passed / 1 skipped** (from 479 at session start — 65 new tests),
colcon **142 tests, 0 failures**. Two ADRs: **0035** (the lens is the first
instrument) and **0036** (the handoff has two triggers).

**The faux launch is now structurally impossible.** There is no literal port
left in the runner; the banner interpolates only what the lens itself wrote
*after binding*, re-verified against `/healthz` on that port; and a lens that
cannot serve refuses the run before the GPU is spent.

### Validated live, on a run that failed in bring-up

Run `20260814-051901` ended as a `rerun` — `bt_navigator` never activated —
which is exactly the failure class that used to be invisible. It was watched
end to end:

| | |
|---|---|
| Lens announced | **+0 s**, simulator at +1 s (was +111 to +126 s) |
| First history sample | **t = 0.04 s** |
| `lens.json` | **1700 rows / 343.7 s**, and it exists *although the lens was never stopped* |
| 77 s after teardown | still serving, `frozen: True`, rates 0.0, final map and 1736 history rows to a new browser |
| Phase recorded | `nav stack` — the phase it actually died in |
| SLAM oracle | `active (attempt 1, from its own log)` in ~2 s |
| **Group reap** | `nav attempt 1: group survived TERM; escalating to KILL` → `group reaped (KILL)` |

**Under the old code that run would have produced no lens evidence at all.**

The reap line is the strongest single result: it confirms TERM genuinely does
not reach the children *and* that the escalation works — which is the mechanism
that made run `025555`'s second SLAM attempt spend 110 s talking to a corpse.

### Open, and honestly so

1. **The cost of the early lens is unmeasured.** One run is not a comparison,
   and this run's nav stack failed twice. That is the known intermittent and it
   has failed identically with a late lens, but **this run cannot distinguish
   the two.** Needs `contract.txt` rates and the `simctl start → nav stack`
   interval across several runs. Rollback if real: move the block below
   `simctl start`; nothing else in ADR 0035 changes.
2. **`reap_previous_lens` has not run live.** A lens is deliberately still
   serving on 8765 as this is written — that is the ADR 0035 contract, not an
   orphan. The next run will exercise the reap.
3. **ADR 0036's second trigger has never fired.** No run since it landed has
   reached the docking phase.
4. **Nav2 bring-up is now the dominant failure.** Two attempts failed in this
   run and it is untouched by this session — no Nav2 parameter was changed, by
   design. It is the obvious next target.

### Morning decisions

- **The contract precondition still never passes** (`scan` off its calibrated
  ~12.0 Hz on every run, always waved through). Unchanged from the previous
  handback; B5 now records the map score as a covariate so this class of
  question becomes answerable without spending runs.
- **A missing lens now refuses the run.** That promotes instrumentation to a
  precondition. It was your call this session; if it proves annoying in
  practice the fallback is a loud warning plus `manifest_error`, one branch.

### Not touched, deliberately

Nav2 and SLAM parameters (`xy_goal_tolerance`, `LIFECYCLE_DEADLINE_S`, the 8 s
settle, `slam_toolbox.yaml`, `MAX_DUPLICATE_WALL_M`, `CREEP_TIMEOUT_S`), scene
topology, `sim_runner.py`, and the fleet repo — **no fleet edits this session**,
so no grant was needed.

### Mistakes made and repaired in-session

- **L1's commit swept in L2's lens-side wiring** (`git add` of the whole file).
  History is append-only, so L2's body carries a `REPAIR NOTE` naming it.
- **A test flake I introduced**: a third decoy in the `residents()` test
  collided with `sleep 8` decoys from the previous pytest invocation — red
  in-suite, green alone. Fixed with a baseline wait and short-lived decoys,
  then run three times consecutively.
- **Two heredoc errors in B5**, both caught by tests rather than by reading:
  importing `run_manifest` where `argv[0]` is `-`, and a test lift that assumed
  the heredoc marker was followed by a newline when it is followed by `|| true`.
