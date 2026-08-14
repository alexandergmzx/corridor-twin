# ADR 0035: The lens is the first instrument, and it outlives the run

- **Status:** Accepted
- **Date:** 2026-08-14
- **Relates to:** the "watch the run, do not autopsy it" rule in `CLAUDE.md`,
  which this record makes structurally true rather than aspirational.

## Context

`CLAUDE.md` says: *"Always debug with the lens up, before reasoning about a run
from its artifacts."* It is called mandatory equipment, and the rule exists
because ignoring it once cost most of a day.

The runner did not honour it. Measured on 2026-08-14 across five Isaac runs:

- The lens was started **after** SLAM and Nav2 activated. **Ten of the twelve
  `rerun()` exits precede that point**, so a run that died in bring-up produced
  no `lens.log` at all — runs `20260814-022725`, `-023029` and `-025555` each
  died having written nothing. Bring-up was unwatchable by construction.
- The lens was killed **fifth** in `teardown()`, which runs before the map-quality
  and startup checks. Independently sampled serving windows: **127 s and 116 s**
  of ~6-minute sessions, `DOWN` for all of bring-up and everything after teardown.
- The banner printed `http://127.0.0.1:8765/` **unconditionally**, from a
  literal, after a health poll that broke on success *or* on the lens process
  dying — while the lens walks to ports 8766-8770 when 8765 is taken and exits
  if all six are busy. It could announce a dead lens, or somebody else's.

The operator's word for the result was "faux launches", and the word was
accurate: they were handed a URL for something that was not there.

**Nothing about the lens required any of this.** It is read-only by
construction — no publisher, service or action client exists anywhere in
`corridor_lens.py`. It subscribes to `/map` with matching latched
TRANSIENT_LOCAL durability, so it collects the map whenever SLAM eventually
publishes one. Every TF lookup is zero-timeout and `None`-tolerant, and every
payload in `build_state()` is optional. `/healthz` is served by the HTTP layer
independently of ROS. The position after Nav2 arrived in a single commit with
no rationale, no comment and no ADR.

The fleet had already reached the opposite conclusion. `simctl` treats a
pre-existing lens as normal and says why: *"slam_lens (a read-only viewer, left
open across sessions by design) subscribes to /sim/ground_truth — with the
topic test, a forgotten lens tab made every start refuse with 'already
running'"*. The corridor's `residents()` preflight had regressed that property
by listing `corridor_lens.py` alongside `nav2_*` and `slam_toolbox`.

## Decision

### 1. The lens starts before the simulator

It is launched immediately after the traps and the watchdog are armed, before
`simctl start`. It cannot precede the Isaac lock, the occupancy gates or the
sourced workspace, and it does not.

**Before rather than after `simctl start`, for one reason:** decision 3 makes a
missing lens refuse the run, and a refusal must not first throw away 70 s of
Isaac load. The two decisions are only coherent together.

### 2. It is not a `resident`

`corridor_lens.py` leaves the `residents()` pattern. That function's own stated
purpose is catching a node that *"offers the same recovery actions as this
run's and can command the robot"*; a process with zero publishers cannot. Under
decision 4 a lens from the previous run is **expected** to still be serving, so
refusing to start beside one would make every second run a rerun.

### 3. A lens that cannot serve refuses the run

Classified INFRASTRUCTURE, before the simulator is started, naming `--no-lens`
as the override. Instrumentation is a precondition, not a nicety: an unwatchable
run is the failure this record exists to remove, and at that point in the script
nothing has been spent, so refusing is free. `lens.log` is a diagnosis candidate
so the refusal quotes its own evidence.

### 4. It outlives the run, freezes, and the next run replaces it

The lens is not killed at teardown. It keeps serving the final map, the last
live frame and the full history to any browser that connects afterwards —
which is when an operator actually looks. It **freezes** once no message has
arrived on any topic for 60 s: still serving, but no longer appending samples
or rewriting its dump.

The freeze is not a nicety either. Without it a lens lingering until morning
would roll the entire run out of the history buffer and overwrite a good dump
with hours of nothing — the linger would be actively harmful. A lens that has
never seen a message is **not** frozen; it is waiting, which is the normal
state during bring-up and reads completely differently.

`reap_previous_lens()` replaces the old lens at the start of each run. An open
browser tab survives the handover, because the page auto-reconnects and the new
lens rebinds the same port.

### 5. The banner is a port the lens reported, then verified

The lens writes `{url, host, port, pid, domain, ...}` to an announcement file
**inside the server context, after the bind has succeeded**, so the file's
existence is evidence the port is being served rather than an intention to
serve it. The runner reads that file back and prints only after `/healthz`
answers **on that port**. No literal port number remains in the runner, and a
test asserts none returns.

A port scan over 8765-8770 was rejected: `lens_stub.py` defaults to 8766 and
also serves `/healthz`, so a scan would cheerfully announce a stub as this
run's instrument. **A health check is not an identity check.**

## Alternatives rejected

**A persistent lens daemon, decoupled from runs.** It needs a control channel
to reset `t0`, the history and the dump path per run, or it mixes runs on one
canvas and writes one dump covering many — exactly the class of mixing the
per-run directory exists to make structurally impossible. It also has no owner,
so a run could silently proceed unwatched, which is the bug.

**Adopting an existing lens instead of replacing it.** Same objection, sharper:
the adopted lens's `--dump` points at the *previous* run's directory, and its
canvas shows the previous run's map under this run's banner, unlabelled.

**Merely moving the kill to the end of teardown.** Necessary but insufficient —
it still dies before the operator looks.

## Consequences

The cost is an idle rclpy participant on the scratch domain between runs,
holding one DDS shared-memory segment and one port, plus a page that must say
`frozen` correctly or it looks stale. Nothing reaps the last lens of a session
except the next run or the operator. `simctl stop` never touched it —
`corridor_lens.py` is not in `SIM_PROCS` — and the shared-memory sweep only
unlinks segments no live process maps, so a lingering lens is safe from both.

Two implementation defects had to be fixed first, and are recorded in their own
commits rather than here: the dump was written only on a graceful unwind (lost
on 3 of 5 runs), and rates were cumulative averages since process start, which
an early start would have turned into a permanent understatement in the one
window where that footer is the only live signal.

**Not established:** whether starting the lens before `simctl start` measurably
costs anything. Its landmark detector runs at 5 Hz from the first scan and is
not behind the pose guard, so this adds a small consumer to the window already
blamed for lifecycle contention. To be measured against `contract.txt` rates
and the `simctl start → nav stack` interval. If it costs anything, the block
moves below `simctl start` — all three of the measured bring-up deaths are
after that point too — and nothing else in this record changes.
