# A quarter of runs never reached the robot

**Status: OPEN. The mitigation is a correlation, not a diagnosis.** For a demo
that has to work once, in front of someone, this is a larger risk than any
remaining defect in the delivery itself.

## The number

Twenty-seven corridor run attempts on 2026-08-13, robot1, domain 67:

| | |
|---|---|
| scoreable | 19 |
| excluded as rerun / crash | **7** |
| excluded because docking never ran | 1 |

Seven of twenty-seven — **26%** — died before the robot could answer the
question. Two more classes were found while counting them, and both had been
invisible:

* Runs the runner classified as `rerun` or `crash` were being **dropped from
  the summary entirely** rather than counted as set aside, because the filter
  tested for a `classification` value the runner never writes. Fixed; the fix
  is what revealed the 26%.
* One run drove the whole transit and delivered, but its docking never ran at
  all — the goal-acceptance response was lost while the robot drove 10.709 m
  anyway, so the gate returned before its dock loop. The transit is a result;
  the docking is not, and scoring it as a docking failure understates every
  docking change measured against it.

## One cause, three presentations

The failures first looked like two modes. They are not.

    [lifecycle_manager_nav] Configuring controller_server
    [lifecycle_manager_nav] ERROR: Failed to change state for node:
        controller_server. Exception:
        controller_server/get_state service client: async_send_request failed.
    [lifecycle_manager_nav] Failed to bring up all requested nodes. Aborting bringup.

`controller_server` is where `local_costmap` is configured, and it is the point
every failure passes through. What varies is only how it fails:

| presentation | what the log shows | cost |
|---|---|---|
| fast abort | `get_state` fails 2.2 s in, manager aborts the whole stack | ~3 s, detected immediately |
| slow configure | completes, but late | 85–89 s |
| hang | configures and never returns | the full deadline |

`bt_navigator` is later in the activation order, so an abort at
`controller_server` leaves it `unconfigured` — which is what the runner then
reports, one step downstream of the actual fault.

**The bring-up time distribution is bimodal with nothing in between**, measured
across 17 bring-ups:

    fast : 9 10 10 11 12 13 13 14 15 16 18 s   (11 of 17)
    slow : 85 86 87 88 88 89 s                 ( 6 of 17)

A spread would indicate load. Two clusters and an empty middle indicate a
discrete stall — a timeout-and-retry inside the lifecycle sequence. **That
mechanism is not identified**, and finding it is the real fix.

### What was done about it

`LIFECYCLE_DEADLINE_S` was 75 s, which is the single worst value available: far
above every fast bring-up so it never saves time, and just below every slow one
so a slow bring-up is *guaranteed* to burn a full attempt and retry. Raised to
110 s, clearing the measured slow mode by 21 s. It cannot slow a fast run down —
the loop exits on the bt_navigator bond, not on the deadline.

This treats a symptom. It is worth doing because the deadline was actively
placed in the one interval where it converts a slow bring-up into a dead run,
and it is not worth mistaking for a fix.

## The shared-memory correlation, stated as a correlation

Isaac leaks one POSIX semaphore per session and never reclaims it:

    /dev/shm/sem.carb-RStringInternals-<pid>

After ~30 sessions there were **144**, every one owned by a dead process, with
`/dev/shm` holding 304 entries. `carb` is Omniverse's Carbonite core.

The failure rate had been worsening across the day, and then:

| | bring-up failures | `async_send_request` races |
|---|---|---|
| the four runs before the sweep | **4 of 4** | 3+ |
| the three runs after it | **0 of 3** | 0 |

Fisher's exact on 4-of-4 against 0-of-3 gives **p ≈ 0.03**.

**This is not a demonstrated cause.** One thing was changed, on a host that had
been cycling Isaac for nine hours, and a transient that cleared on its own is
not excluded. Space was never the constraint — `/dev/shm` was 2% full — so if
the semaphores are implicated it is through some mechanism not yet named, and
"stale segments confuse DDS discovery" is a story rather than a measurement.

The reaping is now part of teardown regardless, on its own merits: an
unattended session accumulates these without bound and nothing else removes
them. It touches only entries whose owning pid is dead — a live one may belong
to a concurrent Isaac — and it fails silently, because a tidy-up that can abort
a teardown is worse than the litter.

## What this means for the demo

- **Budget for reruns.** At the observed rate, roughly one run in four needs
  repeating, and a failed bring-up costs about four minutes.
- **The failure is loud and early**, which is the one good thing about it. It
  happens during bring-up, before the robot moves, and the runner classifies it
  explicitly rather than producing a bad delivery number.
- **The recorded fallback matters more than it did.** A live demo that depends
  on a bring-up with a 26% failure rate needs the recording available.

## Open questions, in the order worth answering

1. What stalls inside `controller_server`'s configure? It brings up
   `local_costmap`, which waits on transforms and sensor data; the runner
   already waits for the TF chain before launching nav, so if a transform is
   still missing at that point, which one?
2. Is the bimodality a fixed timeout? 85–89 s is suspiciously tight for a
   phenomenon that would otherwise vary with load.
3. Does the `/dev/shm` correlation survive a deliberate replication — let the
   semaphores accumulate again and see whether the failures return?

**None of these was attempted.** Nav2 parameters are fenced for this session,
and (1) and (2) likely live there.
