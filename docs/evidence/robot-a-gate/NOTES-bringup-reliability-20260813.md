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

## One cause, four presentations

The failures first looked like two modes, then three. They are one.

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
| `inactive [2]` | configures, but `follow_path` is not serving when bt_navigator activates | the full deadline |

`bt_navigator` is later in the activation order, so an abort at
`controller_server` leaves it `unconfigured` — which is what the runner then
reports, one step downstream of the actual fault.

**The bring-up time distribution is bimodal with nothing in between**, measured
across 17 bring-ups:

    fast : 9 10 10 11 12 13 13 14 15 16 18 s   (11 of 17)
    slow : 85 86 87 88 88 89 s                 ( 6 of 17)

A spread would indicate load. Two clusters and an empty middle indicate a
discrete stall — a timeout-and-retry inside the lifecycle sequence.

### The chain, later read out of the launch logs

A `wide_corner` run at 19:12 printed the whole thing:

    attempt 1: Configuring controller_server
               local_costmap.local_costmap: Configuring        <- and hangs
    attempt 2: Activating bt_navigator
               ERROR "follow_path" action server not available
                     after waiting for 1.00s
               ERROR Exception when loading BT:
                     Action server follow_path not available
               ERROR Failed to change state for node: bt_navigator
               ERROR Failed to bring up all requested nodes. Aborting bringup.

`follow_path` **is** `controller_server`'s action server. So:

    local_costmap is slow to configure
      -> controller_server comes up late
      -> bt_navigator activates and waits 1.00 s for follow_path
      -> the wait expires and the behaviour tree fails to load
      -> the lifecycle manager aborts the WHOLE stack
      -> the runner reports "bt_navigator never reached ACTIVE",
         three steps downstream of the fault

That is the single race behind all four presentations — abort at 2.2 s, slow
configure at 85-89 s, hang, and `inactive [2]`. What differs is only where the
clock runs out.

**Both timeouts in the chain are Nav2 parameters** — the costmap's
configuration work and bt_navigator's 1 s service wait — and Nav2 parameters
are fenced for this session, so neither was touched.

### The contention was self-inflicted

`corridor_profile_run.sh` pauses between SLAM and Nav2 precisely for this. The
pause was **8 s**, and `86e5a01 perf(run): stop paying for time the run does not
need` halved it to 4 s to make runs shorter. The comment written with that
change named the risk exactly:

> *"Halved, not removed. The contention it guards against is real and it is
> what the nav bringup aborts on."*

It is. Restored to 8 s in `b9a844a`.

**This is not a controlled A/B and must not be read as one.** Every run
measured here was already at 4 s, so there is no clean before, and the reaper
above landed in the same window. The restoration stands on the trade rather
than on a proof: four seconds saved per run against a quarter of runs dying at
about 250 s each, which is a bad bargain at any plausible failure rate.

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

## The host degrades over a long session

The settle restoration was tested in a batch of the same shape as the one
before it, and the test **failed to answer the question** -- which is itself the
finding.

Both `nominal` runs were non-starts, in two ways neither seen earlier that day
and neither matching the bring-up race:

| run | what happened |
|---|---|
| 195321 | bring-up clean; A drove **0.128 m** and stopped. No landmark detections at all. Truth and odometry agree it did not move |
| 195837 | `no map-frame pose via TF: "map" ... does not exist`. slam_toolbox alive but dropping EVERY scan -- `Message Filter dropping message: frame 'laser_frame' ... queue is full` -- so no map was ever built |

Neither has an obvious link to a 4 s pause between SLAM and Nav2.

The day's shape is the point. Runs this morning were largely clean; through the
evening the failures grew both more frequent and **more varied** -- the
bring-up race, then a dead evaluator subscription, then a robot that will not
move, then a SLAM that will not consume its own scans. That is the signature of
accumulating state, not of a code defect, on a host that had cycled Isaac about
forty times.

**The batch was stopped rather than completed.** On a degrading host, further
runs produce evidence that cannot be interpreted in either direction: a clean
batch would not have validated the settle and a failed one would not have
condemned it. Twenty more minutes to learn nothing is worse than stopping.

So `b9a844a` is **committed and unvalidated**, and the honest test is a fresh
session on a rested host, run before the day's Isaac cycles accumulate.

For the demonstration this is the operational rule that falls out of it:
**do not rehearse for hours and then present on the same boot.**

## What this means for the demo

- **Budget for reruns.** At the observed rate, roughly one run in four needs
  repeating, and a failed bring-up costs about four minutes.
- **The failure is loud and early**, which is the one good thing about it. It
  happens during bring-up, before the robot moves, and the runner classifies it
  explicitly rather than producing a bad delivery number.
- **The recorded fallback matters more than it did.** A live demo that depends
  on a bring-up with a 26% failure rate needs the recording available.

## Open questions, in the order worth answering

1. **Does the restored 8 s settle actually reduce the rate?** Measured after
   the fact, in a batch of the same shape as the one before it. Six runs is a
   weak instrument for a 26% rate — 0 or 1 failures would be consistent with no
   effect at all — so read the count, not a verdict.
2. **Why is `local_costmap` slow to configure?** It waits on transforms and
   sensor data; the runner already waits for the `map -> base_footprint` chain
   before launching nav, so if something is still missing at that moment, what?
3. **Is bt_navigator's 1.00 s `follow_path` wait the right number?** It is the
   proximate trigger and it is a Nav2 parameter.
4. Is the bimodality a fixed timeout? 85–89 s is suspiciously tight for
   something that would otherwise vary with load.
5. Does the `/dev/shm` correlation survive deliberate replication — let the
   semaphores accumulate again and see whether the failures return?

**Only (1) was attempted.** (2), (3) and (4) live behind the Nav2 parameter
fence this session set, and (5) costs a day of runs to answer honestly.
