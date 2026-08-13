# Session plan — delivery closes by relative goal; arrival semantics re-pinned

**Branch** `delivery-close-2026-08-13` (from `6963a1c`)
**Started** 2026-08-13 ~10:00 CST · **Budget** 6 h · **Stop starting new units** 15:30 CST
**Governing input** the overshoot diagnosis,
[`docs/evidence/robot-a-gate/NOTES-why-A-overshoots-B-20260813.md`](../evidence/robot-a-gate/NOTES-why-A-overshoots-B-20260813.md)
(ten bags), and ratified decisions D1–D9.

Unattended-grade rules apply: append-only history, green-checkpoint commits,
Isaac single-occupancy via `/tmp/fleet-isaac.lock`, park-don't-decide, lens up
on every live run. **Exception granted and exercised:** D8 authorised one
`--force-with-lease` push on `gate-green-2026-08-12` (see the log below).

---

## Why this session exists

A is not failing to navigate. It reaches the delivery standoff to **3–12 cm on
every run in the population**, and then keeps driving, because SLAM's pose is
0.8–2.2 m behind the truth and the goal checker therefore says it is still
short. Thirteen of 22 `nominal` runs end against the street's far wall.

The root measurement: over the first 2 m of travel the **EKF registers
0.94–1.05 of truth and the scan matcher registers 0.13–0.72**. SLAM then
corrects good odometry toward its own bad estimate. The error is longitudinal —
7–97× the across-corridor error — and half of it is already present within two
metres of the spawn.

So the fix is not a better map. It is to **stop gating the delivery on a
map-frame number at all**, and to close the last stretch from the laser
measurement, which is the one quantity immune to that error.

---

## Naming

The brief's handback line said "the <name> scoreboard". That is precisely the
name D8 exists to remove, so throughout this session the artefact is **"the
scoreboard against the 2026-08-04 interview corrections"**. Flagged rather than
silently substituted.

---

## Unit queue and status

| Unit | What | Status |
|---|---|---|
| **D8** | Privacy: reword, redact every branch, delete the pack file | **DONE** — see log |
| **W0** | This plan | **DONE** |
| **W1** | Goal yaw from the manifest + regression test + **NOTES correction** | **DONE** — `896d304` |
| **W2** | Docking preemption: D1 arming, map-frame proximity deleted | **DONE, RED** — `a29dc19` |
| **W3** | ADR 0033, arrival semantics (must resolve the ADR 0029:129 conflict) | **BLOCKED on the decoy decision** |
| **W4** | Acceptance runs, three profiles, dock ON | **BLOCKED on W3** |
| **W5** | Matcher A/B — **only if W4 is banked**, 90 min hard cap | pending |
| **W6** | One dedicated camera session (concurrent is impossible — see below) | pending |
| **W7** | Speed estimator v0, if time | pending |

Skip-edges: W5 is dropped entirely if W4 is not banked by 14:00. W7 is dropped
if W6 has not produced footage by 15:00. W6 is dropped if the Isaac lock is
held.

---

## Four deviations from the brief, decided in advance

### 1. W1's premise is wrong, and the error is mine

The brief says the goal "has never carried a reachable yaw (`w=1.0` = facing
back up the corridor)". That sentence is quoted from my own NOTES, and the
NOTES are wrong. Measured:

| | world yaw |
|---|---|
| map +x axis (A's spawn heading) | **+7.13°** |
| the goal as sent today, `w = 1.0` | **+7.13°** |
| bearing from the standoff to B | **0.00°** |

The goal already points at B, 7.1° off. And the proposed fix moves almost
nothing — against measured arrival yaws of −51.4° to −78.6°:

| goal yaw | yaw error | vs 34.4° tolerance |
|---|---|---|
| current, +7.13° | 58.5 – 85.7° | FAILS |
| W1 proposal, 0.00° | 51.4 – 78.6° | **STILL FAILS** |

The real reason the yaw check fails is that **A arrives mid-turn** — still
heading south, 51–79° from any sensible goal heading — and would have to rotate
in place to finish, which a 1.3 m position error never lets it reach.

W1 is kept and **demoted**: deriving the goal yaw from the manifest instead of
leaving an accidental `w=1.0` is right, cheap and testable, but it is a
correctness fix, not a blocker fix, and its commit must not claim otherwise.
**W2 is what closes the delivery.**

### 2. W2's "immune by construction" holds for position, not for yaw

```
goal − believed_pose = R(believed_yaw)·detection_body
                       − standoff · unit(R(believed_yaw)·detection_body)
```

Believed **position** appears on both sides and drops out — the relative goal is
immune to translational map error by construction, as the brief claims.
Believed **yaw does not cancel**; it rotates the offset. Measured map-yaw error
is 2–9°, which at a 0.6 m offset is **≤ 0.09 m of residual**. Bounded and small,
and it goes in the ADR as a stated residual, not as claimed immunity.

### 3. W6's "no extra Isaac" is not available

A corridor run brings Isaac up through the fleet's `simctl start --backend
isaac`, which creates **no camera and no render product**; the runner's
occupancy guard (`corridor_profile_run.sh:323`) greps for
`isaac_5_1_ros_camera` and *refuses to start* if it is running — the two
bringups are mutually exclusive by design. Attaching a second process is not an
alternative (`SimulationApp` is a single in-process Kit runtime; a second
process is definitionally a second Isaac, which the lock forbids). Bridging
from domain 67 would be semantically wrong, not merely unconfigured: 67 carries
`sim/ground_truth`, the exact truth class the certificate exists to exclude, and
no `/clock` at all.

Worth recording because it will tempt someone: **the mast camera prim is
already loaded in the corridor session** — `/World/Actors/PCameraMast/PCam` is
present and valid in `arena_corridor_robot1_nominal_m6_n3.usd`. The gap is ~30
lines of OmniGraph. But those lines live in `sim_runner.py`, a **fleet file**,
which this session's rules forbid.

So W6 becomes **one dedicated ~8 min camera session** after W4, which is what
`crossing_session.sh` already does and what ran green on P's mast last night.
Cost, said rather than buried: the footage is of A **driven along the authored
route**, not of a Nav2 delivery. For a speed estimator that is arguably better
— constant, known speed — but W7's numbers must be labelled as such.

### 4. W5's first A/B already exists and costs one flag

**Every run in the diagnosis used the FLEET canonical `slam_toolbox.yaml`, not
the corridor config.** `corridor_profile_run.sh:59` defaults to the fleet file;
`--corridor-slam` opts into `config/robot1/slam_robot1_corridor.yaml`. I never
passed it. So all ten bags ran with `do_loop_closing: true`.

The corridor config differs by **exactly one line** — `do_loop_closing: false`
— and exists because a 2.83 m single-step `map→odom` jump was attributed to a
false closure. My diagnosis measured two early jumps of **+0.41 m and +0.32 m**;
the first is outside the ±0.3 m bound that `correlation_search_space_dimension`
puts on one correlative match. In a corridor of two long parallel walls,
traversed once, a false loop closure is the textbook failure — and every
accepted closure on a single-pass route is necessarily false.

Experiment 1 is therefore `--corridor-slam` versus default, measured with
`tools/diagnostics/travel_registered.py`. No parameter edited, no fleet file
touched, the alternative config already on disk.

---

## Fenced — do not touch this session

- The **odometry-trust penalty block** in the SLAM configs. The corridor file's
  own header records that the fleet's near-wall study falsified penalty tuning
  at verdict level on three bags: *"do NOT ship a slam_toolbox_nearwall.yaml"*.
- Any **fleet file**. This repo is a scenario member, not a co-owner of fleet
  tooling.
- Nav2 / slam params outside W5's controlled A/B scope.

---

## Verification each unit owes

- **W1** — regression test asserting the goal yaw equals the route's final
  heading and forbidding the identity quaternion; the corrected NOTES lines.
- **W2** — dock tests green with the two rewritten; `test_the_spawn_phantom_is_refused`
  and `test_arming_does_NOT_depend_on_the_map_pose` still passing **by name**;
  then a docked run whose `rejections` carry no map-frame reason and whose state
  reaches at least ACQUIRE. If the replacement set is *also* insufficient, that
  is a result and gets written down.
- **W3** — index row **and** mermaid decision-map node in the same commit, or
  `test_repository_contract.py:96` fails.
- **W4** — three consecutive `nominal` greens under ADR 0033, then the other two
  profiles. Artifacts session-scoped. Reds are committed findings, in bold.
- **W5** — `travel_registered.py` before and after each A/B, both recorded in
  `degeneracy-study.md` regardless of outcome. Revert by default: keep a change
  only if the ratio reaches ≥ 0.9 on `nominal` without degrading `wide_corner`.
- Every commit — `bash tools/check_workspace.sh` green, counts pasted from the
  run that produced them, never from memory. (Written-down counts have gone
  stale three times; I published a false one this week.)

---

## Corrections I owe, carried in

1. **NOTES lines 155–160** say the goal faces "back up the corridor" and that
   the delivery goal "has never carried a reachable orientation". Both wrong —
   see deviation 1. Corrected additively in W1's commit.
2. That is the **fourth** defect found in that document (bag count, missing run,
   population counts, now this). Every one was in a framing claim, not in a
   measurement; the mechanism and the 13–72% registration figure have survived
   every check. Said out loud here so the handback does not have to discover it.

---

## Log

### D8 — privacy. Unit zero, because PREFLIGHT failed

PREFLIGHT found the name still present at the tips of `main`, `origin/main`,
`gate-green`, `origin/gate-green`, six further remote branches, two local-only
branches, one untracked file, and one commit subject.

| # | Site | Disposition |
|---|---|---|
| 1 | `7ee4b29` subject | reworded → `55f7b6d`, tree identical; `gate-green` → `9ef1083`, `phase3-launch` + `delivery-close` → `23b5dfb` |
| 2 | `main`: `docs/evidence/source-diagram/NOTES.md` | redacted `ae5cbc1`, pushed |
| 3 | `gate-green`: that file + `docs/v2-plan.md` | redacted `569a76c`, force-pushed **with lease** |
| 4 | six merged remote branches | `3c7c985 f773ca0 3cd1f27 9bff54b fb13275 0aa36b3`, all pushed |
| 5 | two local-only branches | `2878adf`, `dc17cc7`, not pushed |
| 6 | `corridor-v2-adr-pack.md` (untracked, D7) | deleted; unreachable blob `6a3a99cf` survives in `.git` until `gc --prune`, local only |
| 7 | `docs/ROBO_TASK.pdf` metadata | **deliberately left** — operator's decision |
| 8 | pre-rewrite objects in history | left; unreachable, local |

Post-sweep across all 20 refs: zero tracked-text hits, zero commit-message hits
(`git log --all -i --grep`), zero `-S` hits. The only remaining route to the
name is `pdfinfo docs/ROBO_TASK.pdf` on `origin/main`.

**Two mistakes of mine inside D8, recorded because they were nearly costly:**

1. My plan scoped the redaction to two branches when the ratified decision said
   *all* branches. Six public branches would have kept the name in tracked text
   while the report said "done". Caught by the post-sweep, fixed before
   reporting.
2. My first attempt used `git filter-branch --msg-filter 'sed …'`, which
   **rewrote all 12 branches including `main`** — `sed` normalises every message
   it touches, so every commit object changes. Restored all 12 from a
   pre-recorded `refs-before-D8.txt`, verified byte-exact, then redid it with
   `commit --amend` + `rebase --onto` so only the 3 branches actually containing
   the commit moved. Trees identical throughout. A third slip inside the repair
   — a regex that spliced a new 7-char prefix onto an old SHA tail — was caught
   by requiring every referenced commit to resolve.

The force-push on `gate-green-2026-08-12` (`865f1d7...569a76c`, explicit lease)
is **the dated append-only exception** this session was authorised to take.
`main` was fast-forward only (`f48c515..ae5cbc1`). Gates before both pushes:
ruff clean, pytest 438 passed / 1 skipped, colcon 141 tests 0 failures.

### W0 — this plan

Written before implementation, per the long-session rule. Binding once written;
re-read first after any context compaction; the handback is its final section.

### W1 — goal orientation (`896d304`)

Done, and with a **third** correction beyond the two the plan anticipated: the
brief's prescribed fix — derive the goal yaw from the authored route's final
heading — would have introduced a worse defect than the one it repaired.
`delivery_standoff_world`'s own docstring says why: reading the authored route
is exactly the "authored line and waypoints" ADR 0022:15-17 keeps out of A's
navigation. The two derivations agree numerically because the standoff sits on
B's approach ray, and that agreement is the trap. Derived from B's bearing
instead; a test deletes `delivery_trajectory` from the manifest and requires
the facing to survive.

NOTES lines 155–160 corrected, with the replaced text quoted in the correction
block so the error stays legible rather than vanishing.

### W2 — docking preemption (`a29dc19`). **RED, and the red is the result**

The map-frame deadlock is gone: `armed()` reads no robot pose at all, and
replayed over seven recorded bags arming now fires on every one, where before
it never fired at all on the docked run (2812 map-frame rejections).

**But on four of seven it arms on the wrong object**, and all four agree to
within 0.11 m. The object is authored geometry: the free west end cap of
`EastWallStub`, 0.318 m wide, centred at (4.565, −1.926), 0.59–0.77 m from B.
It passes shape, chord and isolation *honestly* — from the approach direction
it genuinely has open space on both sides — and it sits between A and B, so A
resolves it first.

Two fixes were measured and both rejected:

| candidate fix | result |
|---|---|
| rank candidates by radius error, not fit residual | indistinguishable, marginally worse |
| require arming frames to agree on a position | **worse: 6 of 7 instead of 4 of 7** |

The second is the informative one. It reads as strictly better — "k scans that
agree they see the same object" rather than "k scans that each saw something" —
and it fails because **the failure is ordering, not agreement**: consecutive
agreement hands the decision to whichever object accumulates a run first, and
that is the decoy. Reverted by default per gate discipline; the weaker rule
ships. The full measurement is
[`NOTES-the-eastwallstub-decoy-20260813.md`](../evidence/robot-a-gate/NOTES-the-eastwallstub-decoy-20260813.md).

Useful number underneath it all: **per frame, the detector picks the real B
88–97% of the time and the stub 2–12%.** Accuracy is not the problem. Arming
being a first-past-the-post decision is.

### Why W3 and W4 are blocked rather than attempted

ADR 0033's proposed dock-on acceptance is "world-frame closest approach
≤ 0.15 m **and** `DELIVERED`". With the decoy unresolved that criterion is red
on four runs in seven for a reason that has nothing to do with arrival
semantics — so pinning it now would either bake in a gate the demo cannot pass
or invite it to be quietly loosened later. ADRs are immutable once accepted;
this one waits for the decision below.

W4 follows W3.

## Morning decisions

1. **The decoy: scene or arming?** Either `EastWallStub` loses its free west
   end — arguably a modelling artefact rather than an intended feature, since
   nothing in the scenario calls for a wall that stops in mid-air — or arming
   stops being first-past-the-post and accumulates evidence across the whole
   approach, where B's 9:1 per-frame advantage would decide it. The first is
   minutes and changes the scene; the second is hours and changes the method.
   **This is the decision that unblocks W3 and W4.**
2. **W6's footage is not of a delivery** (deviation 3). If footage of a real
   Nav2 delivery is the actual requirement, that needs the fleet edit and a
   different session.

---

## Handback

*(written at session end, or on early failure — whichever comes first)*
