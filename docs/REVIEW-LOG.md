# Review log

What independent review has already challenged, and how each finding was
settled. **Read this before raising a finding** — several entries below record
arguments that were made, tested, and resolved, including three where the fix
differs from what the audit prescribed and one where the prescription would have
broken the build.

Disagreeing with a disposition here is welcome. Re-deriving one from scratch is
the waste this file exists to prevent.

| Round | Subject | Commits |
|---|---|---|
| 1 | Documentation and decisions | `5a3a083`, `43db0fc`, `ab09aff` |
| 2 | Code | `b5194bf`, `55eb69e`, `dbcf57e` |
| 3 | Closing the deferred findings | `a101b28`, `82b490d` |
| 4 | Independent review of rounds 1–3 | `f2e2504`, `be4694f`, `1099245`, `64220a7`, `ade033e`, `7a5980a` |

---

## Disputed, or resolved differently from the prescription

These three cost the most to re-derive, so they are recorded in full.

### M1 — the estimate path's truth guard

**Raised as:** the truth guard is weaker on the estimate path than on the
display; an `Odometry` subscription added to `node.py` would pass. Prescribed
fix: union the two token sets.

**The example does not hold.** `src/police_observer/test/test_camera_pipeline.py`
already asserts `"Odometry" not in source` and `"ground_truth" not in source`
for `node.py`. The stated failure mode was already caught.

**The prescribed fix would have broken the build.**
`src/police_observer/police_observer/synthetic_node.py` legitimately contains
`ground_truth`: it *publishes* the evaluator topic `test/ground_truth/speed`.
That is the truth-isolation design working as ADR 0002 intends — the harness
needs a truth channel the observer cannot read. A token rule cannot distinguish
reading truth from publishing it, so unioning the lists would have failed a file
for doing the right thing.

**The finding's core was real, for different reasons.** The blocklist named only
the tokens someone had thought of, so `/tf` and `get_world_pose` passed freely;
and it covered `node.py` alone, leaving `estimator.py`, `synthetic.py` and
`synthetic_node.py` unchecked.

**Resolved** in `ab09aff` by enumerating the subscriptions each estimate-path
module constructs — both `create_subscription(TYPE, ...)` and
`message_filters.Subscriber(node, TYPE, ...)`, the two forms this package uses —
and requiring every message type to be `Image` or `CameraInfo`. This
distinguishes subscribing from publishing and fails on a truth subscription of
any type, including one nobody thought to name.

**Verified by mutation**: adding an `Odometry`, `TFMessage` or `PoseStamped`
subscription to `node.py` fails the new test. Only the first would have been
caught by the previous guard.

### C1 — the coverage flag

**Raised as:** `every_enforcement_gate_measured` is structurally always false,
because it compares the measured set against every authored gate including the
first — which arms the estimator and can never carry a speed. Correct. It read
`False` on every run of every profile while coverage was in fact complete, and
losing a gate could not have moved it.

**Resolved differently in the test.** The audit asked for an assertion that the
flag is true. On the module test fixture it legitimately reads **false**,
because that fixture truncates the window at `x = 5.0` and only gate 4.0 is
reachable — the detector working correctly. Asserting only "true" would have
repeated the original mistake in the opposite direction: a flag pinned to one
value is what caused this.

`55eb69e` emits `measurable_gates_m` alongside `enforcement_gates_m` and drives
the flag to **both** values for the right reasons — false on the truncated
window, true on the full one with all four gates measured.

### C3 — a policy gap killing the observer mid-run

**Raised as:** an uncovered width raises `ValueError` out of `_on_frame`, a
subscription callback. Correct and reproduced.

**Resolved by failing at construction, not by catching.** `MarkerMap` now checks
every width the corridor actually presents when it is built. Catching in
`_on_frame` would have kept a misconfigured observer running while silently
dropping frames, which is worse than refusing to start: the demonstration would
look alive and produce nothing.

---

## Round 1 — documentation and decisions

| ID | Sev | Finding | Disposition |
|---|---|---|---|
| H1 | High | `CLAUDE.md` blocked work already delivered | Fixed `43db0fc`. Gate removed, milestone table rewritten against evidence |
| H2 | High | `docs/README.md` contradicted its own capability matrix | Fixed `43db0fc`. Milestone lines and growth map corrected |
| H3 | High | `HANDOFF.md` claims table understated the project | Fixed `43db0fc`. Three rows were "Not implemented"/"False" for shipped, evidenced capability |
| M1 | Med | Truth guard weaker on the estimate path | **Disputed, see above.** Resolved `ab09aff` by a different mechanism |
| M2 | Med | Runtime subscription audit deferred to a slice that happened | **Closed.** Structural enumeration in `ab09aff`; a live `ros2 node info` capture is now recorded in [`evidence/live-demo/runtime-node-info.txt`](evidence/live-demo/runtime-node-info.txt) |
| M3 | Med | Measured VRAM never reached `ACTIVATION.md` | Fixed `43db0fc`. 3,411 MiB headless and 3,547 MiB GUI added |
| M4 | Med | Evidence index omitted the `live-demo` topic | Fixed `43db0fc`, plus a test so an unlisted topic fails the gate |
| M5 | Med | Policy change had no ADR | Fixed `5a3a083`. ADR 0015 and ADR 0016 |
| M6 | Med | The headline violation has no redundancy | **Accepted, documented, not fixed.** Owner chose documentation over a second policy move. See "Known open" |
| M7 | Med | R17 parked in a stash is not durable review state | **Closed.** R17 itself is fixed in `a101b28`; the validator is committed and the stash is gone |
| L1 | Low | Test counts wrong in three places | Fixed `43db0fc`. Counts then went stale twice more, which is why the handoff now cites the command instead |
| L2 | Low | `DESIGN.md` carried four superseded claims | Fixed `43db0fc` |
| L3 | Low | README documented three unmeasured demo options | Fixed `43db0fc`. Verified by computation: at 1.0 m/s only `nominal_m6_n3` can violate |
| L4 | Low | Motion is constant-speed only | **Open.** No `PathSpeedProfile`; piecewise acceleration, dropped-frame coverage and latency are unreported |
| L5 | Low | ADR 0014 edited after acceptance | Recorded, not charged. The Decision is unchanged and the correction is marked |

## Round 2 — code

| ID | Sev | Finding | Disposition |
|---|---|---|---|
| C1 | High | Coverage indicator structurally always false | **Resolved differently, see above.** `55eb69e` |
| C2 | High | Speed policy unvalidated and order-dependent | Fixed `b5194bf`. Rules normalized by sorting; empty sets, duplicate thresholds and non-positive values rejected on both sides |
| C3 | High | A policy gap kills the observer mid-run | **Resolved differently, see above.** `b5194bf` |
| C4 | Low | Synthetic report cannot detect a wrong `path_axis_fraction` | **Closed by `82b490d`.** The schedule now advances along route arc length and reads world X off the authored trajectory, so the conversion no longer cancels |
| C5 | Low | Debounce satisfiable by one observation pair | **Open, documented.** Needs >2 m between accepted observations, roughly 30 dropped frames at 1.0 m/s. Unreachable today |
| C6 | Low | Evidence tool could write build output into `docs/` | Fixed `dbcf57e`. The build gets a temporary scratch directory |
| C7 | Low | Inconsistent schema tolerance for `role` | Fixed `55eb69e`. Reporter now uses `.get("role", "gate")`, matching `MarkerMap` |
| C8 | Low | Display callback one refactor from throwing | Fixed `dbcf57e`. Drops the marker and logs instead of ending the display |

### What round 2 could not break

Recorded because it is evidence, and because re-testing it has low expected
value: the occlusion solver's empty-interval sentinel is absorbing under every
subsequent clip; frustum exclusion requires the condition at both yaw extremes;
the uncertainty propagation `σ_speed = √(σ_from² + σ_cross²)/elapsed` is
analytically correct; `yaw_range`'s monotonicity assumption holds on all three
segments; and the render gate keeps pixel analysis separate from truth
comparison.

---

## Found while fixing, missed by both audits

Recorded so the pattern is visible rather than flattering.

- **`HANDOFF.md` had three more stale sections than round 1 reported** — section
  *headings* asserting "Gate: no motion yet", "Next slice…: coverage, not
  motion", and "Motion slice, blocked until both gates above pass". Marked
  superseded in `43db0fc`, with the genuinely open parts called out.
- **Its R2 row still read "deferred"** for the deterministic reporter that
  landed in `82417a8`.
- **The reviewer-instruction paragraph still said "Do not continue into motion
  even if this review passes"** — the H1 defect, in the one paragraph that
  directs reviewer behaviour. Missed while fixing three other instances of the
  same defect in the same file.
- **The handoff header went stale again inside the same session**, naming branch
  `main` and "everything below is pushed" while six commits sat unpushed on a
  branch. Second occurrence, caught by both audits. Now closed by a test rather
  than another manual correction.
- **The commits-under-review table was two cycles behind**, listing eleven
  commits ending at `3d9a754` and missing every demo commit and both audit
  rounds.

Two of these five are defects in the very work that was fixing the same class of
defect. Prose status that duplicates git state drifts faster than anyone
maintains it; the durable fixes have been tests and cited commands, not more
careful editing.

---

## Known open — please do not re-raise as new

Each is recorded deliberately, with its reason.

| Item | Why it is open |
|---|---|
| No canonical static qualification | The live run does not replace a paired dwell capture with its own mirror control. Next implementation slice |
| Pose-to-render latency uncharacterised | Never measured; no offset compensates for it. One camera period is 0.066 m at 1.0 m/s |
| R11 — runtime corridor-profile reload | The observer reads `corridor_profile` once at construction |
| M6 — the violation has no redundancy | Two gates in the strict zone, `consecutive_estimates: 2`. Owner accepted as documented risk; margin is 0.935 m/s against a 0.80 limit |
| Live coverage is one profile at one speed | `nominal_m6_n3` at 1.0 m/s. The other profiles cannot violate at that speed by design |
| C5, L4 | See the tables above |


---

## Round 3 — closing what round 1 and round 2 deferred

Two findings were deferred on the grounds that fixing them was risky or costly.
Re-measuring showed both grounds had dissolved.

### R17 — the half-occluded reference plate

Deferred because every occlusion-free placement measured at the time lost gate
8.0 at 0.6 m/s. **That was not a placement problem.** It was the continuity
guard treating noise-level backward station steps as pose jumps, at a speed
where the per-frame advance (0.0397 m) sits below the station noise (0.0414 m).
`e0bea0c` fixed the guard; re-measured against it, *every* occlusion-free
placement holds all four gates on all three profiles at all three speeds.

The plate moved to `along_m 0.75` at 0.60 m, which is **strictly better** than
the occluded original on every metric:

| | occluded original | relocated |
|---|---:|---:|
| Occluded accepted frames | 187 | **0** |
| Worst station error | 0.0845 m | **0.0817 m** |
| Worst gate speed error | 0.0559 m/s | **0.0530 m/s** |

The live demonstration was re-run on the corrected geometry and reproduces the
same narrative: compliant at gates 4 and 6, over at 8 and 10, exactly one
violation, 0.195 m/s exceedance, maximum speed error 0.0331 m/s.

**A consequence worth reviewing:** the new invariant bounds the supported
`(m, n)` envelope from above — `m/2 - n < 0.349`. `m = 8.0` with `n = 3.0` is
now refused, where it previously built. That build was only ever succeeding by
rendering a half-buried marker, but it is a real reduction in the range the
generator accepts, and it deserves a second opinion.

### C4 — the report's blind spot

The lesson generalises past this instance: a control that derives its command
from the same field the system under test divides by cannot measure that field.
The schedule now advances along route arc length, which is also how the live
Isaac run drives, so the synthetic control and the live run finally share a
definition of "travelled a metre".

---

## Round 4 — independent review of the first three rounds

Five findings, all confirmed, all fixed by the reviewer. Three were mine.

| ID | Sev | Finding | Disposition |
|---|---|---|---|
| F1 | High | Non-finite values split the two policy validators | Fixed `f2e2504`. `nan <= 0` and `inf <= 0` are both false, so the build side accepted what the observer side rejected |
| F2 | Med | `cdb6f79` re-recorded the evidence and left seven citations behind | Fixed `1099245`. Two camera figures also disagreed with their own `summary.json` |
| F3 | Med | The envelope justification was wrong and the narrowing avoidable | Justification corrected `64220a7`; **capability restored `7a5980a`**, see below |
| F4 | Med | Three routes past the subscription enumeration | Fixed `be4694f`. Keyword forms and `tf2_ros.TransformListener` all bypassed a positional-only walk |
| F5 | Low | Handoff counts went stale a third time | Fixed `ade033e`. Counts removed; the header now says to read them off your own run |

### F1 is the instructive one

`test_speed_policy_validation_agrees_across_packages` exists *specifically* to
catch a split between the two validators, and its six-case list had no
non-finite case. `.inf` is spellable directly in YAML, so the authored config
could produce a stage and a manifest with no complaint and then kill the
observer at construction — the exact failure shape `772d027` and `f992470` were
both about. A test written to catch a class of defect still only catches the
cases someone thought to enumerate.

### F3 — and the same mistake twice in one file

I wrote in ADR 0015 that `m = 8.0, n = 3.0` "leaves no visible east face for a
reference to sit on". Review measured it and that is simply not what the
geometry says: the band runs `m/2 - n` to `m/2`, so it is `n` tall wherever it
sits, and at `m = 8.0, n = 3.0` it is the same height as on the default profile
with the upper plate inside it. The bound was real; the reason given for it was
invented.

Review corrected the reason and deferred the capability fix, "because moving a
plate changes the measured accuracy figures". That ground was also not measured
before being written. All three configured profiles have entry width 6.0, so
clamping placement to the band floor does not bind on any of them and their
surveys come out identical. `7a5980a` restores `m = 8.0` through `m = 10.0` at
`n = 3.0`, and the new geometry was measured: every frame accepted, all four
gates at all three speeds, speed error 0.0123–0.0247 m/s.

## The deferral pattern

Three deferrals this session, three stated grounds that dissolved on contact:

| Deferred | Stated ground | What measurement showed |
|---|---|---|
| R17 | "every occlusion-free placement loses gate 8.0 at 0.6 m/s" | Not a placement problem. The continuity guard was treating noise as pose jumps; once fixed, every placement held |
| C4 | "closing it shifts the published figures" | It shifted them by 0.004 m/s, and the shift was an improvement |
| F3 | "moving a plate changes the measured accuracy figures" | It moved no profile that has figures |

The lesson is not that deferring is wrong. It is that a deferral's *stated
ground* is a claim like any other, and none of these three was measured before
being written down — including by the reviewer, who caught the pattern in my
work and then reproduced it once. A deferral that names a cost should either
cite the measurement or say plainly that the cost is estimated.

Worth applying to what is still deferred: the static requalification and the
pose-to-render latency both carry costs that have never been measured either.

---

## Round 5 — what could not see the new wall

Modelling the east-wall stub (ADR 0018) exposed a structural gap rather than a
one-off omission, found by the owner noticing RViz did not draw it.

**The manifest never published building footprints.** Per profile it carried
`occluders` -- the analytic proof's slab list -- and nothing else about walls, so
every manifest consumer was *structurally unable* to see a wall the proof does
not reference. `NorthBuilding` had been invisible the same way since it was
authored; nobody noticed because nothing drives past it.

| Where | Missed | Fixed |
|---|---|---|
| `viz_node.py` | Drew `occluders`, so the stub and the north wall were never rendered | `3bf0995`. Draws `walls`, emphasising the ones the certificate uses as witnesses |
| `tools/isaac_5_1_smoke.py` | Two hardcoded four-name tuples. Did not fail; silently stopped covering the stub | `3bf0995`. Derives from the manifest, with a contract test requiring it |
| `docs/DESIGN.md` prim tree | Listed four buildings | Corrected |
| `ACTIVATION.md`, `README.md`, `DESIGN.md` occlusion figures | **Stale since ADR 0017**, not 0018 | Corrected |
| `README.md`, `DESIGN.md` route prose | Still said "line-arc-line" | Corrected |

The proof itself was never wrong. The composed-mesh audit discovers prims from
the stage by collision schema and reported all five buildings throughout. What
was blind was the reporting and the display.

### The stale figures are the recurring pattern again

Three documents still quoted *78 certified pairs, 204 audit rays, 3.116 m
nearest, 76 `SouthBuilding` / 2 `CornerBuilding`, 50 constant-X / 28 constant-Y*.
Those describe P behind the corner mass — superseded by **ADR 0017**, a round
earlier, and they survived it. Measured now: **5 intervals, 404 rays, 5.366 m,
`EastBuilding` sole blocker, all constant-X.**

`test_live_run_headline_figures_match_the_recorded_summary` did not catch it
because it reads `live-demo/summary.json`, and these figures come from the
occlusion certificates. That is the fourth recurrence of prose drifting from a
measured artifact, and the third time the durable fix was a test that reads the
artifact rather than more careful editing. **Extending that test to the
certificates is the open follow-up**, and it is the only thing that would have
caught this.
