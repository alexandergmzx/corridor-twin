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
| 5 | Manifest consumers missed newly authored walls | `3bf0995` |
| 6 | Police placement and certificate integrity | Implemented; independent review found 5 further issues in the fix itself — see round 7 |
| 7 | Independent review of round 6's own fixes | `ab6c787`, `d115e0e`, `01527b7`, `b867536`, `e79b63c` — Implemented, **pending independent review** |

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

---

## Round 6 — police placement and certificate integrity

The owner identified that P is on the wrong side of the wall relative to the
source drawing. Independent review confirmed that the measured source puts P
inside/west of the east wall while the scene authors it outside/east. The same
review found that the visibility command can pass after the actual USD P is
moved into view because it continues aiming its rays at manifest bounds.

These and the related verifier-runtime, observer/UI, calibration, documentation
and validation findings are implemented on `audit/police-placement-2026-07-29`,
pending the independent review this round's own handback requests before any
GPU requalification. The evidence, required regressions, execution order and
handback contract are recorded in the
[active implementation handoff](HANDOFF-2026-07-29-POLICE-PLACEMENT-AUDIT.md).

| ID | Sev | Finding | Disposition |
|---|---|---|---|
| A6-H1 | High | P authored east of the east wall, opposite the measured source | Fixed. P moved to the wall's inner face; `test_p_stands_on_the_source_drawing_side_of_the_east_wall` derives the regression from the measured source pixels, independent of the placement code. [ADR 0019](adr/0019-relocate-p-inside-the-east-wall-with-a-corner-screen.md) `f28d321` |
| A6-H2 | High | `verify()` took P's bounds from the manifest and never checked them against the composed stage | Fixed `1f8a08f`. Reproduced first: the pre-fix verifier reported `passed=True` for a stage-only P translation into an open, camera-visible spot, confirmed by running that exact mutation against the old code. `stage_police_bounds()`/`stage_camera_facts()` bind the proof to the stage; `test_stage_only_police_substitution_is_rejected` and `test_manifest_only_police_substitution_is_rejected` cover both directions |
| A6-M1 | Med | An in-channel/visible P drove recursive certification into pathological subdivision | Fixed `6c638f1`. Measured before the fix: 40.7 s and 327,719 coverage entries for one profile's negative control. A total `call_budget` across the whole search (not just per-branch depth) bounds it; the same control now resolves in a fraction of a second. `test_a_genuinely_visible_placement_fails_promptly` pins the timing budget |
| A6-M2 | Med | RViz cleared a violation on raw speed while the detector rearmed on the conservative speed | Fixed `d99c18d`. `conservative_speed_mps()`/`is_conservatively_compliant()` in `estimator.py` are the one place this is computed; both the detector and the display call through them. `test_a_boundary_measurement_rearms_the_display_like_the_detector` drives a measurement that is raw-over-limit but conservatively compliant |
| A6-M3 | Med | The sensor contract said a changed CameraInfo resets the estimator; nothing detected the change | Fixed `22589e4`. `calibration_materially_changed()` compares K, D, dimensions, distortion model, and frame; `Calibration` carries no timestamp so a stamp-only change cannot trigger it. `test_a_distortion_model_change_resets_the_observation_window` keeps K/D numerically identical to isolate the reset from PnP accuracy |
| A6-M4 | Med | `CLAUDE.md`, the documentation map, and release document carried stale/conflicting state | Fixed. `docs/README.md`'s status header and capability matrix, `docs/DESIGN.md`'s prim tree and measured-figures section, and `docs/RELEASE-v1.0-interview.md`/`docs/ACTIVATION.md` (marked paused/pending-refresh rather than rewritten, since neither can be re-measured without a GPU) all updated in the documentation commits on this branch. **This disposition overclaimed — see round 7** |
| A6-L1 | Low | Lane-width and finite-dimension validation failed late or with misleading errors | Fixed `75f0581`. Reproduced first: a NaN `corridor_length_m` built cleanly through `load_scenario()` and only failed inside `validate_layout()` with an unrelated "B does not stand in the next street" message. `_require_finite()` checks every numeric scenario field before any sign/range check; `resolve_profiles()` gets the equivalent guard for `--m`/`--n` |

A6-M2 and A6-M3's regressions (`test_a_boundary_measurement_rearms_the_display_like_the_detector`,
`test_a_distortion_model_change_resets_the_observation_window`,
`test_a_timestamp_only_change_does_not_reset`) need the colcon-generated
`corridor_interfaces` package and are skipped under a bare `pytest` run, the
same way the rest of `test_enforcement_view.py` always has been; run
`colcon test` to execute them.
Do not record a disposition here until its behavior and regression are committed.

## Round 7 — independent review of round 6's own fixes

Round 6 fixed the police-placement and verifier-binding defects and asked for
independent review before GPU requalification. That review found five further
issues in round 6's own implementation — one High, three Medium, one Low — all
confirmed by reproduction before being fixed here.

| ID | Sev | Finding | Disposition |
|---|---|---|---|
| A7-H1 | High | `stage_camera_facts()` derived the camera's world position and FOV from the composed stage but never its rotation, so the analytic certificate and the mesh raycast audit both computed every heading along the route from the manifest-derived trajectory alone. Reproduced first: rolling the stage camera 180° about its own local Z axis (same position, same aperture, same forward axis — only the up axis flips) left `verify()` reporting `passed=True` with 396 mesh-audit rays and 0 failures, unchanged from the unmutated stage | Fixed `ab6c787`. Forward and up are now read from the stage's actual local-to-world rotation via `TransformDir` and checked against the manifest-implied route-start heading, the same way position already was. `test_stage_only_camera_rotation_is_rejected` reproduces the exact roll; a second, unmerged manual check confirmed a 180° yaw (camera facing backward) is rejected too |
| A7-M1 | Med | The lane-width check (`lane_width <= 2*turn_radius_m - clear_width_m`, added for A6-L1) could not fire at the scenario's own default turn radius: with `clear_width_m=6.0` and `turn_radius_m=2.0` the right-hand side is already −2.0, and `lane_width` is never negative. The A6-L1 regression only exercised it by also widening `turn_radius_m` to 4.0, which its own docstring admitted was necessary | Fixed `d115e0e`. Swept `depth_fraction` and `turn_radius_m` against the real downstream route-fits-in-drivable-space check to confirm the true boundary is not a closed form these three fields alone can reproduce exactly (it also depends on the corridor taper heading, a per-profile fact this scenario-level check does not have). Replaced the formula with a deliberately conservative, profile-independent sufficient condition — `lane_width <= turn_radius_m` — documented as a coarse pre-check, not the tight bound. The regression now widens only `depth_fraction`, leaving the default turn radius untouched |
| A7-M2 | Med | RViz's `_readout_marker()` computed the printed compliance margin from the raw speed while `_on_estimate()` already rearmed the marker's color on the shared conservative-speed path (A6-M2's own fix). A measurement over the limit raw but conservatively compliant therefore turned the display green while printing a negative margin under a "compliant" label | Fixed `01527b7`. The margin now goes through the same `conservative_speed_mps()` the color decision uses. Reproduced first, then fixed: at `speed_mps=0.85`, `speed_stddev_mps=0.04`, `confidence_sigma=2.0`, `limit_mps=0.8`, the old code printed "compliant −0.05 m/s margin"; the regression now asserts the literal text "compliant +0.03 m/s margin" and that "-0.05" is absent, not just marker color |
| A7-M3 | Med | A6-M4's disposition claimed `docs/README.md` was fully reconciled. It was not: the resource-envelope table described the R17 plate-relocation's 3,486 MiB figure — measured weeks before ADR 0019 moved P — as "the live demonstration on the corrected geometry", which misattributed the number and contradicted the pending-refresh banners `ACTIVATION.md`/`RELEASE-v1.0-interview.md` already carried. No mechanical test tied any of this to the canonical evidence | Fixed `b867536`. Added the same pending-refresh banner to `docs/README.md` and corrected the resource-envelope row to cite the live-demo's own 3,354 MiB figure with an honest predates-ADR-0019 caveat. `test_docs_readme_gpu_figures_stay_labelled_pending_refresh` in `test/test_repository_contract.py` pins both, closing the "nothing extended the contract tests" half of the finding |
| A7-L1 | Low | `corner_screen_bounds()` set the screen's north (Y) face to P's own north edge — 0.3 m short of the true north wall — while `building_footprints()` and `validate_layout()` both described the screen as hanging from that wall | Fixed `e79b63c`. The 0.3 m gap was originally load-bearing (avoiding occlusion of north-wall reference plates surveyed in the screen's x-range); those plates were relocated west of the screen's x-range entirely earlier in this same audit, so the gap had become stale, undocumented debt. Confirmed empirically before closing it — extending the screen to the true wall face left every authored profile's certificate (ray count, failures, nearest-blocking distance) byte-identical, since no marker's x-range overlaps the screen's — then closed it rather than only correcting the prose |

All five fixes are additive commits on `audit/police-placement-2026-07-29`;
none touch GPU/Isaac evidence, which stays paused pending this round's own
independent review. Full workspace check after all five:
`env ROS_LOG_DIR=/tmp/corridor-twin-ros-log bash tools/check_workspace.sh` —
ruff clean, 187 portable tests passed / 1 skipped (up from 185/1), 128 colcon
tests / 0 failures (up from 127; the new repository-contract test lives
outside any ROS package and does not add to the colcon count).
