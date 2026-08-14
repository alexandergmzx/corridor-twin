# Session plan — the test ladder, then make the bump physically possible

**Branch** `bump-bench-2026-08-14`, from `1d7307a`.
**Started** 2026-08-14 01:46 CST. `date` between units.

Unattended hard rules bind: append-only, green-checkpoint commits, Isaac
single-occupancy, park-don't-decide, lens on every live run, **nothing pushed**.

**Isaac occupancy discipline, restated because I broke it twice last night:**
the clear-check runs as its **own step**, its output gates the launch, and it
uses a `/proc` scan — `pgrep`/`grep` forms match their own command line.

---

## Why this session exists

Nine Isaac runs last night, zero completed deliveries. Every creep bug cost a
~25-minute Isaac cycle to find, and all of them were catchable offline in
milliseconds. The directive: **restructure testing cheapest-first — lens early,
Isaac last — and fix that A cannot collide with B.**

Two agents verified the ground: one on build facts, one adversarial on the
geometry including a **replay of run 20260814-003844's session bag**.

---

## The blocker chain: four, not one

### 1. R1's ±15° mask cone makes contact geometrically impossible — CONFIRMED

B's angular half-width is `asin(0.12/r)`: more than 15° everywhere inside
**0.4636 m**, and ±33.5° at contact. Below ~0.41 m the leaked shoulder returns
sit inside the 0.35 m stop. The binding return is not the tangent but the beam
just outside the cone edge:

    d(15°) = r·cos15° − sqrt(0.12² − r²·sin²15°)
    at r = 0.3455  ->  0.254 m     (my earlier 0.324 was the tangent, not the binding return)
    crosses 0.35   ->  r ≈ 0.407–0.417, beam-phase dependent

**Bag replay, run 003844.** Governed `/cmd_vel` duty cycle collapses exactly on
this geometry: **98%** of ticks moving while r 0.70→0.42, **28%** at 0.42→0.38,
**12%** at 0.38→0.35, **0%** below. Leaked minima across the pinned tail:
0.217–0.334 m — precisely the "obstacle at 0.24–0.32 m" the governor logged.

### **A correction I owe the record**

`abbf610` diagnosed this run as "the mask was never reaching the governor — the
creep was gated behind a TF lookup", and the handback repeated "the mask works;
the caller was not feeding it". **The bag contradicts both.** The 98% moving
phase extends below r = 0.47, where B's nose is already inside 0.35 m — a dead
mask stops there. So the mask *was* live and being fed; the TF fix was real but
secondary, and the leak is what pinned A.

My in-process proof ("0/31 ticks without the approach, 29/31 with it") fed B as
a **single beam**. A one-beam B cannot leak. That is the same fixture blindness
as every docking test in `test_governor.py` (`_ahead()` — one return, one
bearing), and it is why a green unit test coexisted with a red robot.

### 2. The stub's south face throttles the creep to 8.7 mm/s

`EastWallStub`'s south face (y = −2.085) lies 0.315 m north of the creep line,
so it enters the ±45° sector at 0.315/sin45° = **0.4455 m** for the entire
aligned approach. Slow-zone scale (0.4455−0.35)/0.55 = **0.174** → 8.7 mm/s.
Budget to contact ≈ 46 s + pivot + debounce ≈ **50 s against a 25 s timeout**.
Even a perfect mask times out. (The east wall I blamed is occluded by B for
most of the creep; measured unmasked minimum is the stub at 0.42–0.49 m.)

### 3. Wheel slip defeats the encoder stall

The twin models it **deliberately**: rear friction authored 0.1
(`build_corridor_arena.py:126`), `/odom_raw` integrated from joint velocities
with a measured 92% wheel/world disagreement on the stand
(`sim_runner.py:709-751`), EKF fuses wheel twist only. At a real bump the
wheels may spin and `measured_vx` never drops.

And the naive laser witness fails: stationary 1-s displacement measures median
16.8 mm but **p95 374 mm** (ICP re-registration jumps) against a true-creep
signal of 34 mm. No epsilon separates them — it needs a robust per-scan-pair
median.

### 4. A governor stop currently forges a bump

Stall accrues whenever EKF vx ≈ 0 while *bearing-aligned*, so a governor-imposed
stop is indistinguishable from contact. The leak pins A at 0.31–0.35 m, **inside**
the 0.39 m sighting ceiling. One solid second forges `DELIVERED_CONFIRMED`
0.12–0.17 m short of contact. It nearly fired in run 003844.

### A standing precondition failure, found by tripping over it

**Every corridor run on this host fails the robot1 Isaac contract, and has
been waved through.** Measured across the last twelve runs: `scan at
13.4–15.1 Hz, want ~12.0` and `battery at 1.6 Hz, want ~1.0`. All eight of
last night's runs passed `--allow-contract-fail`; two attempts tonight without
it stopped at the precondition, which is how this surfaced.

Not a regression and not caused by anything in this session — the scan rate is
a property of the twin, not of the docking mask. But it means the phrase
"contract precondition passed" has never been true for robot1 in the corridor,
and a 17% scan-rate overshoot is not nothing when a stationarity witness is
being tuned against per-scan displacement.

The twin's own bring-up banner explains the mechanism and makes it worse
reading, not better: `/scan` lands on 12 Hz *"by CALIBRATION against a measured
but unexplained 72-messages-per-render-second emission rate, not by the sensor
honouring its configured rate."* So the target is a fitted constant against an
emission rate nobody has explained, and 13.4–15.1 Hz is that fit drifting.

**Morning decision.** Either robot1's corridor contract number is wrong —
CLAUDE.md is explicit that contract figures are per-robot and robot1's must
come from robot1's own measured entries, and a calibrated ~12.0 may simply not
be robot1's number in this arena — or the calibration needs redoing against the
unexplained emission rate. Both are outside this session's scope. Recorded, not
fixed, and not tuned to green.

### Verified non-blockers

The empty-sector fail-closed **never** triggers during the creep — the stub
face and east wall always leave unmasked returns at 0.44–0.82 m. Deadman,
command cadence (no `/cmd_vel_raw` gap > 0.3 s inside the creep) and scan QoS
are all clean.

---

## Ratified this session

| | ruling |
|---|---|
| **A1** | **Mask = target silhouette.** Mask a return iff its point lies within `authored b_radius + 0.10 m` of the declared centre, dead-reckoned by odometry between confirmations. **Authored** radius, because fitted spans 0.072–0.168 m. **New topic** `~/docking_disc`, so an old sender's margin cannot be misread as a disc radius. **It also closes a hole the cone has today**: the cone masks the entire sensor-to-target segment, so a foot planted between A and B is currently invisible to the governor. Wall (0.362 m from B's centre) and stub (0.568 m) verified never masked, ≥0.06 m slack including worst staleness. |
| **A2** | **Slow-zone exemption for the clamped creep.** While a declaration is live, commands at/below the 0.05 m/s clamp skip only the linear slow-down. Hard stop at 0.35, deadman, stale-scan, empty-sector and off-object rules untouched. Braking distance at 0.05 m/s is millimetres. Creep becomes ~8–12 s of visible motion. |
| **A3** | **Stall witness = (governor actually permitted forward motion during the debounce) AND (laser odometry stationary, robust per-scan-pair median over 2–3 s, ε from the 003844 bag).** Encoders corroborate, never suffice. Sighting ceiling 0.39 → ~0.26 m. Used as a *witness*, never fused — consistent with `laser_odometry`'s stated limits. |

---

## The test ladder

| tier | what | cost | catches |
|---|---|---|---|
| **T0** | **Creep bench** — closed loop, no ROS, no GPU. Fleet raycaster `yahboomcar_sim/arena.py:63` over `export_scan_walls.wall_segments()`, **B as a 32-gon**. Real detector + machine + governor chained. Unicycle kinematics, contact and **slip** modes | ms | all four blockers |
| **T1** | ROS in-process, real governor node, scans from the same raycaster (**never single-beam**) | ~20 s | wiring, QoS, staleness |
| **T2** | Lens docking tile; `lens_stub` grows the same keys; `lens_probe` headless | s | display; makes T3/T4 watchable |
| **T3** | **Terminal micro-arena** — A spawns 0.75 m from B. `simctl --no-patrol --no-slam --no-rviz`: no Nav2, no lifecycle manager, **immune to the bring-up hang** | ~3 min | physics contact |
| **T4** | Full acceptance run | ~8 min | end to end |

**Bench validation rule.** T0 must **reproduce run 003844 under the ±15° cone**
— duty collapse from ~0.42 m, pin at ~0.35 m — before its pass under the disc
is trusted. It must also reproduce the forgery and the slip case. A bench that
cannot reproduce the observed failures is decoration.

---

## Queue

| # | unit | box | status |
|---|---|---|---|
| V0 | Plan + branch | 15 m | **DONE** |
| V1 | T0 bench + the four reproductions | 90 m | **DONE** — `c52b505` |
| V2 | A1 + A2 (fleet) and A3 (repo), each proven on the bench; extended-arc fixtures | 90 m | **DONE** — fleet `c9c773b` |
| V3 | T1 ROS bench **+ the live wiring itself** | 30 m | **DONE** — `5b96a1b` |
| V4 | T2 lens tile | 45 m | **SKIPPED** — see below |
| V5 | T3 micro-arena — the bump on screen | 45 m | folded into V6 |
| V6 | Orphan reap between nav attempts, then T4 | 60 m | **DONE** — reap not needed; T4 run `023306` |
| V7 | Evidence + ADR 0034 + the `abbf610` correction | 45 m | **DONE** |

**V3 was rescoped mid-session, and the rescope mattered.** As planned it was a
ROS in-process bench. But the fixes existed only in the bench: `corridor_dock`
carried the radius, and nothing published it. An Isaac run at that point would
have used the old cone and failed exactly as the previous nine did. The live
wiring — disc topic, `/cmd_vel` readback, `/odom_laser` witness — went in first,
and the T1 probe became its direct test rather than a separate tier.

**V4 skipped deliberately.** The lens tile makes the docking state easier to
watch; the existing lens already renders map, scan, pose ghosts and the landmark,
which is enough to see whether A reaches B. With the wiring proven at T1 the
next unknown is physics, and that is only answerable in Isaac. Recorded as a
skip rather than done.

**V5 folded into V6.** The micro-arena's value was avoiding the bring-up hang by
skipping Nav2. Building it needs a `--spawn` flag, a relaxed clearance gate and
a tagged arena filename — roughly the cost of two full runs. The full run was
attempted directly instead; if bring-up proves to be the blocker, the
micro-arena is the fallback and its cost is then justified.

---

## Log

| unit | outcome |
|---|---|
| V0 | branch `bump-bench-2026-08-14`, this document |
| V1 | `tools/creep_bench.py`. Reproduces the cone leak, the slow-zone false stall, the slip case and the forgery. **The bag-reproduction bar is MISSED**: pin at declared 0.4029 m against the bag's 0.3455 m, 0.057 m out against a ±0.03 m bar. Mechanism matches, radius does not — recorded, not resolved, because the real robot got *closer* than the bench and still never contacted |
| V2 | A1 disc + A2 exemption in the fleet (`c9c773b`, under grant); A3 dual witness in `corridor_dock.py`. Bench: `disc`/`slip`/`misaligned` reach contact within 0.6–1.4 mm; the three cone scenarios stay red as negative controls. **The single-beam fixture blindness is dead** — `_disc_scan()` ray-traces a real cylinder and the cone's failure is now asserted. Fleet suite 97, of which `test_governor.py` 49 → 54 |

## Handback

### Where it got to

**A now drives all the way in. Nothing notices when it arrives.**

Closest approach to B went from **0.3455 m to 0.2252 m** against a 0.2175 m
contact — from 128 mm short to **7.7 mm** short — and the governor permitted
**476 of 476** creep ticks at full speed where it previously permitted none
below 0.42 m. The run still reports `ARRIVED_UNPROVEN` and still FAILS, and the
reason is now a different one: not "A cannot move", but "A moved and no witness
saw it stop".

### Commits

| repo | hash | what |
|---|---|---|
| corridor-twin | `c52b505` | T0 creep bench (V1) |
| **fleet** | `c9c773b` | disc mask + slow-zone exemption + ray-traced cylinder fixtures |
| corridor-twin | `de4e36e` | dual witness in `corridor_dock` |
| corridor-twin | `f37c0ae` | bench evidence, and the `abbf610` correction |
| **fleet** | `fc3ac41` | `~/docking_disc` topic, bounded radius, refusal-not-clamp |
| corridor-twin | `5b96a1b` | live wiring + T1 probe |
| corridor-twin | *(this)* | ADR 0034, live evidence, plan |

Fleet work is under the per-session grant amending R1's docking mode, on
`corridor-docking-mode-2026-08-13`. **Nothing pushed, either repo.**

### Gates

`bash tools/check_workspace.sh`: ruff clean, **pytest 479 passed / 1 skipped**,
colcon **142 tests, 0 failures**. Fleet `yahboomcar_safety`: **102 passed**
(`test_governor.py` 49 → 54 → 62).

### The three things that would move this forward, in order

1. **Replace the laser stationarity signal.** Measured this run: the matcher's
   noise floor during a creep is ~4× the true speed (60 mm/s median against
   16.5 mm/s actual; 0.882 m net displacement against 0.393 m). No fixed ε on
   that signal can work. Candidates worth measuring: the change in the
   matcher's own residual, the detector's range trend across the last
   sightings before B goes blind, or drive current/effort. **Do not retune
   0.030** — the problem is the signal, not the number.
2. **Settle whether A touched B.** 7.7 mm short of a *modelled* contact range
   is inside that model's slop. A truth-plane contact flag from the simulator,
   read as an evaluation output only, would answer it in one run and is the
   cheapest thing on this list.
3. **The `--spawn` micro-arena (T3), now clearly worth its cost.** Each full
   run is ~4 minutes of bring-up for ~24 s of the behaviour under test. The
   witness work in (1) needs many iterations of exactly those 24 s.

### Morning decisions

- **The contract precondition has never passed for robot1 in the corridor.**
  `scan 13.4–15.1 Hz` against `want ~12.0` across the last twelve runs; all of
  them waved through with `--allow-contract-fail`. The 12.0 is itself a
  calibration against an unexplained 72-msg-per-render-second emission rate.
  Either the number is wrong for robot1 or the calibration is stale. Not
  touched here.
- **ADR 0034 is Accepted with decision 3 marked open**, because decisions 1, 2
  and 4 are confirmed live and only the witness threshold is refuted. If the
  preference is that a partly-refuted record should be Proposed instead, say
  so and it gets superseded rather than edited.
- **`yahboomcar_config/rviz/slam_debug.rviz` is dirty in the fleet tree** — an
  RViz auto-save from an earlier session, not mine, outside the grant. Left
  untouched.

### Not done, and why

- **T2 lens docking tile** — skipped. The existing lens showed the run
  adequately, and with T1 proving the wiring the open question was physics.
- **Dead-reckoning the declaration between sightings** — specified in A1, not
  implemented. It did not bite this run: the creep held full speed throughout,
  so the stale declaration never cost anything. It will matter once the creep
  spends longer in the blind zone.
- **Sighting ceiling 0.39 → 0.26 m** — deferred. It guarded against the forgery
  that the dual witness now prevents structurally.
