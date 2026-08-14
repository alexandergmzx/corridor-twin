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
| V3 | T1 ROS bench | 30 m | |
| V4 | T2 lens tile | 45 m | |
| V5 | T3 micro-arena — the bump on screen | 45 m | 2 attempts max |
| V6 | Orphan reap between nav attempts, then T4 | 60 m | |
| V7 | Evidence + ADR 0034 + the `abbf610` correction | 45 m | never skipped |

---

## Log

| unit | outcome |
|---|---|
| V0 | branch `bump-bench-2026-08-14`, this document |
| V1 | `tools/creep_bench.py`. Reproduces the cone leak, the slow-zone false stall, the slip case and the forgery. **The bag-reproduction bar is MISSED**: pin at declared 0.4029 m against the bag's 0.3455 m, 0.057 m out against a ±0.03 m bar. Mechanism matches, radius does not — recorded, not resolved, because the real robot got *closer* than the bench and still never contacted |
| V2 | A1 disc + A2 exemption in the fleet (`c9c773b`, under grant); A3 dual witness in `corridor_dock.py`. Bench: `disc`/`slip`/`misaligned` reach contact within 0.6–1.4 mm; the three cone scenarios stay red as negative controls. **The single-beam fixture blindness is dead** — `_disc_scan()` ray-traces a real cylinder and the cone's failure is now asserted. Fleet suite 97, of which `test_governor.py` 49 → 54 |

## Handback

*(written at session end, or on early failure — whichever comes first)*
