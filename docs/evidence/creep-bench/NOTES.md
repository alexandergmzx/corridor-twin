# Creep bench — why A could not touch B, and what fixed it

Closed-loop offline bench for the terminal docking creep. No ROS, no GPU, no
Isaac: 13 s for the whole matrix against ~25 minutes for one Isaac cycle. It
exists because nine Isaac runs on 2026-08-13 produced zero completed
deliveries, and every bug behind them was catchable in milliseconds.

## Command

```bash
cd /home/alexmint/Development/robot-fleet/src/corridor-twin   # the SYMLINKED path (D5)
source .venv/bin/activate
python tools/creep_bench.py --scenario all \
  --profile nominal_m6_n3 \
  --json out/evidence/creep-bench/matrix-nominal.json
```

Run it from the symlinked checkout or set `CORRIDOR_FLEET_SRC`; from the
physical path the fleet imports resolve into `~/Development/yahboomcar-ros2`
and the bench exits. `test/test_creep_bench.py` covers the decisive scenarios
in the ordinary suite and works from either path.

## What it runs

Real components, wired as the live gate wires them: `LandmarkDetector` →
`DockingMachine` (`tools/corridor_dock.py`) → the fleet's
`governor.forward_min_range` / `governor.decide`. Scans come from the fleet
raycaster (`yahboomcar_sim/arena.py`) over `export_scan_walls.wall_segments()`
for the authored corridor, 360 beams, `r_min` 0.12 m. Unicycle kinematics at
10 Hz, the rate `corridor_nav_gate` spins at.

**B is a 32-sided polygon, not a beam.** That is the whole point. Maximum
radial error 0.6 mm. Every docking test in the fleet's `test_governor.py`
modelled the target as a single return at a single bearing, and a one-beam
target cannot leak outside an angular mask — which is how a green safety suite
coexisted with a robot that could not reach its target.

| | |
|---|---|
| Isaac Sim | not used — this is the point of the bench |
| GPU | not used |
| Scan model | 360 beams, `r_min` 0.12 m, `r_max` 8.0 m, fleet raycaster |
| Control rate | 10 Hz |
| Profile | `nominal_m6_n3` |
| Contact range | 0.2175 m centre-to-centre, from the authored radii |

## Result — 2026-08-14

| scenario | mask | slow-zone | state | contact | final r | t |
|---|---|---|---|---|---|---|
| `cone_leak` | ±15° cone | applied | `ARRIVED_UNPROVEN` | **no** | 0.4031 | 25.1 s |
| `slow_zone_false_stall` | ±15° cone | applied | `DOCKING` | **no** | 0.4031 | 25.0 s |
| `forgery` | ±15° cone | applied | `DOCKING` | **no** | 0.4031 | 25.0 s |
| `disc` | silhouette | exempt | `DELIVERED_CONFIRMED` | **yes** | 0.2181 | 9.4 s |
| `misaligned` | silhouette | exempt | `DELIVERED_CONFIRMED` | **yes** | 0.2189 | 11.4 s |
| `slip` | silhouette | exempt | `DELIVERED_CONFIRMED` | **yes** | 0.2181 | 9.4 s |

**PASS** on the three fixed scenarios: contact within 0.6 mm (`disc`, `slip`)
and 1.4 mm (`misaligned`) of the 0.2175 m contact range — beam discretisation,
not tolerance on truth. **The three cone scenarios stay RED deliberately** and
are kept as permanent negative controls.

`slip` reaches contact with the pose pinned and the encoders reporting motion
throughout, which is the case that defeats an encoder-only stall detector.

### Duty cycle by range — the signature

Fraction of ticks commanding motion, per band of true range:

| band | cone | disc |
|---|---|---|
| 0.70–0.42 | 1.000 | 1.000 |
| 0.42–0.38 | 0.014 | 1.000 |
| 0.38–0.35 | never reached | 1.000 |
| 0.35–0.00 | never reached | 0.974 |

The cone's collapse is the shape the session bag showed.

## The reproduction, and where it falls short

**Bar (from the session plan): the cone reproduction's pin radius within
±0.03 m of the bag's 0.3455 m. That bar is MISSED, by 0.057 m.**

Like-for-like, on the quantity a bag actually holds — a bag has no ground
truth, so only the *declared* range and the leaked minimum are comparable:

| | bag `20260814-003844` | bench `cone_leak` |
|---|---|---|
| declared range at the pin | 0.3455 m | **0.4029 m** |
| leaked minima over the pinned tail | 0.217–0.334 m | 0.283–0.344 m |
| governor reason | `obstacle at 0.24–0.32 m` | `obstacle at 0.34 m` |
| duty collapse | 98 → 28 → 12 → 0 % | 100 → 1.4 → — |

The bench's own detector is essentially unbiased (declares 0.4029 against a
true 0.4031), so the gap is not a units mismatch — it is a real difference.
The bench pins **further out** than the robot did, and its leaked band is
shallower. The mechanism reproduces; the exact radius does not.

The likely cause is detector bias on real returns: if the live detector
under-declares range by ~0.06 m, both runs are the same physical pin. That is
a hypothesis, not a measurement, and it is untested. It is recorded rather
than resolved because it does not change the fix — the real robot got
*closer* than the bench and still never contacted, and the disc admits the
target at every range including contact (asserted at four ranges in the
fleet's `test_the_disc_mask_admits_the_target_at_every_range`).

**Do not quote this bench as a validated replica of the bag.** It reproduces
the failure mode, not the run.

## The four blockers, and their scenarios

1. **The ±15° cone cannot admit a contact.** B's half-width is `asin(0.12/r)`:
   over 15° everywhere inside 0.4636 m, 33.5° at contact. Below ~0.41 m the
   target's own shoulders fall outside the mask and inside the 0.35 m stop, so
   the filter brakes on the object it was told to drive into. → `cone_leak`.
2. **The stub throttles the creep.** `EastWallStub`'s south face sits 0.315 m
   off the approach line, entering the ±45° sector at 0.4455 m for the entire
   creep: slow-zone scale 0.174, so 0.05 m/s becomes 8.7 mm/s — 46 s of travel
   against a 25 s budget, and *below the controller's own 10 mm/s stall
   threshold*, so a healthy creep reads as contact. → `slow_zone_false_stall`,
   which fires a false arrival 1.1 s in without the exemption.
3. **Wheel slip defeats the encoder stall.** Rear friction is authored at 0.1
   (`build_corridor_arena.py:126`) and `/odom_raw` integrates joint velocities,
   so at a real bump the wheels spin and `measured_vx` never drops. → `slip`.
4. **A governor stop forges a bump.** A stop imposed by the filter is
   indistinguishable from contact to a bearing-and-encoder witness, and the
   leak pins A at 0.31–0.35 m — inside the old 0.39 m sighting ceiling. →
   `forgery`, which reported `DELIVERED_CONFIRMED` without touching B before
   the dual witness, and now refuses.

## Correction to the record

`abbf610` diagnosed run `20260814-003844` as *"the mask was never reaching the
governor — the creep was gated behind a TF lookup"*. **The bag replay
contradicts that.** The governed duty cycle is 98% while range closes from
0.70 to 0.42 m — well below the 0.47 m at which B's nose crosses the 0.35 m
stop, where a dead mask would already be braking. The mask was live and being
fed. The TF fix was real but secondary; the leak is what pinned A.

The in-process proof offered alongside that diagnosis — "0 of 31 ticks moving
without the approach, 29 of 31 with it" — fed B as a **single beam**, which
cannot leak. It measured that the mask was plumbed in, and was reported as
though it measured that the mask worked.
