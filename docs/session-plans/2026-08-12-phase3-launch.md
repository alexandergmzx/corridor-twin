# Close correction 2, launch Phase 3

> **APPROVED 2026-08-12 23:23 CST. UNATTENDED** — Alexander is away, so the hard
> rules bind in full: git is local-only and `git push` does not exist tonight;
> history is append-only; a commit is a green checkpoint or it does not happen;
> Isaac is single-occupancy under `/tmp/fleet-isaac.lock`; judgment calls are
> **parked, not decided**.
>
> **Budget 9 h — started 23:23, ends 08:00. No new unit starts after 07:30.**
> `date` between units. Branch `phase3-launch-2026-08-12` from
> `gate-green-2026-08-12` (`865f1d7`). The handback is this document's final
> section and is written even if the session fails early.

**Naming.** The task author is referred to by role, never by name, in code,
docs, commits and evidence. Corrected in V0 where it was tracked.

## Live status

| Unit | State | Notes |
|---|---|---|
| V0 session plan | **DONE** 23:26 | lifecycle-glob fix `2253bf8`; name scrub `2973e04` |
| V1a ADR 0031 — B is the cylinder | **DONE** 23:45 `05f3173` | bearing cone had to widen 60°→76°: the merged B is refused by the old one at 0.3 m |
| V1b the camera prim moves to P | **DONE** 23:55 `4316307` | one `UsdGeom.Camera` on the stage and it is P's; certificate green; LOS 5/5 on all three profiles |
| V2 corner-arc yaw, offline | **DONE** 00:03 `ad662d9` | **the corner is innocent** (arc 1.02–1.10); the gate compared two windows |
| — *handoff at 00:35: priorities reordered* | | lens first, then run duration |
| L1 lens harness (stub + probe) | **DONE** 01:15 `8df179d` | headless chromium: screenshot **and** console. No MCP, nothing installed |
| L2 the lens map | **DONE** 01:15 `8df179d` | `OX`/`OY`/`SC` undefined → the render loop died on frame one. **It had never rendered live** |
| L3 stuck → diagnosed FAIL | **DONE** 01:38 `0412009` | wall-clock deadlines, log-watching bringup, INT/TERM exits. Negative control: diagnosed FAIL at +262 s |
| L4 shorter runs | **DONE** 01:44 `9e1accb` | bring-up **144 s → 89 s (−38%)**, total 403 → 359 s |
| L5 corrections owed | **DONE** 01:49 `501e8f7` | the false test count; two REAL orphans the preflight caught; 25 stray markers |
| V3 acceptance re-runs | **DONE** 02:19 `d58266c` | **both gated profiles still RED**; yaw is a spread, not a bias |
| V5 the camera session | **DONE** 02:46 `b98292d` | certificate **GREEN**, mutation **RED** on P's mast; a drive-speed literal had rotted at 0.30 scale |
| V6 Replicator dataset | in progress 02:58 | smoke 20/20 rendered, label overlay verified by eye; bulk running |
| V4 demo-candidate run | not started | deprioritised: the arrival gate is red, so a docked run cannot reach DELIVERED |
| V7 training | pending V6 | |

**The 00:35 handoff reordered the night.** Two priorities came in: the lens no
longer showed the SLAM map, and runs were long and hung silently. Both are
closed — and the lens turned out never to have rendered live at all, which is
recorded in [`docs/evidence/lens/NOTES.md`](../evidence/lens/NOTES.md). The
five tooling units cost 95 minutes against a 3-hour budget.

## What this session inherits

Correction 1 (domain isolation) is closed — ADR 0026, certificate green with
mutation red. Correction 2 (autonomous navigation) **works and its gate is
red**: A delivers to 2–11 cm of the standoff on all three profiles, but
duplicate-wall reads 0.78–0.84 m against 0.20, and the yaw scale reads
1.108–1.166 against 1.0 ± 0.1 on both tapered profiles while `uniform` — the
untapered one — passes at 1.060. Correction 3 (active AI/ML) has never started,
because P's camera was never placed.

Placing it turned out to be a decision rather than a task: ADR 0019's corner
screen, authored so that A cannot see P, also hides the corridor from P. From
P's own height 0/5 enforcement stations are visible; from a 1.50 m mast on P's
own footprint, 5/5 clear in 3-D at 2.29–4.68 m.

**This handoff ratifies that mast pose.** The memo records the ratification and
the session proceeds on it.

## V0 — session plan and two corrections. **DONE**

Three commits, each green first:

1. **`2253bf8`** — the lifecycle poll globbed `*active*`, which matches
   `"inactive [2]"`. Four of 2026-08-12's runs were declared ready while
   `bt_navigator` was still configuring, sent their goal, and were refused. It
   read as a flaky bringup race for a whole session. Both polls now test the
   state's prefix, and the second test runs the four real states through the
   real glob.
2. **`2973e04`** — the name scrub.
3. This file.

## V1a — ADR 0031: B is the cylinder. **75 min hard**

No GPU, and first, because every render tonight must show the final scene.

Today there are two prims: `/World/Actors/B`, a `Cube` 0.135³ × 0.51 at
`(5.038, −2.4, 0)`, and `/World/Actors/BLandmark`, a `Cylinder` r 0.12 h 0.5,
0.8 m south. They become **one cylinder at the delivery point**, in the
drawing's pocket, radius from config through the manifest.

**The radius is the sensor's, the height is the person's.** 0.12 m is set by
MS200 angular resolution and stays in ADR 0030's named unscaled set; the height
scales with the scenario, 1.7 → 0.51.

Removed with it: the B↔post separation floor (`geometry.py:727-738`), which
existed *only* so the detector could cluster the two apart, and the
`landmark_offset_m` constraint set — one object supersedes both.

**Contact semantics ride the same ADR.** Final-approach distance =
`max(governor stop floor, robot half-length + B radius + clearance)`, derived
from committed constants, never a literal. The governor's 0.35 m is a **laser
range**, so its term is 0.35 + B radius + the lidar's forward offset, and it
wins against 0.0975 + 0.12 + clearance. That is the honest result: **the
governor is never bypassed and the demo win is defined at a distance it
permits.** Demo win = `DELIVERED` with world-frame distance-to-B ≤ that value,
on the evaluation plane. **Nav2's 0.15 m arrival gate is unchanged.**

Containment is re-derived, not carried: the goal↔target distance drops
**1.000 m → 0.600 m**, and the ±60° bearing cone was sized for the old offset.
**The spawn-region negative control must still go red.**

**Acceptance:** `scene.build` both configs → occlusion certificate green at the
as-run scale → three arenas rebuilt → `check_arena_matches_manifest.py` passes →
`check_workspace.sh` green. **Plus** the 3-D line-of-sight re-check for the
ratified mast against the merged-B stage — B is a new opaque cylinder near P's
corner and the memo's 5/5 was measured without it.

**Skip-edge at 75 min:** ship config, composer, manifest, detector path and the
ADR; park the viz/doc cosmetics as a following commit.

## V1b — the camera prim moves to P. **45 min hard**

No GPU. Pulled forward out of V5 so that one arena build serves V3–V6 and the
risky part happens early rather than at 04:00 inside an Isaac session.

The adapter cannot point at a prim that does not exist, so the stage must carry
P's camera first. **Moved, never duplicated:** the composer authors exactly one
`UsdGeom.Camera` and it is P's, at P's authored-bounds midpoint plus a new
`police.camera_mast_height_m` — the 1.5 that is currently the one literal in
`p_cam_candidates.py` becomes config-owned. `/World/Actors/A/CameraMount` stays
as a plain Xform: A is camera-less, and the geometric proofs keep their eye
point.

Nothing on the ROS side changes — topics, `frame_id`, allowlist and certificate
are already `/p_cam/*`. The naming was migrated ahead of the geometry.

The static-probe schedule treats the camera pose as A's pose plus mount height.
A fixed mast makes `expected_station_x_m` meaningless: **that path raises, it is
not silently repurposed.**

**Skip-edge at 45 min:** if `occlusion.py` fights back, revert V1b whole with a
new commit that says it reverts, run V3/V4 on the V1a scene, and let V5 do the
prim move inside its own box at the cost of one arena rebuild.

## V2 — corner-arc yaw, offline first. **60 min hard, no GPU**

**`/imu/data` is in none of the bags.** The recorder's fixed topic list carries
`/imu` but not the madgwick output, so the offline chain is
**truth → `/imu` (raw gyro) → `/odom` (EKF)**; the madgwick stage exists only as
the live tap already in each run's `gate.json`.

That tap already says most of the answer:

| profile | truth | `/imu` | `/imu/data` | `/odom` |
|---|---|---|---|---|
| nominal | −77.54° | ×0.9997 | ×0.9995 | **×1.1663** |
| wide_corner | −68.54° | ×1.0022 | **×1.0953** | ×1.1081 |
| uniform | −344.31° | ×1.0670 | ×1.0670 | ×1.0597 |

On nominal the **entire** excess appears at the EKF. `wide_corner`'s ×1.095
between `/imu` and `/imu/data` is the first thing to settle: madgwick
republishes `angular_velocity` unchanged, so an identity step cannot scale
anything, and `uniform` reports the two as bit-identical — that is the control.
Integrating `/imu` from the bag over the same window falsifies or confirms it
without a GPU.

New tool extends `odometry_scale_audit.py`, whose reader, windowing,
straight/turning split and multi-bag summariser transfer unchanged; the arc
label comes geometrically from `trajectory_from_manifest()` — nominal's arc is
station **3.4348–4.4519 m** (97.1°), wide_corner's **3.4520–4.4319 m** (93.6°) —
and is cross-checked against the kinematic one. `corridor_yaw_audit.py` is
**not** usable: it is a live bench instrument that commands a pivot.

Deliverable: per profile × {arc, straight, whole} × {truth, `/imu`, `/odom`},
JSON plus promoted NOTES, and **the stage named in one sentence**. Then by
outcome: corridor-side → one commit with an A/B (the fleet's offline EKF
harness, 25 min timebox, park if it does not run out of the box); fleet-side →
measure cleanly, park as a fleet OI with the evidence, state the gate
consequence plainly. **No `slam_toolbox` or Nav2 tuning under any outcome**
(ADR 0029's law).

## V3 — acceptance re-runs. Tight profiles, dock off, lens up

`nominal_m6_n3` and `wide_corner_m6_n4_5`, gated, domain 67, one run per call.
Masked map ≤ 0.20, pinned thresholds from their one constant, the startup
criterion, zero landmark events, session-scoped artifacts with arena and
manifest hashes.

**Stated honestly:** these run against the merged-B scene, so closest-approach
is not like-for-like with the 08-12 evening table. The map and yaw metrics are,
because neither depends on B. If V2 parked fleet-side they run anyway and the
residual is reported against the named cause — a red with a measured mechanism
is deliverable evidence. Infrastructure failures are reruns, twice at most,
classified explicitly.

## V4 — demo-candidate run

`nominal_m6_n3`, dock **on**, merged B, lens up, labelled a demo candidate and
never quoted as a gate. Acceptance: `TRANSIT → ACQUIRE → REFINE → DELIVERED`
with world-frame distance-to-B logged against V1a's derived threshold. One run;
a failure is a finding, not a retry loop past two.

## V5 — the camera session. Isaac, lock held into V6

Point the adapter at the mast prim, run, verify. **Isolation certificate green
with mutation red in the new topology** — P's camera originates in A's plane and
crosses per ADR 0026's mechanism; the allowlist is unchanged, which is the claim
being re-verified rather than assumed. Capture one 640×360 and one 1280×720
reference frame of the corridor with A mid-route, with full provenance and VRAM
through `isaac_gpu.py`.

## V6 — the Replicator dataset. The overnight payload, 2 h

**Spec committed before a frame renders.** Replicator is greenfield here and the
adapter's contract tests *ban* `rep.create.render_product` — the generator is a
separate offline authoring tool, and the spec says so: the one-render-product
law governs the runtime demo scene, which still carries exactly one.

Verified installed and recorded in the commit per the installed-version rule:
`omni.replicator.core-1.12.27`, annotator `bounding_box_2d_tight`, labelling via
`isaacsim.core.utils.semantics.add_labels`.

Spec pins counts, the train/val split, the randomization ranges (lighting, A's
yaw, A's station along the route envelope, lateral offset, all three profile
geometries) and the labels: **A's box in P's frame from truth, on the evaluation
plane only**, plus world pose and P-frame range/bearing for later metric error.
Generated at **both** 640×360 and 1280×720, paired on the same scene states so
ADR 0024's resolution decision is made against ADR 0026's measured transport
ceiling. Manifest with hashes; **20 random label overlays spot-validated before
bulk generation**; checkpointed so a partial dataset is usable.

## V7 — training harness and first fine-tune. **45 min**

`transformers` 4.57.6 is already in Isaac's Python, so RT-DETR
(`PekingU/rtdetr_r18vd`, Apache-2.0) needs no install and satisfies ADR 0024's
permissive-only clause with the licence check recorded in the commit.
Ultralytics is rejected on AGPL grounds by that ADR. `pycocotools` is absent —
the metric is val loss plus a hand-rolled IoU@0.5 rate, and nothing is
pip-installed into Isaac's ABI at 07:00. Training holds **no** Isaac lock: the
dataset is rendered once and the harness trains against files on disk.

## Rules for tonight

Two attempts at any failing command for the same reason, then record and move
on. Every gate run writes JSON; thresholds printed and enforced from one
constant; infrastructure failures are reruns, twice at most, classified
explicitly. Gates run dock-off always; demo runs are labelled. Fleet writes only
under V2's narrow terms, as a separate commit. Isaac lock acquired and released
around every session, teardown verified on every failure path. **Nothing
pushed.**

## Handback

*(written at session close)*
