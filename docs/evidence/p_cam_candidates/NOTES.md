# Where P's enforcement camera can stand — a decision memo

> ## RATIFIED 2026-08-12 (evening)
>
> **P's camera pose is `corner_mast_over_the_screen`: the 1.50 m mast at P's
> own position.** Ratified by Alexander in the Phase-3-launch handoff, on the
> 5/5 line-of-sight, 5/5 frustum, 2.29–4.68 m row below.
>
> Implemented the same session. The mast height is now authored as
> `police.camera_mast_height_m` (5.0 m authored → **1.50 m** at the committed
> 0.30 factor), the pose is derived once by `scene.geometry.p_cam_pose` and
> reaches the composer, the manifest and the occlusion certificate from there,
> and the stage's single `UsdGeom.Camera` is
> `/World/Actors/PCameraMast/PCam`. A is camera-less; `/World/Actors/A/
> CameraMount` survives as a plain Xform because the geometric visibility gate
> is cast from A's eye.
>
> **Re-measured against the merged-B stage** (ADR 0031, which turned B into one
> cylinder at the delivery point) and now by a committed, repeatable tool
> rather than an ad-hoc snippet:
>
> ```
> python3 tools/p_cam_line_of_sight.py --profile <p> \
>   --out out/evidence/p_cam_candidates/los-3d-<p>.json
> ```
>
> | profile | usable | range to A | worst bearing off axis |
> |---|---|---|---|
> | `nominal_m6_n3` | **5/5** | 2.34 – 4.81 m | 21.8° |
> | `wide_corner_m6_n4_5` | **5/5** | 2.34 – 4.81 m | 22.7° |
> | `uniform_m6_n6` | **5/5** | 2.37 – 4.81 m | 24.7° |
>
> 72 opaque triangles, the same set the A-cannot-see-P proof casts against.
> The ranges differ from the 2.29–4.68 m below because these stations are
> fractions of the approach leg rather than the absolute 0.60–3.00 m used
> originally; they also now cover **all three profiles**, where the original
> claimed only nominal. B is not an occluder in either measurement: the
> raycaster walks `/World/Environment`, and B stands under `/World/Actors`
> down the next street, away from the corridor sightlines.
>
> **Still open, and parked for the morning:** whether the mast wants visible
> scenery. Only the camera prim is authored — no pole, no collider — so the
> occlusion certificate is untouched and the choice is reversible. A camera
> floating at 1.5 m reads badly in a third-person viewport.

**2026-08-12, geometry only, no GPU.** The original memo follows, unedited.
**It chose nothing; the ratification above is what chose.**

```
python3 tools/p_cam_candidates.py --manifest out/corridor.manifest.json \
  --profile nominal_m6_n3 --out docs/evidence/p_cam_candidates/geometry-nominal_m6_n3.json
```

Artifact: [`geometry-nominal_m6_n3.json`](geometry-nominal_m6_n3.json). Scene:
the committed 0.30-scale manifest, arena hash-checked against it. Camera
contract read from the manifest, not restated: **640 × 360, 75° horizontal,
mount height 0.21 m**.

## The finding, before the table

**P cannot see the corridor from where P stands — at P's own height.** All five
enforcement stations are blocked from a camera on P's body, and so is every
point of A's approach. Raising the same footprint to a 1.5 m mast clears all
five, so the constraint is height, not position.

The blocker is ADR 0019's corner screen — the partition authored specifically so
that **A cannot see P**. It works. It also blocks the reverse sightline, and
ADR 0021 then made P's camera the only sensor that matters. The two decisions
are individually sound and jointly contradictory, and nothing had measured the
contradiction because P's camera has never been placed.

This is a decision for you, not a bug to fix quietly. Two poses clear the screen
and the geometry costs of each are below.

## Candidates

Measured per enforcement station (0.60, 1.20, 1.80, 2.40, 3.00 m along the
approach): can the camera see **A** there — line of sight against the authored
walls, and inside the declared frustum.

| candidate | eye (m) | 3-D line of sight | in frustum | usable | distance to A |
|---|---|---|---|---|---|
| `at_P_down_the_corridor` | (5.24, 0.72, 0.21) | **0 / 5** | 5 / 5 | **0 / 5** | — |
| `at_P_raised` | (5.24, 0.72, 0.63) | **0 / 5** | 5 / 5 | **0 / 5** | — |
| **`corner_mast_over_the_screen`** | (5.24, 0.72, **1.50**) | **5 / 5** | 5 / 5 | **5 / 5** | 2.29 – 4.68 m |
| `north_wall_before_the_screen` | (3.30, 0.81, 0.42) | 5 / 5 | 4 / 5 | 4 / 5 | 1.05 – 2.80 m |
| `north_wall_midpoint` | (1.80, 0.81, 0.42) | 5 / 5 | 1 / 5 | 1 / 5 | 1.26 m |

*Line of sight is the real 3-D test — `scene.occlusion`'s own raycaster against
the stage's 72 opaque triangles, to A's body centre at 0.075 m. Artifact:*
[`line-of-sight-3d-nominal_m6_n3.json`](line-of-sight-3d-nominal_m6_n3.json).

### What each pose costs the detector

**`north_wall_before_the_screen`** is the closer of the two poses that work.
A is 1.05–2.80 m away across the four stations it covers, which is a comfortable
range for a 640 × 360 frame: A's 0.195 m body spans roughly 45 px at 2.8 m and
120 px at 1.05 m on a 75° lens. It loses station 3.00 m, which falls 39.8° off
axis against a 37.5° half-frustum — **just outside, by 2.3°**. A slightly wider
lens or 5 cm of repositioning recovers it, and that is a real choice rather than
a rounding error.

It is **not on P's body.** P stands at the corner; this is a wall mount 1.9 m
west of P. Whether that is still "P's camera" is a scenario question: it is
ordinary for roadside enforcement (police cameras are pole-mounted, not
head-mounted), and ADR 0021 says the render product is P's instrument without
saying it is P's eyes. Nothing in the ADR forbids it. Worth an explicit line in
whatever ADR settles this.

**`north_wall_midpoint`** sees A pass beside it — four of five stations are in
line of sight but out of frustum, because a camera inside the corridor looking
along it has A crossing its field rather than approaching down it. It also has
the shortest range, 1.26 m, which is the best pixel density on offer and the
worst coverage.

**The two at P's own position** see nothing of the corridor. They would see B's
end of the street, which is where the delivery happens and not where speed is
measured.

### The mast: flagged in 2-D, then measured in 3-D, and it clears

The plan-view test called the mast blocked, because the screen is in the way in
plan — which is exactly what a mast is meant to defeat. The screen and the
corridor walls are 1.2 m tall at this scale, so the 2-D answer was structurally
incapable of judging it.

The 3-D check was then run: `scene.occlusion`'s own raycaster, the same one that
proves A cannot see P, against all 72 opaque triangles on the stage.
**`corner_mast_over_the_screen` sees all five stations, blocked by nothing**,
with every bearing within 1° of dead ahead.

That makes it the only candidate that is 5/5 on both tests, and it is the one
that keeps the camera **at P** — no separate mount, and no question about
whether a wall fixture 1.9 m away is still "P's camera". Its cost is range: A is
2.29–4.68 m away rather than 1.05–2.80 m, so the subject is smaller in frame.
On a 640 × 360 sensor at 75°, A's 0.195 m body spans roughly **27 px at 4.68 m**
and 55 px at 2.29 m.

**27 px is the number to argue about**, and it is a detector question rather than
a geometry one: it is comfortable for a learned box detector and marginal for
the ArUco-on-A baseline, whose plate would be a fraction of that. Whether the
baseline needs its own closer pose, a larger plate on A, or a longer lens is the
first thing the chosen pose forces a decision about.

## What Phase 3 needs next, in order

1. **This decision.** The detector's training data, its evaluation stations and
   the ArUco-plate baseline all depend on where the camera is. Nothing further
   is worth building until the pose is chosen.
2. ~~A 3-D line-of-sight check on the mast~~ — **done**, it clears 5/5.
3. **One rendered frame per surviving candidate** through the ADR 0009 adapter.
   Not produced tonight: the adapter's camera prim is still
   `/World/Actors/A/CameraMount/FrontCamera`, A's v1 camera, and moving it to a
   P-owned mount is the first implementation step of the chosen pose rather than
   a preview of it. Recorded as the outstanding half of this unit.
4. **Then** the Replicator dataset spec, the training harness, and the
   ArUco-on-A baseline, all of which key off the chosen geometry.

## Scope

Nominal profile only. `wide_corner` and `uniform` share the corridor's north
wall and P's position, so the blocked-from-P finding carries; the usable
distances do not, and are not claimed here.
