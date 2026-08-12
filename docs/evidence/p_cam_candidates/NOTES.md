# Where P's enforcement camera can stand — a decision memo

**2026-08-12, geometry only, no GPU.** For Alexander. **Nothing is chosen here.**

```
python3 tools/p_cam_candidates.py --manifest out/corridor.manifest.json \
  --profile nominal_m6_n3 --out docs/evidence/p_cam_candidates/geometry-nominal_m6_n3.json
```

Artifact: [`geometry-nominal_m6_n3.json`](geometry-nominal_m6_n3.json). Scene:
the committed 0.30-scale manifest, arena hash-checked against it. Camera
contract read from the manifest, not restated: **640 × 360, 75° horizontal,
mount height 0.21 m**.

## The finding, before the table

**P cannot see the corridor from where P stands.** All five enforcement
stations are blocked, and so is every point of A's approach.

The blocker is ADR 0019's corner screen — the partition authored specifically so
that **A cannot see P**. It works. It also blocks the reverse sightline, and
ADR 0021 then made P's camera the only sensor that matters. The two decisions
are individually sound and jointly contradictory, and nothing had measured the
contradiction because P's camera has never been placed.

This is a decision for you, not a bug to fix quietly. Three ways out, and the
geometry costs of each are below.

## Candidates

Measured per enforcement station (0.60, 1.20, 1.80, 2.40, 3.00 m along the
approach): can the camera see **A** there — line of sight against the authored
walls, and inside the declared frustum.

| candidate | eye (m) | route stations usable | distance to A | plates usable |
|---|---|---|---|---|
| `at_P_down_the_corridor` | (5.24, 0.72, 0.21) | **0 / 5** | — | 0 / 5 |
| `at_P_raised` | (5.24, 0.72, 0.63) | **0 / 5** | — | 0 / 5 |
| `corner_mast_over_the_screen` | (5.24, 0.72, 1.50) | 0 / 5 *(see caveat)* | — | 0 / 5 |
| **`north_wall_before_the_screen`** | (3.30, 0.81, 0.42) | **4 / 5** | 1.05 – 2.80 m | 5 / 5 |
| `north_wall_midpoint` | (1.80, 0.81, 0.42) | 1 / 5 | 1.26 m | 2 / 5 |

### What each pose costs the detector

**`north_wall_before_the_screen`** is the only pose that watches the approach.
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

### The caveat that matters: this test is 2-D

`corner_mast_over_the_screen` is reported blocked because the screen is in the
way **in plan**, which is exactly what a mast is meant to defeat. The screen and
the corridor walls are 1.2 m tall at this scale; a camera at 1.5 m looks over
all of them.

**So the mast is not refuted — it is unmeasured.** Refuting or confirming it
needs the 3-D check, and this repository already has one: `scene.occlusion`
raycasts real triangles off the USD stage and is what proves A cannot see P. It
was not run here because the pose set was being explored, not decided.

If the mast clears, it is the most attractive option of all: it keeps the camera
at P, needs no new mount, and has an unobstructed view of the whole approach at
2.3–4.7 m.

## What Phase 3 needs next, in order

1. **This decision.** The detector's training data, its evaluation stations and
   the ArUco-plate baseline all depend on where the camera is. Nothing further
   is worth building until the pose is chosen.
2. **A 3-D line-of-sight check on the mast**, via `scene.occlusion`, before the
   choice is made — it is the cheapest of the options if it clears.
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
