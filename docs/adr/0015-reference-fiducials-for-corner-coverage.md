# ADR 0015: Restore corner enforcement coverage with reference fiducials

- Status: Accepted
- Date: 2026-07-27
- Extends: [ADR 0013](0013-size-fiducials-from-delivered-camera.md), which
  remains accepted and is not superseded.

## Context

Enforcement gates sit every 2 m along the corridor at stations 2, 4, 6, 8 and
10. Past camera x ≈ 7.5 the observer stopped producing estimates entirely, so
gates 8.0 and 10.0 never carried a measurement and the tightest rule could not
be exercised from camera evidence at all.

The cause is angular, not photographic. Near the corner the corridor is `n`
wide, so a wall marker sits about `n/2` from the centreline; on the nominal
profile that is 1.5 m. A target 2 m ahead and 1.5 m off-axis subtends roughly
37°, which is outside the 75° camera's 37.5° half-FOV. Larger codes, more
pixels or a better renderer cannot fix that — the markers are not blurry, they
are out of frame. Any fix has to put targets **three to eight metres ahead** of
the camera, which means surfaces beyond the corridor's end.

Two candidate surfaces exist in the authored scene: the north wall, which
already extends east past the corner to cap the next street, and the east
building face at `x = 18`. They are perpendicular.

## Decision

Add a second class of surveyed fiducial, **reference plates**, on those two
surfaces. They are pose evidence only and are never enforcement stations.

Five plates, with every parameter measured rather than chosen:

| Surface | `along_m` | Height | Size |
|---|---:|---:|---:|
| north wall | 13.0 | 0.9 m | 0.60 m |
| north wall | 15.0 | 2.0 m | 0.60 m |
| north wall | 17.0 | 3.1 m | 0.60 m |
| east face | 1.8 | 2.6 m | 1.00 m |
| east face | 0.0 | 1.0 m | 1.00 m |

Four properties are load-bearing.

**The heights are staggered because level plates destroy each other.** Plates
receding along a wall at one height telescope in the image into a contiguous
strip, where each nearer plate paints over the farther one's ArUco quiet zone
and only one of them decodes. Staggering the heights is what makes them decode
together. This was measured, after a first design that failed with 0.32 m
station error for what was initially misdiagnosed as occlusion.

**Smaller plates win.** An earlier 0.80 m / 1.20 m pairing performed worse than
the accepted 0.60 m / 1.00 m: larger targets collide in-image at close range
and clip at the frame edge. Bigger is not better once the constraint is angular.

**The two host planes must be perpendicular.** A frame that sees only one plane
yields coplanar correspondences and reintroduces the planar-PnP ambiguity that
ADR 0013's marker-count rule was meant to remove. Combining a north-wall plate
with an east-face plate keeps the correspondence set rank 3. The estimator
enforces this directly by rejecting any set whose centred correspondence matrix
has rank below 3, rather than inferring safety from how many markers were seen.

**Roles are explicit in the manifest.** Every marker carries `role: gate` or
`role: reference`. Gates define enforcement stations; references never do. The
split exists so a reference plate cannot silently become a phantom gate at
`x = 13` where no policy was ever authored.

## Consequences

- Gates 8.0 and 10.0 produce measurements, so the strict corner rule became
  exercisable. Confirmed on rendered Isaac pixels, not only synthetically:
  [corner frame](../evidence/live-demo/corner-references.png) and
  [live evidence](../evidence/live-demo/NOTES.md).
- Coverage extends to roughly x = 10.8, which brackets the last gate at 10.0.
  Past that only the two coplanar east-face plates remain in view; the
  estimator correctly returns nothing there rather than emitting an ambiguous
  pose, and a regression drives that frame on all three profiles.
- Reference placement is profile-dependent and validated per resolved profile.
  The east face spans `y` only up to the north wall at `m/2`, so it shortens
  with a narrower entry width; the plate backing, `9/7` the code size, is what
  overhangs first.
- Reference placement bounds the supported `(m, n)` envelope from both sides,
  and both bounds are checked per resolved profile:

  | Bound | Rule | Why |
  |---|---|---|
  | Entry width floor | `m >= 4.886`, declared as `m >= 5.0` | The east face spans `y` only up to `m/2`, so a narrow corridor shortens it until the upper plate overhangs |
  | Corner mass clearance | the band must hold the plate | The corner mass reaches north to `m/2 - n`, so the visible strip of east face is `n` tall wherever it sits. A profile whose band is shorter than a plate is refused; `m = 6.0, n = 1.0` is |

  > **Correction, 2026-07-27.** This section first justified that refusal by
  > saying the geometry "leaves no visible east face for a reference to sit
  > on". That is not what the geometry says, and review measured it: the
  > usable band runs from `m/2 - n` up to `m/2`, so its height is `n` and does
  > not depend on `m` at all. At `m = 8.0, n = 3.0` the band is 2.985 m tall —
  > the same height as on the default profile — and the *upper* east plate at
  > `along_m: 1.8` sits inside it and validates. Only the lower plate fails,
  > because `along_m: 0.75` is an absolute coordinate while the band's
  > position shifts with `m/2 - n`.
  >
  > So `0.349` was a real bound on the configuration as authored, and the
  > validator that enforces it is correct — a plate half inside a building is
  > exactly the defect R17 was. But it was a property of one hard-coded plate
  > coordinate, not of the scenario.
  >
  > **Second correction, same day.** The paragraph above went on to defer the
  > fix, "because moving a plate changes the measured accuracy figures and needs
  > re-measurement". That ground was not checked before it was written, and it
  > does not hold. All three configured profiles have entry width 6.0, so their
  > band floors sit below the configured coordinate — two of them negative — and
  > clamping placement to the floor does not bind on any of them. Their surveys
  > come out identical, so nothing measured moved and no re-measurement was
  > owed.
  >
  > Placement now clamps to the band floor (`7a5980a`). `m = 8.0` through
  > `m = 10.0` at `n = 3.0` build, and the newly reachable geometry was measured
  > rather than assumed: at `m = 8.0, n = 3.0` the synthetic run accepts every
  > frame, measures all four gates at 0.6, 1.0 and 1.8 m/s, and holds
  > gate-derived speed error between 0.0123 and 0.0247 m/s — comparable to the
  > default profile. The bound that remains is the honest one: a band shorter
  > than the plate is still refused.
  >
  > The decision to add reference fiducials and to validate their placement is
  > unchanged by either correction.

- The lower east-face plate must clear the corner mass, not merely sit on the
  wall. It was originally centred at `along_m: 0.0`, and on the default profile
  the corner mass's north edge is at exactly `y = 0.0` — so half the plate was
  behind the corner building. `SyntheticCamera` projects without raycasting and
  rendered it whole, which is why synthetic runs looked clean while a
  composed-stage raycast flagged 59 of 164 accepted frames on the approach.

  Moved to `along_m: 0.75` at 0.60 m, which measured **strictly better** than
  the occluded original on every metric: zero occluded frames, and lower worst
  station error (0.0817 m against 0.0845) and worst gate-derived speed error
  (0.0530 m/s against 0.0559) across all three profiles.

  Note for anyone re-measuring this: an earlier attempt to relocate the plate
  appeared to cost gate 8.0 at 0.6 m/s. That was not a placement problem. It was
  the continuity guard treating noise-level backward station steps as pose
  jumps, since at 0.6 m/s the per-frame advance is below the station noise.
  With that corrected, every occlusion-free placement holds all four gates on
  all three profiles.

## Alternatives rejected

- **Bigger or higher-resolution markers.** Does not address an out-of-frame
  target, and costs VRAM and visual credibility.
- **A wider camera FOV.** Changes the delivered sensor contract that the
  observer, the qualification gates and ADR 0009 are all written against.
- **A second camera at the corner.** Violates the one-camera invariant outright.
- **Plates on one plane only.** Cheaper to author, but coplanar correspondences
  revive the pose ambiguity ADR 0013 exists to prevent.
