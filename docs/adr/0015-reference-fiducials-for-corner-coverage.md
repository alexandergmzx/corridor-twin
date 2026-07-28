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
- The declared support floor is `m >= 5.0`. The configured plates admit any
  entry width down to `m = 4.886`.
- **Open:** on `nominal_m6_n3` the corner mass's north edge sits at exactly
  `y = 0.0`, which is `m/2 - n`, and marker 84's backing spans `y ∈ (-0.643,
  +0.643)`. Half that plate is behind the corner building. The projection-only
  synthetic camera renders it whole, so the defect is invisible there; a
  composed-stage raycast flags 59 of 164 accepted frames on the approach. The
  live run measured all four gates regardless, so the cost is one of five
  reference plates rather than corner coverage. Tracked as R17.

## Alternatives rejected

- **Bigger or higher-resolution markers.** Does not address an out-of-frame
  target, and costs VRAM and visual credibility.
- **A wider camera FOV.** Changes the delivered sensor contract that the
  observer, the qualification gates and ADR 0009 are all written against.
- **A second camera at the corner.** Violates the one-camera invariant outright.
- **Plates on one plane only.** Cheaper to author, but coplanar correspondences
  revive the pose ambiguity ADR 0013 exists to prevent.
