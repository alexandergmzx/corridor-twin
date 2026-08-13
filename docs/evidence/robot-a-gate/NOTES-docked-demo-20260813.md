# The docked demo run: every derivation fired, and docking never armed

**2026-08-13, run `20260813-035938`.** `nominal_m6_n3`, **docking ON**,
reported not gated, lens up. The first live exercise of ADR 0031's contact
semantics and of the containment re-derived against the merged B — none of
which had run before this.

```bash
bash tools/corridor_profile_run.sh --profile nominal_m6_n3 --robot robot1 \
  --allow-contract-fail --no-rviz
```

## The derivations, live

```
dock: armed, expecting a landmark of radius 0.12 m
dock: containment -- route 5.750 m, window 0.900 m,
      arm after 4.850 m of A's own travel, detection within 76 deg of the goal
dock: final approach 0.470 m from B's centre, derived --
      the governor's floor and geometric contact, larger wins
```

All three are computed, not written down: the radius from the manifest, the
window from the route, the cone from `atan2(standoff, tolerance)`, and the
0.470 m from the governor's 0.35 m laser stop plus B's radius. ADR 0031's
arithmetic holds in the running system.

## Docking never armed, and the reason is the interesting one

`state: TRANSIT`, `refinements: 0`, `landmark_map_frame: null`. The rejection
counts say exactly why:

| containment test | rejections |
|---|---|
| out of laser range | 5307 |
| **too far from the goal in the map frame** | **2812** |
| too early in the route | 0 |
| detection is not where the goal is | 0 |

Travel passed. The **bearing cone passed** — the widening from 60° to 76° was
necessary and is not what blocked this. What blocked it was the **map-frame
proximity test**.

That is precisely the failure ADR 0029 named and removed from the arming
condition: *"the first version armed on the map-frame distance from the robot
to the nominal goal, which is precisely the number this whole mechanism exists
to not trust: on a run where A came within 0.49 m of B physically, its drifted
map pose never came within 3 m of the map goal."* Arming was moved to laser
range for that reason.

The containment added on 2026-08-12 (`e17f83d`) then put a map-frame test back
— for a good reason, the spawn phantom, which range alone could not reject —
and it now refuses for the original reason. A drifted 2812 times' worth of
"too far from the goal", while its own laser was measuring B correctly: earlier
tonight the lens showed the detector confirming B at 0.576 m with a fitted
radius of 0.1133 m against an authored 0.12.

**This is a genuine design tension, not a bug to patch at 04:17.** Two guards
protect against opposite failures:

- laser-only arming trusts a measurement the map cannot corrupt, and admitted a
  phantom near spawn that cost a whole mission;
- map-frame containment rejects that phantom, and blocks the real thing exactly
  when the map is bad — which is the condition docking exists for.

Resolving it means choosing which risk to carry, or finding a third
discriminator that is neither the map nor bare range — A's own travelled
distance is already one of the three, and it passed here. **Parked for
Alexander.**

## And the run nearly hit the cap

596 s against a 600 s watchdog, from a bring-up of 99 s. With `--dock` the nav
gate spends its timeouts in series — the dock refinement loop for `--timeout`,
*then* the result wait for `--timeout` again — so a docked run costs roughly
twice a transit run and, on this evidence, is within 4 seconds of being killed
by its own cap.

That is the unbounded-nav-gate shape recorded in this session's runner work: a
`--timeout 200` that does not bound the gate to 200 s. It did not need fixing
for the gated runs, which are dock-off. It needs fixing before a docked run is
a demo candidate. **Parked with that reason.**

## What the run does say

`travelled 7.721 m`, no nav failure, and the transit itself is unremarkable —
A leaves cleanly and drives its seven metres, as it has all night. The
demonstration cannot yet be run docked, and the reason is documented above
rather than discovered on the day.
