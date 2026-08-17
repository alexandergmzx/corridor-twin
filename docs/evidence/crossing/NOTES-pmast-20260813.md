# The camera is on P's mast, and the isolation claim still holds

**2026-08-13.** First crossing session since the single render product moved
from A's v1 mount to `/World/Actors/PCameraMast/PCam` — the 1.50 m mast at P's
own position, ratified 2026-08-12 and authored by ADR 0031's session. A is
camera-less.

```bash
bash tools/crossing_session.sh --label 640x360-pmast --seconds 45
```

## The requirement gate: GREEN, mutation RED

| | observed in P's plane | unexpected | missing | `/clock` | verdict |
|---|---|---|---|---|---|
| certificate | `/clock`, `/p_cam/camera_info`, `/p_cam/image_raw` | **none** | **none** | 723 msgs, advancing | **GREEN** |
| mutation control | the same **plus `/test/ground_truth/speed`** | `/test/ground_truth/speed` | none | 814 msgs, advancing | **RED** |

P's observed graph equals the declared allowlist **exactly**, and deliberately
relaying one A-plane topic turns it red. That is ADR 0026's requirement gate,
re-verified in the new topology. **Nothing on the ROS side had to change** —
topics, `frame_id`, allowlist and QoS were already `/p_cam/*`, migrated ahead
of the geometry — so this run tests that the claim survives the prim move, and
it does.

Producer gate: **14.976 Hz rendered against 15.0 declared, ratio 0.9984.**

## The crossing ratio is close to its floor, and it moves

Two sessions tonight, same label, same settings:

| run | image ratio | camera_info ratio | floor | |
|---|---|---|---|---|
| ADR 0026 (2026-08-11) | 0.954 | 0.998 | 0.95 | pass |
| tonight, first | **0.9745** | 0.9966 | 0.95 | pass |
| tonight, second | **0.9265** | 0.994 | 0.95 | **FAIL** |

The small stream crosses at 0.994–0.998 every time; the large one does not.
The bridge's own attribution — *size-dependent transport loss* — is consistent
across all three, and the image ratio sits **on** its floor rather than
comfortably above it: 0.954, 0.9745, 0.9265 across three runs of the same
configuration.

**That is reported, not tuned.** No threshold was moved and no setting was
changed between the two runs tonight; the second differs from the first only in
the drive speed (see below), which is not a transport parameter. Three points
around a floor is a distribution worth naming before ADR 0024's resolution
decision leans on it — and it says the 640×360 crossing is not the comfortable
margin a single 0.954 suggested.

## A literal that stopped being true when the scenario scaled

The first session tonight went **RED on `clock_advancing`** with
`unexpected=[]` and `missing=[]` — the isolation claim passing while the
certificate failed.

`/clock` had delivered **zero messages** into P's plane, because the producer
was already dead when the certificate ran. The adapter stops when A reaches the
end of the route (`isaac_5_1_ros_camera.py:366`), so the drive speed decides how
long it lives. `DRIVE_SPEED=0.35` was chosen for the **authored 24.601 m**
route, where it lasts ~70 s. ADR 0030 scaled the scenario to 0.30 and the route
became 7.38 m, so the same speed ran out after **21 s** — measured,
`updates_completed: 1267` against a 15000 cap.

The constant's own comment records this exact lesson from the 1.0 m/s era:
*"finishes the authored route in ~24 s, which is shorter than the capture
window."* It was fixed once with a bigger number, and the bigger number went
stale the same way.

It is now **derived** — `route / (capture + 150 s)`, read from the manifest —
so the producer outlives the capture, the certificate and the mutation control
by construction rather than by a literal that happens to fit the current scale.

## A defect found and not fixed, recorded

The producer gate in the second run reported `updates_completed: 1267` and
`adapter_sim_span_s: 21.1` — **the first run's numbers**. The crossing
measurement reads `drive-schedule-<label>.json` before the current adapter has
written it, so a re-run under the same label scores the previous run's
schedule. The final adapter line for that session reads 11716 updates.

It did not affect the crossing ratio, which is measured live on both planes,
and the producer ratio it did affect (0.9984) is the same on both runs. Left as
a finding rather than fixed at 02:45; it is the same stale-artifact shape as
the `find -newer run.json` defect from 2026-08-12.
