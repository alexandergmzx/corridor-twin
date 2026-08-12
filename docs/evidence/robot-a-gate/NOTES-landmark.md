# A sees B — first live landmark detection

**2026-08-12 02:06 CST.** Isaac Sim 5.1.0.0, RTX 5070 Ti, ROS 2 Jazzy, scratch
domain 67. Robot A = robot1 (Yahboom twin, ADR 0027), MS200 lidar: 360 beams,
12 Hz declared, 0.12–8.0 m.

## Command

```bash
export CORRIDOR_ARENA_DIR=$PWD/out/small
export CORRIDOR_MANIFEST=$PWD/out/corridor-small.manifest.json
bash tools/corridor_profile_run.sh --robot robot1 --profile nominal_m6_n3 \
  --gated --allow-contract-fail --domain 67
```

Session `20260812-015929-isaac-d67`. Artifact: `gate-robot1-landmark-run.json`.

**Caveat carried into every artifact:** the twin publishes `/scan` at 14.3 Hz
against a declared 12.0, a pre-existing defect of the stock yahboom arena. The
run proceeds under `--allow-contract-fail` with the failure recorded, not
lowered.

## Result

| | measured | authored |
|---|---|---|
| detected | **true** | — |
| first detection | **2.763 m** @ bearing 0.255 rad | — |
| frames to confirm | **3** | 3-of-5 rule |
| closest tracked | **0.309 m** | — |
| confirmed frames | 207 of 3740 scans | — |
| fitted radius, mean | **0.0665 m** | **0.063 m** (+5.5%) |
| fitted radius, range | 0.0379 – 0.0881 m | — |
| fit residual, mean | 0.00934 m | — |

A detects B's post from 2.76 m, confirms in exactly the k-of-n minimum, tracks
it to 0.31 m, and recovers the authored radius to 5.5%.

## Reproduced (n=2)

Second run, session `20260812-030240-isaac-d67`, artifact
`gate-robot1-landmark-run2.json`:

| | run 1 | run 2 |
|---|---|---|
| detected | true | **true** |
| first detection | 2.763 m | **2.409 m** |
| frames to confirm | **3** | **3** |
| closest tracked | 0.309 m | 1.121 m |
| fitted radius, mean | 0.0665 m (+5.5%) | 0.0723 m (+14.8%) |
| confirmed frames | 207 of 3740 | 71 of 3891 |

Detection reproduces, acquires at 2.4-2.8 m, and confirms in exactly the k-of-n
minimum on both runs. The fitted radius is biased HIGH on both, which is the
expected direction for a Kasa fit on a short arc and is why the acceptance
window is +/-40% of the authored radius rather than a tight symmetric band.

The two runs differ in how long the post stayed tracked (207 vs 71 confirmed
frames) because they differ in how close A got and how long it dwelt there --
which is a property of the navigation, not of the detector.

## Why this matters more than the number

Every other measurement of "where is B" in this system passes through the SLAM
map, and on this run the map diverged as usual. **This one does not.** It is a
range and bearing in the laser frame, computed from one scan by fitting a
circle of the manifest's authored radius. It is true whatever the map believes.

## What it is not

Not an arrival mechanism. The arrival gate remains Nav2 `SUCCEEDED` within
0.15 m map-frame, and the demonstration must pass with this detector disabled.
Nothing consumes the detection yet: it is recorded, not acted on. Terminal
docking (one refinement, ever) remains unbuilt.

## Same-run navigation, for context

World-frame delivery from truth: closest approach **0.4038 m** at t+81 s,
`walked_away_m` 1.214. The map-frame nav gate failed as it has all night; that
number is computed in a frame SLAM owns and is not quotable while the map
diverges.
