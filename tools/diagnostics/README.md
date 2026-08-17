# Why A overshoots B — the three scripts that answered it

Offline, bag-only, no GPU. Written 2026-08-13 to answer a question the run
artifacts could not: A reaches the delivery standoff to 3 cm and then drives
out of the corridor.

| script | question |
|---|---|
| `at_closest.py <bag>` | at the moment A was physically at the standoff, what did Nav2 believe, and which half of the goal check failed? |
| `axis.py <bag>` | is the SLAM pose error ALONG the corridor or ACROSS it? |
| `error_vs_time.py <bag>` | does that error creep, or does it jump? |
| `bearing_to_b.py <bag>` | how far off A's nose does B sit while A is close enough to dock? (sized the body-frame cone) |
| `arming_replay.py <bag>` | would the CURRENT arming rules have fired, when, and on WHAT? (found the EastWallStub decoy) |
| `chord_sweep.py <bag>` | can the chord ceiling separate B from that decoy? (answer: no) |
| `run_batch.sh <n> <profile>` | sequential live runs, because single runs have been over-read all week |
| `batch_summary.py [<run-dir>]` | docked on B or the decoy, closest approach, walked away, refinements |

    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
    PYTHONNOUSERSITE=1 python tools/diagnostics/at_closest.py \
      ~/Development/MicroROS/MicroROS-assets/bags/20260813-102705-isaac-d67

Findings: [../../docs/evidence/robot-a-gate/NOTES-why-A-overshoots-B-20260813.md](../../docs/evidence/robot-a-gate/NOTES-why-A-overshoots-B-20260813.md)

**Kept deliberately rough.** These are diagnosis scripts, not gates: they
hardcode the nominal profile's spawn heading and the goal the gate sends. They
are committed because the question will be asked again, not because they are
tools.
