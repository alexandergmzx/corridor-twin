# ADR 0025: Join the fleet workspace by symlink and pin; record the corridor domains; tell the pip path true

- Status: Accepted
- Date: 2026-08-11
- Source: `robot-fleet`, `rasptank-ros2`, and `yahboomcar-ros2` layout and
  conventions, verified in code in [`docs/v2-plan.md`](../v2-plan.md)
  F6–F13, F19, F25–F28, F30, F32.
- Extends [ADR 0008](0008-runtime-environment-boundaries.md): the
  ROS/OpenUSD/Isaac runtime boundaries stand; this record adds *where the ROS
  side is also buildable*. Extends
  [ADR 0020](0020-communication-domain-isolation.md)'s domain allocation.

## Context

v2 consumes fleet assets on every axis: the robot twins, `simctl`, the
drive-and-map and Nav2 gate patterns, and the `ground_station` build
environment. v1 ran standalone by design (0008). Two build environments for
one deliverable is sourcing drift waiting to happen, and the fleet's no-copy
asset rule forbids vendoring the twins here.

Verification corrected three assumptions a first draft of this record had
made. The ROS side of this repository is *already* four colcon packages —
membership is a workspace question, not a packaging one. The "pip-only proof
path" was half true: the occlusion/certificate chain really is importable
from `usd-core` + `PyYAML` + stdlib, but `scene.build` unconditionally
imports OpenCV through `scene/marker_assets.py`, satisfied only by the
`--system-site-packages` venv leaking apt's `python3-opencv` — and CI has
exactly one job, which installs full ROS Jazzy; no pip-only CI lane has ever
existed. And the fleet reserves domains by convention, not code: 20 is
hardware (refused in code), 66 the sim default, 68 the replay default, 67/69
scratch by prose — with 70 already used ad hoc in two bench-card rehearsals.

## Decision

1. **Membership by symlink and pin.** `robot-fleet/src/corridor-twin` becomes
   a relative symlink to the existing checkout (the fleet's own MicroROS
   Flow-A precedent), and `fleet.repos` gains a four-line vcstool pin so the
   clean-clone flow materializes the repo for anyone else. The corridor's ROS
   packages join the `ground_station` symlink farm — one symlink per package,
   visible in `git diff`, per that workspace's allow-list rule.
2. **Corridor tools resolve the fleet root the `_layout.py` way.** Env
   override first, then symlink-preserving path walking — never
   `realpath(__file__)`, which escapes the symlink into the real checkout
   and silently breaks every `../yahboomcar-ros2` sibling resolution. Tools
   that reach into the fleet import or mimic the existing resolver contract
   rather than inventing a parallel one.
3. **Both build homes stay; the corridor gate is this repository's.**
   `tools/check_workspace.sh` remains the corridor's own ruff → pytest →
   colcon gate, run from this repo root; a fleet-workspace build of the same
   packages through `ground_station` is fleet CI's business. The two builds
   share sources through symlinks and must not share build directories —
   `build/`, `install/`, `log/` stay local to whichever root ran colcon.
4. **Domain plan: 42/43 stand; 44 is reserved; 70 is dirty.** The committed
   42 (A-plane) / 43 (P-plane) allocation of ADR 0020/0021 is kept — the
   principle worth recording is collision-free allocation, not particular
   digits, and migration would buy symmetry at the cost of an amendment ADR,
   seven-plus file edits, and re-verifying four tests inside the delivery
   window. **44 is reserved for corridor replays**, reserve-only until first
   replay use (a 2026-08-11 cleanliness grep of both repositories found no
   use of 44). **70 is recorded as dirty/unavailable** — used ad hoc in two
   2026-08-09 fleet rehearsals — so nobody allocates it later. Corridor
   tooling never defaults to 0, 20, 66, 67, or 68, and exports its own
   `ROS_DOMAIN_ID` (nothing in the fleet exports it for a caller). The
   allocation is recorded in `robot-fleet/docs/architecture.md` as a D-nn
   entry in the same change that adds the pin.
5. **The pip path is told true and made true.** The claim this record
   preserves is precise: the **occlusion/certificate chain** (`scene.occlusion`
   and everything it imports) stays importable from `usd-core`, `PyYAML`, and
   the standard library alone. `scene.build` additionally needs NumPy and
   OpenCV; those dependencies are **declared** — `python3-opencv` and
   `python3-numpy` in `corridor_scene/package.xml` for the rosdep path, and a
   note beside the pip requirements naming the system OpenCV expectation —
   rather than left to the site-packages leak. OpenCV is deliberately *not*
   added to `requirements.txt`: a pip OpenCV wheel in the ROS-shared venv
   risks the NumPy-ABI conflict the environment discipline exists to prevent.
   A genuinely pip-only CI lane for the certificate chain is future work and
   is not claimed until it exists.
6. **Assets resolve by environment, in both directions.** The corridor arena
   USD is located by a `CORRIDOR_ARENA_USD`-style variable in the manner of
   `YAHBOOM_ARENA_USD` (absolute path or basename); nothing is copied across
   repositories. The robot2 runner needs the matching env hook on the fleet
   side — a rasptank-repo change, since its arena constant is hardcoded
   today.
7. **Ledger boundaries.** Fleet decisions about the corridor (adoption,
   domains, gate sessions) are D-nn rows in `architecture.md` §2 and OI-nn
   rows in §11 — not in `research-plan.md`, whose unit is the gate table and
   the session-results section. Scenario decisions are ADRs here.
   Cross-reference by identifier; no duplication.

## Consequences

- `.github/workflows/ci.yml` here keeps its single rosdep-driven job
  unchanged; the corridor's new packages resolve their dependencies through
  it (`domain_bridge`, `domain_coordinator`, and now OpenCV/NumPy via
  `corridor_scene/package.xml`).
- The corridor arena must be loadable by the fleet Isaac runner for robot2 —
  the first executable task of the v2 plan and the prerequisite for the ADR
  0022 gate. Until the rasptank-side env hook exists, that runner cannot be
  pointed at any non-default arena without a code edit.
- Corridor Isaac sessions contend for the machine-wide single-occupancy slot
  under the fleet's honor-system protocol; there is no code lock to rely on.
- The fleet's own R-06 rejection of per-robot domain isolation is not
  contradicted: that rejection protects the shared-map, one-graph fleet;
  the corridor is a two-actor scenario whose requirement *is* the boundary.
  The distinction is recorded so neither repository's rule erodes the
  other's.

## Alternatives considered

- **Stay standalone, source the fleet install by env var.** Rejected: two
  environments, drift, and every gate script would need a compatibility
  shim.
- **Move the scenario into the fleet repo wholesale.** Rejected: fractures
  the immutable 0001–0019 chain and the repository the task author has
  already reviewed.
- **Vendor the robot twins into this repo.** Rejected: violates the fleet's
  no-copy rule and duplicates verified assets.
- **Migrate the corridor to domains 70/71/72 for symmetry with a fleet
  block.** Rejected: churn without a property gained — 42/43 collide with
  nothing, and 70 turned out to be dirty anyway.
- **Add OpenCV to `requirements.txt` to make `scene.build` pip-pure.**
  Rejected: a pip OpenCV in the `--system-site-packages` venv invites the
  NumPy 1.x/2.x ABI break the environment rules exist to prevent; the honest
  fix is declaration plus documentation, not a riskier install path.
