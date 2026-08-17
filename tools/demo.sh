#!/usr/bin/env bash
# The corridor-twin demonstration, in one command per run.
#
#     bash tools/demo.sh deliver     # autonomous delivery, A's plane
#     bash tools/demo.sh enforce     # F3.1 enforcement pass, both planes
#
# THIS IS THE ENTRY POINT. It contains no simulation logic: `deliver` calls
# tools/corridor_profile_run.sh and `enforce` calls tools/run_demo.sh. The two
# names are close enough to confuse -- `run_demo.sh` is the enforcement engine,
# `demo.sh` is the thing you type.
#
# WHY IT EXISTS: THREE DEFAULTS THAT ARE WRONG AND FAIL QUIETLY
# -------------------------------------------------------------
# Every one of these produces a run that starts, finishes, and writes plausible
# artifacts -- for the wrong scenario. None of them errors.
#
#   1. corridor_profile_run.sh defaults --robot to robot2, the twin ADR 0027
#      REJECTED (its odometry published nothing until ~5 m in, on a matcher
#      tuned for a 4x4 m room). A is robot1. We pass --robot robot1 always.
#
#   2. run_demo.sh defaults STAGE to out/corridor.usda and SPEED_MPS to 1.0 --
#      v1's kinematic box at five times A's top measured speed. Its own header
#      says in capitals that a bare run "IS NOT THE v2 DEMONSTRATION". We set
#      the composed arena and 0.22 m/s, the F3.1 environment that every figure
#      in DELIVERY.md came from (docs/evidence/ship-day/NOTES.md).
#
#   3. build_corridor_arena.py defaults --robot to rasptank, so it writes
#      arena_corridor_rasptank_<profile>.usd while the enforcement pass opens
#      arena_corridor_robot1_<profile>.usd -- a file that is then stale or
#      absent. We pass --robot robot1 to the builder too.
#
# The overrides live here rather than in those scripts on purpose: their
# recorded invocations stay reproducible, and this file is the one place that
# says what the v2 demonstration IS.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# A is robot1 (ADR 0027). Never inherited from a caller's default.
ROBOT="robot1"
PROFILE="nominal_m6_n3"
# Kept in step with build_corridor_arena.py's PROFILES tuple; a name outside it
# would compose an arena that does not exist rather than failing here.
KNOWN_PROFILES="nominal_m6_n3 wide_corner_m6_n4_5 uniform_m6_n6"
# The v1 stage. If STAGE ever resolves to this, the run is not the v2 demo.
V1_STAGE="out/corridor.usda"
ISAAC_PYTHON="${ISAAC_PYTHON:-$HOME/isaac/env_isaaclab/bin/python}"
DRY_RUN="${CORRIDOR_DEMO_DRY_RUN:-0}"
AS_RECORDED=0

usage() {
  cat <<'USAGE'
Usage: bash tools/demo.sh <deliver|enforce> [options]

This runs the DEMONSTRATION, and the demonstration is robot1: A is robot1 by
ADR 0027, so the robot is fixed here and there is no --robot option.

For a MEASUREMENT whose subject is the robot -- re-running ADR 0027's gate on
robot2 after a matcher retune, say -- call the runner directly and name it:

    bash tools/corridor_profile_run.sh --robot robot2 --profile <name> --gated

  deliver   A navigates autonomously to B and docks onto contact.
            Governed Nav2 on a live SLAM map, no authored route.
            Runs on A's plane only. Watched by the lens, which refuses
            the run if it cannot see.

  enforce   P measures A's speed from P's own roadside camera, across
            the domain gateway. This is F3.1 -- the pass the speed table
            and the capture come from. Builds the composed arena first
            if it is missing.

Options (both subcommands):
  --profile <name>   nominal_m6_n3 (default) | wide_corner_m6_n4_5 | uniform_m6_n6
  --as-recorded      deliver only: use the SLAM params every committed
                     2026-08-14 artifact used, rather than the canonical
                     fleet params. See the note in the deliver section.
  -h, --help         this text

Anything else is passed through to the underlying runner.

Environment:
  ISAAC_PYTHON=<path>            default ~/isaac/env_isaaclab/bin/python
  CORRIDOR_DEMO_DRY_RUN=1        print the resolved command and exit,
                                 without starting Isaac
USAGE
}

die() { echo "demo.sh: $*" >&2; exit 1; }

require_known_profile() {
  case " $KNOWN_PROFILES " in
    *" $PROFILE "*) ;;
    *) die "unknown profile '$PROFILE'. Known: $KNOWN_PROFILES" ;;
  esac
}

require_isaac_python() {
  [ -x "$ISAAC_PYTHON" ] \
    || die "Isaac Python not found at $ISAAC_PYTHON (set ISAAC_PYTHON=<path>)"
}

# --- deliver ---------------------------------------------------------------
#
# corridor_profile_run.sh already acquires and releases the machine-wide Isaac
# lock itself, so this path must NOT take it -- that would deadlock against the
# child.
#
# --allow-contract-fail is required on this host, and it is an override of a
# real defect rather than a lowered bar: robot1's twin publishes /scan at
# 13.4-15.1 Hz against a declared 12.0 (one run at 8.6). Without the flag the
# runner classifies that as INFRASTRUCTURE and refuses to start. With it the
# check still runs, still fails, and every artifact carries
# "PRECONDITION FAILED (recorded, overridden)".
#
# --corridor-slam is deliberately NOT passed. It does not turn SLAM on or off;
# it swaps which params file the corridor's own SLAM uses, selecting
# config/robot1/slam_robot1_corridor.yaml -- whose first line reads "NOT IN
# USE, kept as a record" and whose header says it "measured no better and was
# observed worse". The canonical fleet params are the better run. --as-recorded
# opts back in, because that is the arm the committed artifacts used.
cmd_deliver() {
  require_known_profile
  local -a cmd=(
    bash "$REPO/tools/corridor_profile_run.sh"
    --robot "$ROBOT"
    --profile "$PROFILE"
    --allow-contract-fail
  )
  if [ "$AS_RECORDED" = 1 ]; then
    cmd+=(--corridor-slam)
    echo "  SLAM params: corridor (--as-recorded; the 2026-08-14 arm)"
  else
    echo "  SLAM params: fleet canonical (loop closure ON)"
  fi
  cmd+=("$@")

  echo "  robot:   $ROBOT (ADR 0027)"
  echo "  profile: $PROFILE"
  if [ "$DRY_RUN" = 1 ]; then
    printf 'DRY RUN: %s\n' "${cmd[*]}"
    return 0
  fi
  "${cmd[@]}"
}

# --- enforce ---------------------------------------------------------------
#
# run_demo.sh starts Isaac and does NOT take the machine-wide lock, unlike
# corridor_profile_run.sh. Two Isaac instances can take down the whole host, so
# this path acquires it here and releases it on every exit.
cmd_enforce() {
  require_known_profile

  local stage="out/arena_corridor_${ROBOT}_${PROFILE}.usd"
  local abs_stage="$REPO/$stage"

  # The guard that matters: the v1 stage is what run_demo.sh falls back to, and
  # a run against it looks entirely normal while demonstrating the wrong thing.
  [ "$stage" = "$V1_STAGE" ] \
    && die "refusing: STAGE resolved to the v1 stage ($V1_STAGE), not a composed arena"

  if [ ! -f "$abs_stage" ]; then
    echo "  arena missing, composing it: $stage"
    local -a build=(
      "$ISAAC_PYTHON" "$REPO/tools/build_corridor_arena.py"
      --robot "$ROBOT" --profile "$PROFILE"
    )
    # A dry run must be hermetic: it composes and prints, and touches neither
    # Isaac nor the arena. That is what lets CI -- which has no GPU, no Isaac
    # interpreter and no out/ -- test the composition at all.
    if [ "$DRY_RUN" = 1 ]; then
      printf 'DRY RUN would build: %s\n' "${build[*]}"
    else
      require_isaac_python
      ( cd "$REPO" && "${build[@]}" ) || die "arena build failed for $PROFILE"
      [ -f "$abs_stage" ] || die "arena build reported success but $stage is absent"
    fi
  else
    echo "  arena: $stage"
  fi

  # Bulk output under out/ -- never docs/evidence/, which holds curated,
  # committed artifacts that tools must not overwrite by default.
  local evidence_dir="out/evidence/demo/${PROFILE}-enforce"

  local -a cmd=(
    env
    "STAGE=$abs_stage"
    "MANIFEST=$REPO/out/corridor.manifest.json"
    "CORRIDOR_PROFILE=$PROFILE"
    "ROBOT_PRIM=/World/Robot"
    "DEACTIVATE_PHYSICS=1"
    "SPEED_MPS=0.22"
    "UPDATES=3000"
    "EVIDENCE_DIR=$REPO/$evidence_dir"
    bash "$REPO/tools/run_demo.sh" --headless --no-rviz --record
  )
  cmd+=("$@")

  echo "  speed:    0.22 m/s (A's measured profile, not v1's 1.0)"
  echo "  evidence: $evidence_dir"
  if [ "$DRY_RUN" = 1 ]; then
    printf 'DRY RUN: %s\n' "${cmd[*]}"
    return 0
  fi

  # shellcheck disable=SC1091
  source "$REPO/tools/isaac_lock.sh"
  trap 'isaac_lock_release' EXIT INT TERM
  isaac_lock_acquire "demo.sh enforce $PROFILE" \
    || die "the machine-wide Isaac lock is held by another session"
  "${cmd[@]}"
}

subcommand="${1:-}"
[ $# -gt 0 ] && shift || true
case "$subcommand" in
  -h|--help) usage; exit 0 ;;
  deliver|enforce) ;;
  "") usage >&2; echo >&2; echo "demo.sh: pick a subcommand." >&2; exit 1 ;;
  *) usage >&2; echo >&2; echo "demo.sh: unknown subcommand '$subcommand'." >&2; exit 1 ;;
esac

passthrough=()
while [ $# -gt 0 ]; do
  case "$1" in
    --profile) PROFILE="${2:?--profile needs a name}"; shift 2 ;;
    --as-recorded) AS_RECORDED=1; shift ;;
    -h|--help) usage; exit 0 ;;
    # Passthrough forwards unknown flags to the runner, and the runner's parser
    # is last-wins -- so `--robot robot2` here would emit
    # "--robot robot1 ... --robot robot2" and quietly run robot2 while the line
    # above printed "robot: robot1". A banner contradicting the run is the exact
    # failure this script exists to prevent, so the flag is refused, not honoured.
    --robot)
      die "--robot is not an option here: the demonstration is robot1 (ADR 0027).
  For a gate re-run on another twin, name it on the runner instead:
    bash tools/corridor_profile_run.sh --robot ${2:-robot2} --profile $PROFILE --gated" ;;
    *) passthrough+=("$1"); shift ;;
  esac
done

echo "corridor-twin demo: $subcommand"
cmd_"$subcommand" ${passthrough[@]+"${passthrough[@]}"}
