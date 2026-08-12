#!/usr/bin/env bash
# One corridor profile, end to end: twin up, drive-and-map gate, one Nav2 goal.
#
#     bash tools/corridor_profile_run.sh --profile nominal_m6_n3 --gated
#     bash tools/corridor_profile_run.sh --profile uniform_m6_n6
#
# v2 plan T3.3. Each profile gets its OWN Isaac session, started and torn down
# here, because the arena is chosen at twin start: switching profiles means
# restarting the simulator, and the occupancy rule means never overlapping two.
#
# Environment is exported HERE (fleet F9): simctl does not export ROS_DOMAIN_ID
# back to its caller, so the gates that run afterwards in this shell would
# otherwise measure domain 0 and report a dead robot. RASPTANK_ARENA_USD is the
# other half -- without it the runner opens the RaspTank's own 4x4 room and the
# whole run measures the wrong scene while passing.
#
# --gated marks a profile whose result is a GATE. Without it the profile is
# REPORTED: its failures are findings, the artifacts are kept identically, and
# this script still exits 0. ADR 0022 gates nominal_m6_n3 and
# wide_corner_m6_n4_5; uniform_m6_n6 is the degeneracy study and is expected to
# struggle, which is a result rather than a regression.
set -euo pipefail

REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$(cd -- "$REPO/.." && pwd)"
FLEET="$(cd -- "$SRC/.." && pwd)"
SIMCTL="$SRC/yahboomcar-ros2/tools/simctl"
CONTRACT_ROBOT2="$SRC/rasptank-ros2/tools/check_rasptank_contract.py"
CONTRACT_ROBOT1="$SRC/yahboomcar-ros2/tools/check_isaac_contract.py"
WS_SETUP="$FLEET/ground_station/install/setup.bash"

PROFILE=""
ROBOT="${CORRIDOR_RUN_ROBOT:-robot2}"
DOMAIN="${CORRIDOR_RUN_DOMAIN:-67}"
GATED=""
ALLOW_CONTRACT_FAIL=0
# Empty means "derive from NAV_TIMEOUT". The recorder now measures the transit
# rather than a fixed bench window, so its length is a property of how long the
# transit is allowed to take, not an independent number that can silently
# truncate a slow delivery.
GATE_SECONDS=""
# RViz ON by default: these runs are watched, and the viewport is how a
# divergence gets noticed at all -- the ghosting that started this whole
# sequence was seen there first. --no-rviz for a genuinely unattended run.
RVIZ_FLAG=""
# HARD wall-clock cap on the whole session, enforced by a watchdog below.
#
# Not a timeout on any one step -- a ceiling on the run existing at all. Runs
# have hung holding the GPU and the machine-wide Isaac lock, which blocks every
# other session, and a hung run is worth nothing anyway.
#
# 420 s, and the number is measured rather than chosen. 300 was tried first and
# is too tight: bring-up alone costs 140-200 s on this box -- Isaac's load, the
# contract measurement, and Nav2's lifecycle activation -- so a 300 s cap left
# a run with no window to navigate in and the watchdog killed it mid-bring-up.
#
# 420 keeps the property that matters (a session cannot hang holding the GPU and
# the machine-wide lock) while leaving ~220 s of transit, which is ample: A's
# closest approach to B was measured at t+60 s, t+68 s and t+119.8 s on the runs
# that reached it, and everything after that was A driving back to spawn.
SIM_MAX_S=420

# The nav window is DERIVED to fit inside the cap, never set past it: a nav
# timeout longer than the session cap is a promise the watchdog will break.
NAV_TIMEOUT=""

while [ $# -gt 0 ]; do
  case "$1" in
    --profile) PROFILE="$2"; shift 2 ;;
    --robot) ROBOT="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --gated) GATED="--gated"; shift ;;
    --allow-contract-fail) ALLOW_CONTRACT_FAIL=1; shift ;;
    --gate-seconds) GATE_SECONDS="$2"; shift 2 ;;
    --nav-timeout) NAV_TIMEOUT="$2"; shift 2 ;;
    --sim-max-s) SIM_MAX_S="$2"; shift 2 ;;
    --rviz) RVIZ_FLAG=""; shift ;;
    --no-rviz) RVIZ_FLAG="--no-rviz"; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[ -n "$PROFILE" ] || { echo "--profile is required" >&2; exit 2; }

case "$DOMAIN" in
  20) echo "REFUSED: domain 20 is the REAL CAR (fleet D-09)." >&2; exit 2 ;;
  42|43) echo "REFUSED: domain $DOMAIN is a corridor demonstration plane (D-20)." >&2; exit 2 ;;
  44) echo "REFUSED: domain 44 is reserved for corridor replays (D-20)." >&2; exit 2 ;;
  66) echo "REFUSED: domain 66 is the fleet's standing sim domain." >&2; exit 2 ;;
  68) echo "REFUSED: domain 68 is marked in use." >&2; exit 2 ;;
  70) echo "REFUSED: domain 70 is dirty/unavailable (D-20)." >&2; exit 2 ;;
esac

ARENA_DIR="${CORRIDOR_ARENA_DIR:-$REPO/out}"
MANIFEST="${CORRIDOR_MANIFEST:-$REPO/out/corridor.manifest.json}"
ARENA="$ARENA_DIR/arena_corridor_${ROBOT}_${PROFILE}.usd"
[ -f "$ARENA" ] || { echo "arena missing: $ARENA" >&2; exit 2; }
if [ "$ROBOT" = robot1 ]; then CONTRACT="$CONTRACT_ROBOT1"; else CONTRACT="$CONTRACT_ROBOT2"; fi
for path in "$SIMCTL" "$CONTRACT" "$WS_SETUP"; do
  [ -e "$path" ] || { echo "fleet layout incomplete, missing: $path" >&2; exit 2; }
done

EVIDENCE="$REPO/out/evidence/robot-a-gate"
# robot1's runner reads the SAME env var name as robot2's -- YAHBOOM_ARENA_USD
# for sim_runner.py:41-46, RASPTANK_ARENA_USD for the rasptank runner. Both are
# exported; each runner reads only its own, so this is harmless and keeps one
# code path (fleet F9: simctl exports neither back to us).
export YAHBOOM_ARENA_USD="$ARENA"
mkdir -p "$EVIDENCE"

export ROS_DOMAIN_ID="$DOMAIN"
export RASPTANK_ARENA_USD="$ARENA"
export PYTHONNOUSERSITE=1

echo "=== corridor profile run: $PROFILE ${GATED:+(GATED)}${GATED:-(reported only)} ==="
echo "  domain : $ROS_DOMAIN_ID"
echo "  robot  : $ROBOT"
echo "  arena  : $ARENA"

occupants() {
  local ancestry=" $$ " walk=$PPID
  while [ -n "$walk" ] && [ "$walk" -gt 1 ] 2>/dev/null; do
    ancestry="$ancestry$walk "
    walk=$(ps -o ppid= -p "$walk" 2>/dev/null | tr -d ' ')
  done
  pgrep -af 'rasptank_twin_runner\.py|isaac_5_1_ros_camera|isaac-sim|/kit/kit' 2>/dev/null \
    | while read -r found rest; do
        case "$ancestry" in *" $found "*) continue ;; esac
        printf '%s %s\n' "$found" "$rest"
      done
}
if [ -n "$(occupants)" ]; then
  echo "REFUSED: an Isaac-shaped session is already running:" >&2
  occupants >&2
  exit 2
fi

# Machine-wide single-occupancy. The occupancy scan above only sees THIS
# machine's process list at one instant; the lock is what serialises sessions
# that start seconds apart. Exit 3 (infrastructure), never a robot result.
# shellcheck disable=SC1091
source "$REPO/tools/isaac_lock.sh"
isaac_lock_acquire "corridor-profile-run $PROFILE (domain $DOMAIN)" || exit 3

set +u
# shellcheck disable=SC1090,SC1091
source "$WS_SETUP"
set -u

nav_pid=""
recorder_pid=""
watchdog_pid=""
WATCHDOG_FLAG="$(mktemp -u)"
stopped=0
teardown() {
  [ "$stopped" = 1 ] && return 0
  stopped=1
  echo "=== stopping $PROFILE on domain $DOMAIN ==="
  [ -n "$nav_pid" ] && kill -TERM "$nav_pid" 2>/dev/null || true
  [ -n "$recorder_pid" ] && kill -TERM "$recorder_pid" 2>/dev/null || true
  [ -n "$watchdog_pid" ] && kill -TERM "$watchdog_pid" 2>/dev/null || true
  "$SIMCTL" stop --domain "$DOMAIN" || true
  sleep 3
  if [ -n "$(occupants)" ]; then
    echo "!! SESSION NOT DEAD:" >&2; occupants >&2; return 1
  fi
  echo "  verified dead"
  isaac_lock_release
}
trap 'teardown || true' EXIT INT TERM

# INFRASTRUCTURE failures below exit 3, distinct from a red gate (exit 1). A
# session that never came up is a rerun, not a result about the robot.
# Marker for "which artifacts belong to THIS run".
SESSION_MARKER=$(mktemp)

# The cap covers the WHOLE session, Isaac's load included. Loading holds the GPU
# and the machine-wide lock exactly as navigating does, and a hang during load
# is the same problem for every other session on this box. Arming after bring-up
# would have made the real wall time cap + load, which is not what a cap means.
#
# $$ inside the subshell is THIS script's pid, not the subshell's -- bash keeps
# it across subshells -- so the watchdog signals the run and its EXIT trap tears
# the session down on the normal path. The flag file survives the signal so the
# exit code can say INFRASTRUCTURE rather than report a killed run as a robot
# failure.
( sleep "$SIM_MAX_S"
  : > "$WATCHDOG_FLAG"
  echo "" >&2
  echo "**WATCHDOG: session exceeded the ${SIM_MAX_S}s cap -- tearing down**" >&2
  kill -TERM $$ 2>/dev/null ) &
watchdog_pid=$!
SESSION_START_S=$(date +%s)
echo "  watchdog armed: ${SIM_MAX_S}s cap covers bring-up AND the transit"

echo "=== simctl start ==="
# --no-patrol is NOT optional. simctl's step 7 launches sim_patrol, which drives
# 1.0 m legs at 0.18 m/s on /cmd_vel_raw for the life of the session. Every
# corridor run before 2026-08-11 carried it, so Nav2's controller and the
# patrol commanded the same topic simultaneously -- the "square patrol" the
# robot was observed doing. The mission's motion policy is one source: Nav2.
#
# RViz was briefly removed to test a starvation hypothesis. That hypothesis is
# WITHDRAWN: the EKF gaps it rested on were an artifact of this repo's own
# recorder timing itself on the wall clock while blocking on a growing /map
# (2727c0c). Measured from the bag, the EKF's worst gap was 0.398 s with none
# over threshold. RViz is back on by default -- the viewport is how the
# ghosting was noticed in the first place.
"$SIMCTL" start --robot "$ROBOT" --backend isaac --domain "$DOMAIN" \
  --no-patrol $RVIZ_FLAG || {
  echo "**INFRASTRUCTURE: simctl start failed for $PROFILE**" >&2; exit 3; }

# Contract numbers are PER-ROBOT and do not transfer. robot2 is checked with
# --imu-hz 60; robot1's checker carries its own WANT_HZ (scan 12 / odom_raw 11
# / imu 25, check_isaac_contract.py:51) and takes no rate flags at all.
# The two checkers do not share a CLI. robot2's takes --imu-hz and --json;
# robot1's takes neither -- its flags are only --seconds/--speed/--turn/--domain
# (check_isaac_contract.py:54-58) and it prints a human table, so its artifact
# is text and its verdict is the exit code.
if [ "$ROBOT" = robot1 ]; then
  CONTRACT_ARGS=(--domain "$DOMAIN")
  CONTRACT_OUT="$EVIDENCE/contract-$ROBOT-$PROFILE.txt"
else
  CONTRACT_ARGS=(--imu-hz 60 --json)
  CONTRACT_OUT="$EVIDENCE/contract-$ROBOT-$PROFILE.json"
fi
echo "=== precondition: $ROBOT contract (${CONTRACT_ARGS[*]}) ==="
# stdout only into the JSON: the checker appends a human summary and its
# FAIL lines after the document, which made every artifact unparseable exactly
# when it mattered most -- on the failures.
if ! python3 "$CONTRACT" "${CONTRACT_ARGS[@]}" >"$CONTRACT_OUT" \
     2>"$EVIDENCE/contract-$ROBOT-$PROFILE.err"; then
  echo "**INFRASTRUCTURE: contract check failed for $ROBOT/$PROFILE; twin not fit to gate**" >&2
  sed 's/^/    /' "$CONTRACT_OUT" 2>/dev/null | tail -12 >&2
  if [ "$ALLOW_CONTRACT_FAIL" = 1 ]; then
    # Deliberate, visible override -- NOT a lowered threshold. The check still
    # ran, still failed, and its artifact is kept unchanged; what this does is
    # let the run proceed so navigation evidence exists at all, with the defect
    # stamped into every artifact it produces. Used when the precondition fails
    # for a reason outside the run under test: robot1's twin publishes /scan at
    # ~14.3 Hz against a declared 12.0 in the STOCK yahboom arena too, so
    # blocking on it would forfeit the night to a pre-existing twin defect.
    CONTRACT_CAVEAT="PRECONDITION FAILED (recorded, overridden): see $(basename "$CONTRACT_OUT")"
    echo "  **proceeding under --allow-contract-fail; every artifact carries the caveat**" >&2
  else
    exit 3
  fi
else
  CONTRACT_CAVEAT=""
fi
[ -z "${CONTRACT_CAVEAT:-}" ] && echo "  contract PASS -> $CONTRACT_OUT"

status=0

# NO WARM-UP DRIVE. The drive-and-map gate used to run here, driving straight
# passes for GATE_SECONDS before Nav2 ever launched -- a second scripted motion
# source, and exactly the "square patrol" shape the mission forbids.
# slam_toolbox does not need a warm-up: it maps during transit, which is the
# whole point of building the map live (ADR 0023). The gate is still run, but
# --observe-only and CONCURRENTLY with the transit, so it measures the mission
# instead of a bench pattern that precedes it.

# WAIT FOR TF BEFORE LAUNCHING NAV. Nav2's costmaps ask for map -> base_footprint
# the instant they activate; if SLAM and the EKF have not published yet, the
# lookups fail with "frame does not exist", bt_navigator's lifecycle transition
# times out, lifecycle_manager aborts bringup, and every later goal is answered
# "Action server is inactive. Rejecting the goal."
#
# That startup race is why nominally identical runs alternated between reaching
# 0.24 m of B and never leaving the spawn.
echo "=== waiting for the TF chain ==="
if ! python3 "$REPO/tools/wait_for_tf.py" --target map --source base_footprint --timeout 120; then
  echo "**INFRASTRUCTURE: map->base_footprint never appeared; twin TF is not up**" >&2
  exit 3
fi

echo "=== nav stack ==="
if [ "$ROBOT" = robot1 ]; then
  NAV_LAUNCH="$REPO/config/robot1/robot1_nav_corridor_launch.py"
else
  NAV_LAUNCH="fleet_bringup robot2_nav_sim_launch.py"
fi
# shellcheck disable=SC2086
ros2 launch $NAV_LAUNCH \
  >"$EVIDENCE/nav-launch-$ROBOT-$PROFILE.log" 2>&1 &
nav_pid=$!

# WAIT FOR THE ACTION SERVER, never a fixed sleep. Lifecycle activation can
# stall -- observed as "failed to send response to /controller_server/
# change_state (timeout)" -- and a fixed sleep then sends a goal into a stack
# that has no navigate_to_pose server, which the gate reports as a navigation
# failure. It is not one: the robot was never asked to move. A stack that never
# activates is INFRASTRUCTURE (exit 3), so it is rerun rather than recorded as
# a result about the robot.
# ADVERTISED IS NOT ACTIVE. `ros2 action list` shows /navigate_to_pose while
# bt_navigator is still inactive, so the old check passed on runs whose
# lifecycle bringup had already aborted -- and the goal was then rejected by a
# server the runner had just called ready. Ask the lifecycle state instead.
echo "  waiting for bt_navigator to reach ACTIVE..."
nav_ready=0
for _ in $(seq 1 40); do
  state=$(ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)
  case "$state" in
    *active*) nav_ready=1; echo "  bt_navigator active"; break ;;
  esac
  sleep 5
done
if [ "$nav_ready" != 1 ]; then
  echo "**INFRASTRUCTURE: bt_navigator never reached ACTIVE in 200 s (last state: ${state:-unknown})**" >&2
  tail -5 "$EVIDENCE/nav-launch-$ROBOT-$PROFILE.log" | sed 's/^/    /' >&2
  exit 3
fi

# The recorder starts BEFORE the goal so the transit is measured from its first
# metre, and outlives the nav gate's own timeout so it cannot truncate a slow
# but successful delivery. It commands nothing (--observe-only).
# Nav gets what the CAP has left after bring-up, measured rather than assumed:
# Isaac's load time varies by a minute between runs, so a fixed nav window either
# overruns the watchdog or wastes the cap. 20 s is reserved for teardown.
if [ -z "$NAV_TIMEOUT" ]; then
  elapsed=$(( $(date +%s) - SESSION_START_S ))
  NAV_TIMEOUT=$(( SIM_MAX_S - elapsed - 20 ))
  if [ "$NAV_TIMEOUT" -lt 30 ]; then
    echo "**INFRASTRUCTURE: bring-up used ${elapsed}s of the ${SIM_MAX_S}s cap; no window left to navigate**" >&2
    exit 3
  fi
  echo "  nav window: ${NAV_TIMEOUT}s (cap ${SIM_MAX_S}s, bring-up took ${elapsed}s)"
fi
: "${GATE_SECONDS:=$((NAV_TIMEOUT + 10))}"
echo "=== T3.3a transit recorder (observe-only, ${GATE_SECONDS}s) ==="
python3 "$REPO/tools/corridor_sim_gate.py" --seconds "$GATE_SECONDS" \
  --profile "$PROFILE" --robot "$ROBOT" $GATED --observe-only \
  --manifest "$MANIFEST" \
  ${CONTRACT_CAVEAT:+--caveat "$CONTRACT_CAVEAT"} \
  --out "$EVIDENCE/gate-$ROBOT-$PROFILE.json" \
  >"$EVIDENCE/gate-$ROBOT-$PROFILE.log" 2>&1 &
recorder_pid=$!

echo "=== T3.3b governed Nav2 goal A->B ==="
python3 "$REPO/tools/corridor_nav_gate.py" --profile "$PROFILE" --robot "$ROBOT" $GATED \
  ${CONTRACT_CAVEAT:+--caveat "$CONTRACT_CAVEAT"} \
  --manifest "$MANIFEST" \
  --timeout "$NAV_TIMEOUT" \
  --out "$EVIDENCE/nav-$ROBOT-$PROFILE.json" || status=1

# The recorder's own verdict is a gate result too, so it is waited for rather
# than killed -- but a nav gate that ended early must not hang the run behind
# the recorder's full window.
echo "=== transit recorder verdict ==="
wait "$recorder_pid" || status=1
sed 's/^/    /' "$EVIDENCE/gate-$ROBOT-$PROFILE.log" | tail -20

teardown || status=1
trap - EXIT INT TERM

# --- map quality, measured, not eyeballed -----------------------------------
# "The walls look single-lined" is not a result. Full SLAM divergence was missed
# once by exactly that check, and every number derived from a diverged map
# (free widths, costmap costs, reachability) is void -- so the map is scored
# before any of them are quoted.
#
# TWO ROWS ONLY, and deliberately: `duplicate wall extent` is the same wall
# drawn twice, which IS the ghosting signature, and `median wall thickness` is
# the smear. --reference is NOT passed: score_slam_map.py:28-30 says its span
# rows measure occupied extent along an axis, which is valid for a convex room
# and NOT for an L-shaped space like this corridor. Saying so here is what that
# docstring asks for instead of quietly scoring the wrong thing.
#
# --self-test runs first every time. An instrument whose negative controls are
# checked once at authoring time is an instrument nobody is checking.
echo "=== map quality ==="
SCORER=/home/alexmint/Development/robot-fleet/src/yahboomcar-ros2/tools/score_slam_map.py
if ! python3 "$SCORER" --self-test >"$EVIDENCE/map-selftest-$ROBOT-$PROFILE.txt" 2>&1; then
  echo "**INFRASTRUCTURE: map scorer self-test FAILED; its verdicts are not trustworthy**" >&2
  status=1
else
  # -newer $SESSION_MARKER, not `ls -t`: a session that saved no map would
  # otherwise silently score the PREVIOUS session's, and report its verdict as
  # this run's. That happened -- two consecutive runs reported an identical
  # 0.800 m duplicate-wall extent because the second produced no map at all.
  SAVED_MAP=$(find \
    "$HOME"/Development/MicroROS/MicroROS-assets/logs/sessions \
    "$HOME"/Development/robot-fleet/src/MicroROS/MicroROS-assets/logs/sessions \
    -maxdepth 2 -name 'map-*.yaml' -path "*-d$DOMAIN/*" -newer "$SESSION_MARKER" \
    2>/dev/null | head -1)
  rm -f "$SESSION_MARKER"
  if [ -z "$SAVED_MAP" ]; then
    echo "**THIS session saved no map on domain $DOMAIN: SLAM produced nothing to score**" >&2
    status=1
  else
    echo "  scoring $SAVED_MAP"
    cp "$SAVED_MAP" "${SAVED_MAP%.yaml}.pgm" "$EVIDENCE/" 2>/dev/null || true
    python3 "$SCORER" --map "$SAVED_MAP" \
      --json "$EVIDENCE/map-score-$ROBOT-$PROFILE.json" \
      | tee "$EVIDENCE/map-score-$ROBOT-$PROFILE.txt" || status=1
  fi
fi

# A run the watchdog killed is INFRASTRUCTURE, never a verdict about the robot:
# it was stopped mid-transit by the clock, so its gate failures describe an
# interrupted run and nothing else.
if [ -f "$WATCHDOG_FLAG" ]; then
  rm -f "$WATCHDOG_FLAG"
  echo "=== $PROFILE: **INFRASTRUCTURE -- killed at the ${SIM_MAX_S}s cap, not a result** ===" >&2
  exit 3
fi
if [ "$status" = 0 ]; then
  echo "=== $PROFILE: PASS ==="
else
  echo "=== $PROFILE: **FAIL** (artifacts kept under $EVIDENCE) ==="
fi
exit "$status"
