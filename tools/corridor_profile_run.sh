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
GATE_SECONDS=90
NAV_TIMEOUT=300

while [ $# -gt 0 ]; do
  case "$1" in
    --profile) PROFILE="$2"; shift 2 ;;
    --robot) ROBOT="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --gated) GATED="--gated"; shift ;;
    --gate-seconds) GATE_SECONDS="$2"; shift 2 ;;
    --nav-timeout) NAV_TIMEOUT="$2"; shift 2 ;;
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

ARENA="$REPO/out/arena_corridor_${ROBOT}_${PROFILE}.usd"
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
stopped=0
teardown() {
  [ "$stopped" = 1 ] && return 0
  stopped=1
  echo "=== stopping $PROFILE on domain $DOMAIN ==="
  [ -n "$nav_pid" ] && kill -TERM "$nav_pid" 2>/dev/null || true
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
echo "=== simctl start ==="
"$SIMCTL" start --robot "$ROBOT" --backend isaac --domain "$DOMAIN" || {
  echo "**INFRASTRUCTURE: simctl start failed for $PROFILE**" >&2; exit 3; }

# Contract numbers are PER-ROBOT and do not transfer. robot2 is checked with
# --imu-hz 60; robot1's checker carries its own WANT_HZ (scan 12 / odom_raw 11
# / imu 25, check_isaac_contract.py:51) and takes no rate flags at all.
if [ "$ROBOT" = robot1 ]; then
  CONTRACT_ARGS=(--domain "$DOMAIN")
else
  CONTRACT_ARGS=(--imu-hz 60)
fi
echo "=== precondition: $ROBOT contract (${CONTRACT_ARGS[*]}) ==="
# stdout only into the JSON: the checker appends a human summary and its
# FAIL lines after the document, which made every artifact unparseable exactly
# when it mattered most -- on the failures.
if ! python3 "$CONTRACT" "${CONTRACT_ARGS[@]}" --json >"$EVIDENCE/contract-$ROBOT-$PROFILE.json" \
     2>"$EVIDENCE/contract-$ROBOT-$PROFILE.err"; then
  echo "**INFRASTRUCTURE: contract check failed for $PROFILE; twin not fit to gate**" >&2
  exit 3
fi
echo "  contract PASS -> $EVIDENCE/contract-$ROBOT-$PROFILE.json"

status=0

echo "=== T3.3a drive-and-map gate (${GATE_SECONDS}s) ==="
python3 "$REPO/tools/corridor_sim_gate.py" --seconds "$GATE_SECONDS" \
  --profile "$PROFILE" --robot "$ROBOT" $GATED \
  --out "$EVIDENCE/gate-$ROBOT-$PROFILE.json" || status=1

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
sleep 25

echo "=== T3.3b governed Nav2 goal A->B ==="
python3 "$REPO/tools/corridor_nav_gate.py" --profile "$PROFILE" --robot "$ROBOT" $GATED \
  --manifest "$REPO/out/corridor.manifest.json" \
  --timeout "$NAV_TIMEOUT" \
  --out "$EVIDENCE/nav-$ROBOT-$PROFILE.json" || status=1

teardown || status=1
trap - EXIT INT TERM

if [ "$status" = 0 ]; then
  echo "=== $PROFILE: PASS ==="
else
  echo "=== $PROFILE: **FAIL** (artifacts kept under $EVIDENCE) ==="
fi
exit "$status"
