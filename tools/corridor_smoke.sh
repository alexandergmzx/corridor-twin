#!/usr/bin/env bash
# Corridor smoke (v2 plan T1.4): drive the RaspTank twin in a corridor arena
# under the fleet's own simctl, and check the robot2 contract against it.
#
#     bash tools/corridor_smoke.sh
#     bash tools/corridor_smoke.sh --domain 69 --arena out/arena_corridor_uniform_m6_n6.usd
#
# WHY THIS SCRIPT EXISTS RATHER THAN A DOCUMENTED COMMAND LINE
# ------------------------------------------------------------
# `simctl` starts a session on the domain it is told about; it does not export
# ROS_DOMAIN_ID back to whoever called it (fleet finding F9). So a caller who
# starts a simulator on 67 and then runs a checker in the same shell measures
# domain 0, finds nothing, and reads it as a dead twin. Every environment
# variable this smoke depends on is therefore set HERE, once, and inherited by
# both simctl and the checker.
#
# RASPTANK_ARENA_USD is the second half of that: the runner defaults to the
# RaspTank's own 4x4 room, so without it this script would smoke-test the wrong
# scene and pass. The arena is passed as an ABSOLUTE path, which the runner's
# hook takes as-is.
#
# DOMAIN CHOICE IS A SAFETY PROPERTY, not a preference. 20 is the real car, 42
# and 43 are the corridor demonstration's own planes, 44 is reserved for
# corridor replays, 66 is the fleet's standing sim domain and 68 is in use --
# fleet architecture.md D-09/D-20. Sim scratch is 67 and 69. The refusal below
# is structural: a wrong domain is refused by comparison before anything is
# started, in the same spirit as simctl's own car-domain refusal, because
# discovery-based detection cannot see a robot that is merely powered off.
set -euo pipefail

# Logical, not physical: bash's `pwd` keeps the symlinked fleet path this
# checkout is reached through, and the sibling tools are only reachable from it
# (D5; see tools/build_corridor_arena.py for the full explanation).
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$(cd -- "$REPO/.." && pwd)"
FLEET="$(cd -- "$SRC/.." && pwd)"

SIMCTL="$SRC/yahboomcar-ros2/tools/simctl"
CONTRACT="$SRC/rasptank-ros2/tools/check_rasptank_contract.py"
WS_SETUP="$FLEET/ground_station/install/setup.bash"

DOMAIN="${CORRIDOR_SMOKE_DOMAIN:-67}"
ARENA="$REPO/out/arena_corridor_nominal_m6_n3.usd"
# The Isaac twin cannot reach the checker's 100 Hz default; 60 is the figure the
# twin is held to. Stated here rather than left to the default so a change is
# deliberate.
IMU_HZ=60
SECONDS_SAMPLE=12

while [ $# -gt 0 ]; do
  case "$1" in
    --domain) DOMAIN="$2"; shift 2 ;;
    --arena)  ARENA="$2";  shift 2 ;;
    --seconds) SECONDS_SAMPLE="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

case "$DOMAIN" in
  20) echo "REFUSED: domain 20 is the REAL CAR (fleet D-09)." >&2; exit 2 ;;
  42|43) echo "REFUSED: domain $DOMAIN is a corridor demonstration plane (D-20)." >&2; exit 2 ;;
  44) echo "REFUSED: domain 44 is reserved for corridor replays (D-20)." >&2; exit 2 ;;
  66) echo "REFUSED: domain 66 is the fleet's standing sim domain (simctl SIM_DOMAIN)." >&2; exit 2 ;;
  68) echo "REFUSED: domain 68 is marked in use." >&2; exit 2 ;;
  70) echo "REFUSED: domain 70 is dirty/unavailable (D-20)." >&2; exit 2 ;;
esac

case "$ARENA" in /*) ;; *) ARENA="$REPO/$ARENA" ;; esac
[ -f "$ARENA" ] || {
  echo "arena missing: $ARENA" >&2
  echo "  build it: ~/isaac/env_isaaclab/bin/python tools/build_corridor_arena.py --profile nominal_m6_n3" >&2
  exit 2
}
for path in "$SIMCTL" "$CONTRACT" "$WS_SETUP"; do
  [ -e "$path" ] || { echo "fleet layout incomplete, missing: $path" >&2; exit 2; }
done

EVIDENCE="$REPO/out/evidence/corridor-smoke"
mkdir -p "$EVIDENCE"
CONTRACT_JSON="$EVIDENCE/contract-domain${DOMAIN}.json"
TOPICS_TXT="$EVIDENCE/topics-domain${DOMAIN}.txt"

# Everything below inherits these. This is the whole point of the script.
export ROS_DOMAIN_ID="$DOMAIN"
export RASPTANK_ARENA_USD="$ARENA"
# A user-local NumPy 2.2 wheel conflicts with Jazzy's NumPy 1.x cv_bridge ABI.
export PYTHONNOUSERSITE=1

echo "=== corridor smoke ==="
echo "  domain : $ROS_DOMAIN_ID  (scratch; 67/69 convention)"
echo "  arena  : $RASPTANK_ARENA_USD"
echo "  imu gate: ${IMU_HZ} Hz"

# Single-occupancy is honour-system in this fleet, so it is checked rather than
# assumed: a second Isaac on one GPU is how a smoke turns into an unexplainable
# performance result.
#
# The pattern matches the twin's actual entry points, NOT the words "isaac" or
# "omniverse" anywhere on a command line. A loose pattern refused this script on
# its first run by matching the shell that launched it -- this repository's own
# paths contain "omniverse", and so does the scratch directory. A guard that
# fires on its own caller is worse than no guard: it trains you to bypass it.
occupants() {
  # Exclude this process AND its whole ancestry. A bare "$$" filter is not
  # enough: the guard has now been tripped twice by the SHELL THAT LAUNCHED IT,
  # because this repository's paths contain "omniverse" and because a caller
  # who types the pattern on a command line puts it into their own cmdline.
  # Both of those are ancestors, never the twin.
  local ancestry=" $$ " walk=$PPID
  while [ -n "$walk" ] && [ "$walk" -gt 1 ] 2>/dev/null; do
    ancestry="$ancestry$walk "
    walk=$(ps -o ppid= -p "$walk" 2>/dev/null | tr -d ' ')
  done
  pgrep -af 'rasptank_twin_runner\.py|isaac-sim|/kit/kit' 2>/dev/null \
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

# ROS setup scripts read variables they do not always set (COLCON_TRACE,
# AMENT_TRACE_SETUP_FILES), so `set -u` has to stand down across the source or
# the smoke dies before it starts anything.
set +u
# shellcheck disable=SC1090,SC1091
source "$WS_SETUP"
set -u

status=0
stopped=0
stop_session() {
  [ "$stopped" = 1 ] && return 0
  stopped=1
  echo "=== stopping session on domain $DOMAIN ==="
  "$SIMCTL" stop --domain "$DOMAIN" || true
  sleep 3
  if [ -n "$(occupants)" ]; then
    echo "!! SESSION NOT DEAD: survivors below" >&2
    occupants >&2
    return 1
  fi
  echo "  verified dead"
  return 0
}
# The session outlives this shell unless it is torn down explicitly, so the trap
# runs on failure and interrupt too, not only on the happy path.
trap 'stop_session || true' EXIT INT TERM

echo "=== simctl start ==="
"$SIMCTL" start --robot robot2 --backend isaac --domain "$DOMAIN"

echo "=== contract check (--imu-hz $IMU_HZ) ==="
if python3 "$CONTRACT" --imu-hz "$IMU_HZ" --seconds "$SECONDS_SAMPLE" --json \
     >"$CONTRACT_JSON" 2>"$EVIDENCE/contract-stderr.txt"; then
  echo "  contract: PASS -> $CONTRACT_JSON"
else
  echo "  contract: **FAIL** -> $CONTRACT_JSON (kept; not retried, not tuned)"
  status=1
fi

# The conditioner and the ground-station matcher are what make a map possible at
# all. Their absence is silent: SLAM simply never builds, and the run looks
# healthy the whole time.
echo "=== required topics ==="
ros2 topic list >"$TOPICS_TXT" 2>&1 || true
for topic in /robot2/scan_filtered /robot2/odom_laser; do
  if grep -qx -- "$topic" "$TOPICS_TXT"; then
    hz="$(timeout 20 ros2 topic hz "$topic" --window 10 2>&1 | grep -m1 'average rate' || true)"
    echo "  $topic present  ${hz:-(no rate sample)}"
    if [ -z "$hz" ]; then
      echo "  **$topic is listed but published nothing in 20 s**"
      status=1
    fi
  else
    echo "  **$topic MISSING** (twin_scan_conditioner / laser odometry not in the loop)"
    status=1
  fi
done

stop_session || status=1
trap - EXIT INT TERM

if [ "$status" = 0 ]; then
  echo "=== corridor smoke: PASS ==="
else
  echo "=== corridor smoke: **FAIL** (evidence kept under $EVIDENCE) ==="
fi
exit "$status"
