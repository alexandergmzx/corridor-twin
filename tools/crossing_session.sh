#!/usr/bin/env bash
# One live crossing session: adapter on A's plane, gateway, measurement in P's.
#
#     bash tools/crossing_session.sh --label 640x360
#     bash tools/crossing_session.sh --label 1280x720 --stage out/corridor-720p.usda --certificate no
#
# v2 plan T2.2 (delivery, latency, VRAM, bridge CPU) and T2.3 (isolation
# certificate + mutation control), which share one session because both need the
# same live crossing and a second Isaac startup buys nothing.
#
# The three processes cannot share a shell. The adapter runs on Isaac's bundled
# Jazzy under Python 3.11 and re-execs itself with system ROS paths stripped;
# the measurement runs on system Jazzy under 3.12; the gateway is a member of
# both domains and therefore pinned to neither. Mixing any two of those ABIs in
# one shell is the failure mode CLAUDE.md's environment discipline exists to
# prevent, so each gets its own subshell with its own environment.
set -euo pipefail

REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_DOMAIN=42
POLICE_DOMAIN=43
LABEL=640x360
STAGE="$REPO/out/corridor.usda"
SECONDS_CAPTURE=60
RATE_HZ=15
UPDATES=15000
# 1.0 m/s finishes the authored route in ~24 s, which is shorter than the
# capture window and made the first run score 0.37 of nominal on a bridge that
# was in fact carrying 95.7% of everything published. Slower drive, longer
# source, honest ratio.
DRIVE_SPEED=0.35
CERTIFICATE=yes
CAMERA_RES=""
ISAAC_PYTHON="${ISAAC_PYTHON:-$HOME/isaac/env_isaaclab/bin/python}"

while [ $# -gt 0 ]; do
  case "$1" in
    --label) LABEL="$2"; shift 2 ;;
    --stage) STAGE="$2"; shift 2 ;;
    --seconds) SECONDS_CAPTURE="$2"; shift 2 ;;
    --drive-speed) DRIVE_SPEED="$2"; shift 2 ;;
    --rate) RATE_HZ="$2"; shift 2 ;;
    --updates) UPDATES="$2"; shift 2 ;;
    --certificate) CERTIFICATE="$2"; shift 2 ;;
    --camera-resolution) CAMERA_RES="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
case "$STAGE" in /*) ;; *) STAGE="$REPO/$STAGE" ;; esac
MANIFEST="${STAGE%.usda}.manifest.json"
EVIDENCE="$REPO/out/evidence/crossing"
mkdir -p "$EVIDENCE"

[ -f "$STAGE" ] || { echo "stage missing: $STAGE" >&2; exit 2; }
[ -x "$ISAAC_PYTHON" ] || { echo "isaac python missing: $ISAAC_PYTHON" >&2; exit 2; }

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
  pgrep -af 'isaac_5_1_ros_camera|rasptank_twin_runner\.py|isaac-sim|/kit/kit' 2>/dev/null \
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

children=()
cleanup() {
  echo "=== teardown ==="
  for pid in "${children[@]:-}"; do
    [ -n "${pid:-}" ] && kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 4
  for pid in "${children[@]:-}"; do
    [ -n "${pid:-}" ] && kill -KILL "$pid" 2>/dev/null || true
  done
  pkill -f 'domain_bridge' 2>/dev/null || true
  sleep 2
  if [ -n "$(occupants)" ]; then
    echo "!! NOT DEAD:" >&2; occupants >&2
  else
    echo "  verified dead"
  fi
}
trap cleanup EXIT INT TERM

echo "=== crossing session: $LABEL ==="
echo "  stage : $STAGE"
echo "  domains: A=$ROBOT_DOMAIN  P=$POLICE_DOMAIN"

# --- gateway: a member of both domains, pinned to neither --------------------
(
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$REPO/install/setup.bash"
  set -u
  export PYTHONNOUSERSITE=1
  exec ros2 launch corridor_gateway gateway.launch.py \
    robot_domain_id:="$ROBOT_DOMAIN" police_domain_id:="$POLICE_DOMAIN"
) >"$EVIDENCE/gateway-$LABEL.log" 2>&1 &
children+=($!)
echo "  gateway starting ($ROBOT_DOMAIN -> $POLICE_DOMAIN, one way)"

# --- adapter on A's plane ----------------------------------------------------
env -u AMENT_PREFIX_PATH -u PYTHONPATH -u ROS_DISTRO -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
    OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID="$ROBOT_DOMAIN" \
    "$ISAAC_PYTHON" "$REPO/tools/isaac_5_1_ros_camera.py" "$STAGE" \
    --manifest "$MANIFEST" --drive-speed-mps "$DRIVE_SPEED" --updates "$UPDATES" \
    --drive-out "$EVIDENCE/drive-schedule-$LABEL.json" \
    --report-gpu-memory ${CAMERA_RES:+--camera-resolution $CAMERA_RES} \
    >"$EVIDENCE/isaac-$LABEL.log" 2>&1 &
ADAPTER_PID=$!
children+=("$ADAPTER_PID")
echo "  adapter starting on domain $ROBOT_DOMAIN (updates=$UPDATES)"

# --- wait for the crossing to open, in P's plane -----------------------------
set +u
source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1

echo "  waiting for /p_cam/image_raw in P's plane (up to 300 s)..."
deadline=$((SECONDS + 300))
until ROS_DOMAIN_ID="$POLICE_DOMAIN" ros2 topic list 2>/dev/null | grep -qx /p_cam/image_raw; do
  [ "$SECONDS" -ge "$deadline" ] && {
    echo "**the crossing never opened; see $EVIDENCE/isaac-$LABEL.log**" >&2
    exit 1
  }
  sleep 5
done
echo "  crossing open"

status=0

echo "=== T2.2 measurement (${SECONDS_CAPTURE}s at ${RATE_HZ} Hz nominal) ==="
python3 "$REPO/tools/crossing_measure.py" \
  --seconds "$SECONDS_CAPTURE" --rate-hz "$RATE_HZ" --label "$LABEL" \
  --robot-domain "$ROBOT_DOMAIN" --police-domain "$POLICE_DOMAIN" \
  --producer-manifest "$EVIDENCE/drive-schedule-$LABEL.json" \
  --out "$EVIDENCE/crossing-$LABEL.json" || status=1

if [ "$CERTIFICATE" = yes ]; then
  # A truth publisher on A's plane, UNBRIDGED, for the whole certificate phase.
  #
  # Without it the certificate is vacuous and says so: this adapter-only session
  # publishes nothing on 42 except the allowlist itself, so "P sees exactly the
  # allowlist" would be trivially true with nothing available to leak. The first
  # run returned INCONCLUSIVE for precisely that reason. /test/ground_truth/speed
  # is simulator truth -- the thing the truth-isolation invariant forbids
  # reaching P -- so its presence on 42 and absence in P is the claim worth
  # certifying, and the mutation below then relays this same topic to prove the
  # instrument would notice.
  echo "=== T2.3 truth source on A's plane (unbridged; the thing that must not leak) ==="
  ROS_DOMAIN_ID="$ROBOT_DOMAIN" python3 "$REPO/tools/truth_source.py" \
    >"$EVIDENCE/truth-source-$LABEL.log" 2>&1 &
  TRUTH_PID=$!
  children+=("$TRUTH_PID")
  sleep 6

  echo "=== T2.3 isolation certificate ==="
  python3 "$REPO/tools/isolation_certificate.py" \
    --robot-domain "$ROBOT_DOMAIN" --police-domain "$POLICE_DOMAIN" \
    --label "$LABEL" --out "$EVIDENCE/certificate-$LABEL.json" || status=1

  # --- mutation control ------------------------------------------------------
  # A second bridge relaying one extra A-plane topic. The certificate must go
  # RED. /test/ground_truth/speed is chosen because it is simulator truth --
  # the exact thing the truth-isolation invariant forbids reaching P -- and
  # because relaying it is a one-line configuration mistake, not a hypothesis.
  # A publisher for it is stood up on A's plane first: a bridge entry for a
  # topic nobody publishes relays nothing, and would certify green while
  # proving only that the mutation was inert.
  echo "=== T2.3 mutation control (must go RED) ==="
  MUTANT="$EVIDENCE/mutant-bridge.yaml"
  sed -e 's/^name: .*/name: corridor_twin_gateway_MUTANT/' \
      "$REPO/src/corridor_gateway/config/corridor_domain_bridge.yaml" >"$MUTANT"
  cat >>"$MUTANT" <<'YAML'
  /test/ground_truth/speed:
    type: geometry_msgs/msg/TwistStamped
    qos:
      reliability: reliable
      durability: volatile
      history: keep_last
      depth: 10
YAML

  BRIDGE_BIN="$(ros2 pkg prefix domain_bridge)/lib/domain_bridge/domain_bridge"
  "$BRIDGE_BIN" "$MUTANT" --from "$ROBOT_DOMAIN" --to "$POLICE_DOMAIN" \
    >"$EVIDENCE/mutant-bridge-$LABEL.log" 2>&1 &
  MUTANT_PID=$!
  children+=("$MUTANT_PID")
  sleep 12

  if python3 "$REPO/tools/isolation_certificate.py" \
       --robot-domain "$ROBOT_DOMAIN" --police-domain "$POLICE_DOMAIN" \
       --label "$LABEL-mutated" --expect-red \
       --out "$EVIDENCE/certificate-$LABEL-mutated.json"; then
    echo "  mutation control: PASS (certificate went red as required)"
  else
    echo "  mutation control: **FAIL** (the leak was not detected)"
    status=1
  fi
  kill -TERM "$MUTANT_PID" 2>/dev/null || true
fi

# --- producer gate: needs the adapter's schedule, written only at drive end ---
echo "=== producer gate (waiting for the adapter to finish its route) ==="
waited=0
while kill -0 "$ADAPTER_PID" 2>/dev/null && [ "$waited" -lt 180 ]; do
  sleep 5; waited=$((waited + 5))
done
if kill -0 "$ADAPTER_PID" 2>/dev/null; then
  echo "  **adapter still running after ${waited}s; producer gate NOT MEASURED**"
  status=1
else
  python3 "$REPO/tools/producer_gate.py" \
    --schedule "$EVIDENCE/drive-schedule-$LABEL.json" \
    --crossing "$EVIDENCE/crossing-$LABEL.json" \
    --declared-hz "$RATE_HZ" || status=1
fi

if [ "$status" = 0 ]; then echo "=== crossing session $LABEL: PASS ==="
else echo "=== crossing session $LABEL: **FAIL** ==="; fi
exit "$status"
