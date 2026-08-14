#!/usr/bin/env bash
# One command for the live corridor-twin demonstration.
#
# The demonstration needs three processes that must not share a shell. The
# observer side runs on system Jazzy under Python 3.12; the Isaac adapter runs
# on Isaac's bundled Jazzy under Python 3.11 and re-execs itself into an
# isolated environment that rejects leaked system ROS paths. They meet over
# DDS, which docs/ACTIVATION.md records working: an external system-Jazzy
# consumer received synchronized 640x360 rgb8 frames from this adapter.
#
# Since ADR 0020 they no longer meet on the same DDS domain. A runs on
# ROBOT_DOMAIN_ID, P runs on POLICE_DOMAIN_ID, and discovery does not cross
# between them, so P cannot see, list, or subscribe to anything A publishes.
# The third process is the gateway, which relays exactly the camera contract
# and /clock one way, A to P. Kill it mid-run and P goes blind -- that is the
# negative control for the isolation claim.
#
# The default run drives A at a constant 1.0 m/s. That single unchanged speed
# is legal on the wide approach and illegal once the corridor narrows and the
# limit tightens, so one pass shows both compliance and exactly one violation
# without anyone touching a throttle.
#
# ---------------------------------------------------------------------------
# V2 TRANSITION WARNING (2026-08-11). THIS SCRIPT RUNS, BUT IT IS NOT THE v2
# DEMONSTRATION. Read this before showing it to anyone.
#
# The topics were renamed to P's camera (/p_cam/*) in v2 plan task T2.2, so
# every process here agrees and the pipeline is end-to-end again. What did NOT
# change is the camera's PLACEMENT: it is still mounted on A and aimed the way
# A's front camera was. Under ADR 0021 the enforcement camera is P's roadside
# instrument and A is camera-less, so what this script currently shows is the
# v1 scenario wearing v2 names.
#
# Concretely, do not quote a run of this script as evidence for:
#   - P's camera placement or field of view       (a later task)
#   - the v2 resolution or rate                   (ADR 0024 re-measures both)
#   - autonomous navigation                       (the route here is authored)
#   - the learned detector                        (ADRs 0023/0024, later phase)
#
# What it IS still good for: the domain split, the gateway crossing, and the
# camera-only speed estimator, all of which are unchanged by the rename.
# ---------------------------------------------------------------------------
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stage="${STAGE:-$workspace_dir/out/corridor.usda}"
# `${stage%.usda}` only strips the v1 suffix. A composed arena is a `.usd`, so
# the default silently became `<arena>.usd.manifest.json`, which does not
# exist, and the run died on a missing manifest rather than on a wrong one.
manifest="${MANIFEST:-${stage%.usd*}.manifest.json}"
speed="${SPEED_MPS:-1.0}"
profile="${CORRIDOR_PROFILE:-}"
updates="${UPDATES:-3000}"
# A composed arena carries the real twin at /World/Robot and a PhysicsScene;
# the v1 stage carries a kinematic box and neither. Both default to the v1
# answer, so nothing changes for a v1 run.
robot_prim="${ROBOT_PRIM:-}"
deactivate_physics=""
if [ "${DEACTIVATE_PHYSICS:-0}" = 1 ]; then
  deactivate_physics="--deactivate-physics"
fi
view="${VIEW:-rviz}"
isaac_python="${ISAAC_PYTHON:-$HOME/isaac/env_isaaclab/bin/python}"
evidence_dir="${EVIDENCE_DIR:-$workspace_dir/out/evidence/live-demo}"
robot_domain="${ROBOT_DOMAIN_ID:-42}"
police_domain="${POLICE_DOMAIN_ID:-43}"

if [[ "$robot_domain" == "$police_domain" ]]; then
  echo "ROBOT_DOMAIN_ID and POLICE_DOMAIN_ID must differ; both are $robot_domain." >&2
  echo "Equal domains would put A and P back on one communication plane." >&2
  exit 2
fi

usage() {
  cat <<'USAGE'
Usage: tools/run_demo.sh [--headless] [--no-rviz] [--record]

Environment overrides:
  STAGE=<path.usda>        stage to open        (default out/corridor.usda)
  MANIFEST=<path.json>     surveyed manifest    (default alongside the stage)
  SPEED_MPS=<float>        constant path speed  (default 1.0)
  CORRIDOR_PROFILE=<name>  corridor variant     (default the stage's selection)
  UPDATES=<int>            adapter safety cap   (default 3000)
  ROBOT_PRIM=<prim>        prim the schedule drives (arena: /World/Robot)
  DEACTIVATE_PHYSICS=1     no solver; required when driving an articulation
  VIEW=<name>              viewport perspective (default rviz; corner, chase)
  ISAAC_PYTHON=<path>      Isaac interpreter    (default ~/isaac/env_isaaclab/bin/python)
  ROBOT_DOMAIN_ID=<int>    A's ROS domain       (default 42)
  POLICE_DOMAIN_ID=<int>   P's ROS domain       (default 43)
USAGE
}

gui_flag="--gui"
rviz="true"
record="false"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) gui_flag="" ;;
    --no-rviz) rviz="false" ;;
    --record) record="true" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ ! -f "$stage" ]]; then
  echo "Stage not found: $stage" >&2
  echo "Build it first:  python -m scene.build --m 6.0 --n 3.0 --out out/corridor.usda" >&2
  exit 2
fi
if [[ ! -f "$manifest" ]]; then
  echo "Manifest not found: $manifest" >&2
  exit 2
fi
if [[ ! -x "$isaac_python" ]]; then
  echo "Isaac interpreter not found: $isaac_python" >&2
  echo "Set ISAAC_PYTHON, or run the simulator-free fallback:" >&2
  echo "  ros2 launch police_observer synthetic_demo.launch.py manifest:=$manifest" >&2
  exit 2
fi
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Workspace is not built. Run: colcon build --symlink-install" >&2
  exit 2
fi

mkdir -p "$evidence_dir"
ros_log="$evidence_dir/ros-side.log"
isaac_log="$evidence_dir/isaac-side.log"
gateway_log="$evidence_dir/gateway.log"
drive_out="$evidence_dir/commanded-pose-schedule.json"

# Job control puts each background job in its own process group whose id is the
# job's pid, which is what makes the cleanup below able to reach every process
# it started. `ros2 launch` spawns the observer, the display and RViz as its own
# children, so signalling only the launcher leaves all of them running: a later
# demo run then finds stale nodes publishing on the same topics and a pile of
# RViz windows. That was observed, not theorised.
set -m
children=()
cleanup() {
  local pid remaining attempt
  for pid in "${children[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done
  for attempt in 1 2 3 4 5 6 7 8; do
    remaining=0
    for pid in "${children[@]:-}"; do
      if kill -0 -- "-$pid" 2>/dev/null; then remaining=1; fi
    done
    [[ "$remaining" == "0" ]] && break
    sleep 0.5
  done
  for pid in "${children[@]:-}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "corridor-twin demonstration"
echo "  stage        $stage"
echo "  manifest     $manifest"
echo "  path speed   $speed m/s"
echo "  evidence     $evidence_dir"
echo "  A's domain   $robot_domain"
echo "  P's domain   $police_domain (reachable from A's only through the gateway)"
echo

# --- P's side: system Jazzy, Python 3.12, police domain ----------------------
# Started first, matching the ordering docs/ACTIVATION.md validates for the
# external probe: the consumer is up before the publisher begins.
(
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$workspace_dir/install/setup.bash"
  set -u
  export PYTHONNOUSERSITE=1
  export ROS_DOMAIN_ID="$police_domain"
  # ros2 launch rejects an empty value outright, so an unset profile has to be
  # omitted rather than passed as corridor_profile:= -- otherwise the whole ROS
  # side fails to start and the run looks like a DDS problem.
  launch_args=(manifest:="$manifest" use_sim_time:=true rviz:="$rviz"
               police_domain_id:="$police_domain")
  if [[ -n "$profile" ]]; then
    launch_args+=(corridor_profile:="$profile")
  fi
  exec ros2 launch police_observer live_demo.launch.py "${launch_args[@]}"
) >"$ros_log" 2>&1 &
children+=($!)
echo "observer + display starting on domain $police_domain (log: $ros_log)"

# --- The gateway: in both domains, which nothing else is ---------------------
# No ROS_DOMAIN_ID is exported here on purpose. The bridge builds one
# participant per domain from its own configuration; pinning it to either side
# would make it a member of that side rather than the boundary between them.
(
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$workspace_dir/install/setup.bash"
  set -u
  export PYTHONNOUSERSITE=1
  exec ros2 launch corridor_gateway gateway.launch.py \
    robot_domain_id:="$robot_domain" police_domain_id:="$police_domain"
) >"$gateway_log" 2>&1 &
children+=($!)
echo "gateway starting, $robot_domain -> $police_domain one way (log: $gateway_log)"

if [[ "$record" == "true" ]]; then
  (
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$workspace_dir/install/setup.bash"
    set -u
    export PYTHONNOUSERSITE=1
    # Records from P's domain, which is the honest view: it captures exactly
    # what the gateway let through plus what P published. A bag taken on A's
    # domain would show topics P never had access to.
    export ROS_DOMAIN_ID="$police_domain"
    cd "$evidence_dir"
    exec ros2 bag record -o rosbag \
      /p_cam/image_raw /p_cam/camera_info \
      /police/speed_estimate /police/speed_violation /clock
  ) >"$evidence_dir/rosbag.log" 2>&1 &
  children+=($!)
  echo "recording to $evidence_dir/rosbag from domain $police_domain"
fi

# The children are in their own process groups now; quieten job notifications.
set +m

# Let the consumers finish DDS discovery before the publisher starts.
sleep 8

# --- A's side: Isaac, bundled Jazzy, Python 3.11, robot domain ---------------
# No system ROS in this shell. The adapter re-execs itself into Isaac's bundled
# Jazzy and rejects leaked AMENT_PREFIX_PATH/PYTHONPATH, so sourcing system ROS
# here would abort the run rather than silently mix two ABIs.
#
# ROS_DOMAIN_ID survives that re-exec: the bootstrap copies the environment and
# replaces only the ROS distribution paths, so A stays on its own domain rather
# than falling back to the default one and finding P again.
echo "starting Isaac on domain $robot_domain (log: $isaac_log)"
env -u AMENT_PREFIX_PATH -u PYTHONPATH -u ROS_DISTRO -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
    OMNI_KIT_ACCEPT_EULA=YES \
    ROS_DOMAIN_ID="$robot_domain" \
    "$isaac_python" "$workspace_dir/tools/isaac_5_1_ros_camera.py" \
    "$stage" \
    ${profile:+--profile "$profile"} \
    --manifest "$manifest" \
    --drive-speed-mps "$speed" \
    --drive-out "$drive_out" \
    --updates "$updates" \
    ${robot_prim:+--robot-prim "$robot_prim"} \
    ${deactivate_physics} \
    --view "$view" \
    --report-gpu-memory \
    ${gui_flag} 2>&1 | tee "$isaac_log"

echo
echo "Isaac finished. Markers to check in $isaac_log:"
echo "  ISAAC_ROS_CAMERA_RENDER_READY   renderer state was read back, not requested"
echo "  ISAAC_ROS_CAMERA_DRIVE          reached_end=True means A completed the route"
echo "  ISAAC_ROS_CAMERA_GPU            VRAM against the RTX 5070 Ti budget"
echo "  ISAAC_ROS_CAMERA_PASS           one render product, one camera"
echo
echo "Observer output is in $ros_log; grep for speed_violation."
echo "Gateway output is in $gateway_log."
echo
echo "If P received nothing, check which of the two it was before blaming DDS:"
echo "  ROS_DOMAIN_ID=$robot_domain ros2 topic list   # empty means A never published"
echo "  ROS_DOMAIN_ID=$police_domain ros2 topic list   # camera topics here means the gateway ran"
