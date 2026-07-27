#!/usr/bin/env bash
# One command for the live corridor-twin demonstration.
#
# The demonstration needs two processes that must not share a shell. The
# observer side runs on system Jazzy under Python 3.12; the Isaac adapter runs
# on Isaac's bundled Jazzy under Python 3.11 and re-execs itself into an
# isolated environment that rejects leaked system ROS paths. They meet over
# DDS, which docs/ACTIVATION.md records working: an external system-Jazzy
# consumer received synchronized 640x360 rgb8 frames from this adapter.
#
# The default run drives A at a constant 1.0 m/s. That single unchanged speed
# is legal on the wide approach and illegal once the corridor narrows and the
# limit tightens, so one pass shows both compliance and exactly one violation
# without anyone touching a throttle.
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stage="${STAGE:-$workspace_dir/out/corridor.usda}"
manifest="${MANIFEST:-${stage%.usda}.manifest.json}"
speed="${SPEED_MPS:-1.0}"
profile="${CORRIDOR_PROFILE:-}"
updates="${UPDATES:-3000}"
isaac_python="${ISAAC_PYTHON:-$HOME/isaac/env_isaaclab/bin/python}"
evidence_dir="${EVIDENCE_DIR:-$workspace_dir/out/evidence/live-demo}"

usage() {
  cat <<'USAGE'
Usage: tools/run_demo.sh [--headless] [--no-rviz] [--record]

Environment overrides:
  STAGE=<path.usda>        stage to open        (default out/corridor.usda)
  MANIFEST=<path.json>     surveyed manifest    (default alongside the stage)
  SPEED_MPS=<float>        constant path speed  (default 1.0)
  CORRIDOR_PROFILE=<name>  corridor variant     (default the stage's selection)
  UPDATES=<int>            adapter safety cap   (default 3000)
  ISAAC_PYTHON=<path>      Isaac interpreter    (default ~/isaac/env_isaaclab/bin/python)
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
echo

# --- ROS side: system Jazzy, Python 3.12 -------------------------------------
# Started first, matching the ordering docs/ACTIVATION.md validates for the
# external probe: the consumer is up before the publisher begins.
(
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$workspace_dir/install/setup.bash"
  set -u
  export PYTHONNOUSERSITE=1
  # ros2 launch rejects an empty value outright, so an unset profile has to be
  # omitted rather than passed as corridor_profile:= -- otherwise the whole ROS
  # side fails to start and the run looks like a DDS problem.
  launch_args=(manifest:="$manifest" use_sim_time:=true rviz:="$rviz")
  if [[ -n "$profile" ]]; then
    launch_args+=(corridor_profile:="$profile")
  fi
  exec ros2 launch police_observer live_demo.launch.py "${launch_args[@]}"
) >"$ros_log" 2>&1 &
children+=($!)
echo "observer + display starting (log: $ros_log)"

if [[ "$record" == "true" ]]; then
  (
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$workspace_dir/install/setup.bash"
    set -u
    export PYTHONNOUSERSITE=1
    cd "$evidence_dir"
    exec ros2 bag record -o rosbag \
      /robot/front_camera/image_raw /robot/front_camera/camera_info \
      /police/speed_estimate /police/speed_violation /clock
  ) >"$evidence_dir/rosbag.log" 2>&1 &
  children+=($!)
  echo "recording to $evidence_dir/rosbag"
fi

# The children are in their own process groups now; quieten job notifications.
set +m

# Let the consumers finish DDS discovery before the publisher starts.
sleep 8

# --- Isaac side: bundled Jazzy, Python 3.11 ----------------------------------
# No system ROS in this shell. The adapter re-execs itself into Isaac's bundled
# Jazzy and rejects leaked AMENT_PREFIX_PATH/PYTHONPATH, so sourcing system ROS
# here would abort the run rather than silently mix two ABIs.
echo "starting Isaac (log: $isaac_log)"
env -u AMENT_PREFIX_PATH -u PYTHONPATH -u ROS_DISTRO -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
    OMNI_KIT_ACCEPT_EULA=YES \
    "$isaac_python" "$workspace_dir/tools/isaac_5_1_ros_camera.py" \
    "$stage" \
    ${profile:+--profile "$profile"} \
    --manifest "$manifest" \
    --drive-speed-mps "$speed" \
    --drive-out "$drive_out" \
    --updates "$updates" \
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
