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
# FLEET CANONICAL params, launched by US so the activation retry applies.
#
# Two separate things got conflated and both are now settled:
#
#   * WHICH PARAMS: the fleet canonical, loop closure ON. Pointing this at
#     config/robot1/slam_robot1_corridor.yaml (loop closing OFF) was my change
#     and it is reverted -- the operator observed SLAM behaving worse, and the
#     argument for it ("a single-pass delivery has no loop to close") was wrong,
#     because slam_toolbox also closes against recent scan chains and that is
#     how it corrects accumulated drift.
#   * WHO LAUNCHES: us, not simctl's step. slam_toolbox intermittently misses
#     its own lifecycle service response and then publishes no map at all;
#     simctl reports "no /map after 120 s" and the run dies in the TF wait.
#     simctl's step has no retry and takes no config hook, so handing SLAM back
#     to it lost the verify-and-retry that makes a run reliable.
#
# --corridor-slam opts into the loop-closing-off file for a deliberate A/B.
SLAM_PARAMS="/home/alexmint/Development/robot-fleet/src/yahboomcar-ros2/yahboomcar_config/param/slam_toolbox.yaml"
# U3: which local controller. dwb is the shipped arm; mppi is the comparison.
CONTROLLER="dwb"
# Terminal docking: drive the final approach from the LANDMARK instead of the
# drifting map goal. --no-dock runs transit-only, which is the configuration the
# demonstration must still pass in.
DOCK="--dock"
# The lens is a live browser view of map + scan + the three pose ghosts + the
# landmark detector. It is ON by default because debugging this from JSON after
# the fact repeatedly cost runs: a phantom detection that re-aimed a whole
# mission was invisible in the metrics and obvious on the canvas.
LENS=1
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
# RAISED 420 -> 600 on 2026-08-12, because bring-up is not a constant and 420
# assumed it was. Measured across six runs that night: 135, 146, 149, 243, 253 s.
# At the top of that range a 420 s cap leaves 157 s of nav window, and four runs
# were killed mid-teardown holding complete measurements they were then not
# allowed to call results.
#
# 600 still bounds the thing the cap is FOR -- a session cannot hang holding the
# GPU and the machine-wide lock -- and with the transit window now sized
# independently (below) a normal run finishes in 410-510 s and never reaches it.
SIM_MAX_S=600

# EVERY LIFECYCLE WAIT IS BOUNDED IN WALL CLOCK, NOT IN ITERATIONS.
#
# Both bring-up loops used to count iterations -- `for _ in $(seq 1 14)` with a
# 5 s sleep -- on the assumption that an iteration costs about 5 s. On run
# 20260813-002222 each `ros2 lifecycle get` blocked for ~13 s and returned
# nothing, so 14 iterations became 255 s and the loop sailed past every bound
# anyone thought it had. A deadline in seconds cannot do that.
#
# 75 s LOOKED generous -- bt_navigator bonded 3.5 s after launch on that run,
# and the healthy cluster reaches ACTIVE inside ~20 s. Measured across 17
# bring-ups on 2026-08-13 it was the worst number available, because the
# distribution is bimodal with nothing in between:
#
#     fast : 9 10 10 11 12 13 13 14 15 16 18 s   (11 of 17)
#     slow : 85 86 87 88 88 89 s                 ( 6 of 17)
#
# 75 sits in the empty gap. It is far above every fast bring-up, so it never
# saves time; and just below every slow one, so a slow bring-up is GUARANTEED
# to burn a full 75 s attempt and retry -- and 7 of 27 runs that day died as
# infrastructure, five of them here.
#
# 110 s clears the measured slow mode by 21 s. It cannot make a fast run slower
# (the loop exits on the bond, not on the deadline) and it turns the common
# slow case from "fail, retry, sometimes fail again" into "succeed once".
#
# The bimodality itself is unexplained -- 9-18 s or 85-89 s and never between
# suggests a discrete stall inside Nav2 bring-up rather than load. Worth
# finding; this does not find it, it stops the deadline from being placed in
# the one interval where it does harm.
LIFECYCLE_DEADLINE_S=110

# How much post-arrival data the transit recorder keeps after the nav gate has
# returned. The recorder's job is to outlive the gate, not to outlast it by two
# minutes: 15 s covers the settle and any late map update without paying for
# the rest of a 210 s window the robot finished with at t+60.
RECORDER_SETTLE_S=15

# And every ros2 CLI call gets a timeout, for the same reason. The CLI depends
# on the ros2 daemon; simctl stops the daemon at the end of every run, so every
# corridor run starts against a cold one. simctl's own comment (simctl:396-406)
# says not to trust `ros2 lifecycle get` here -- these loops now corroborate
# with it rather than depend on it.
ROS_CLI_TIMEOUT_S=3

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
    --controller) CONTROLLER="$2"; shift 2 ;;
    --no-dock) DOCK=""; shift ;;
    --no-lens) LENS=0; shift ;;
    --corridor-slam) SLAM_PARAMS="$REPO/config/robot1/slam_robot1_corridor.yaml"; shift ;;
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
# ONE DIRECTORY PER RUN, and the directory name IS the correlation id.
#
# Every artifact used to be named `<what>-<robot>-<profile>.<ext>` in one flat
# directory, so each run overwrote the last -- except on the paths where a run
# died before writing, which left the PREVIOUS run's file in place and made the
# directory read as though this run had succeeded. It was demonstrably mixed:
# map-robot1-nominal_m6_n3.pgm saved at 12:48 sat beside a gate JSON written at
# 13:16, and nav-launch-...-attempt2.log from 03:39 was older than -attempt1.log
# from 13:12, because an attempt counter is not a session id.
#
# Mixing is now structurally impossible rather than merely discouraged, and
# `latest-<robot>-<profile>` keeps the convenience of a stable path (the fleet's
# `latest-d<domain>` precedent, _session_record.py:67).
RUN_ID="$(date +%Y%m%d-%H%M%S)-$ROBOT-$PROFILE"
RUN_DIR="$EVIDENCE/$RUN_ID"
RUN_JSON="$RUN_DIR/run.json"
mkdir -p "$RUN_DIR"
# Stamped ONCE, before anything starts. run.json cannot serve as this: it is
# rewritten throughout the run, so it is always newer than the session bag and
# `find -newer` matched nothing -- the startup criterion went unmeasured on the
# first run that needed it.
RUN_START_MARKER="$RUN_DIR/.started"
: > "$RUN_START_MARKER"
ln -sfn "$RUN_ID" "$EVIDENCE/latest-$ROBOT-$PROFILE"

# The manifest helpers. Fail-open on every one of them: recording a problem
# must never become the problem.
manifest() { python3 "$REPO/tools/run_manifest.py" set --path "$RUN_JSON" "$@" || true; }

PHASE="startup"
phase() {
  PHASE="$1"
  local now elapsed
  now=$(date +%s)
  elapsed=$(( now - ${SESSION_START_S:-now} ))
  printf '%s\n' "$1" > "$RUN_DIR/.phase" 2>/dev/null || true
  printf '%s +%ss %s\n' "$(date +%H:%M:%S)" "$elapsed" "$1" \
    >> "$RUN_DIR/phases.log" 2>/dev/null || true
  echo ""
  echo "=== [$(date +%H:%M:%S) +${elapsed}s] $1 ==="
}

# The launch log that best explains a death in the current phase. Newest
# attempt first, because the last one is the one that was running.
diagnosis_log() {
  local candidate
  for candidate in "$RUN_DIR/nav-launch-attempt2.log" \
                   "$RUN_DIR/nav-launch-attempt1.log" \
                   "$RUN_DIR/slam-attempt2.log" \
                   "$RUN_DIR/slam-attempt1.log" \
                   "$RUN_DIR/contract.txt"; do
    [ -s "$candidate" ] && { printf '%s' "$candidate"; return; }
  done
}

write_diagnosis() {
  local why="$1" log elapsed
  log="$(diagnosis_log)"
  elapsed=$(( $(date +%s) - ${SESSION_START_S:-$(date +%s)} ))
  python3 "$REPO/tools/run_manifest.py" diagnose --path "$RUN_JSON" \
    --why "$why" --phase "${PHASE:-unknown}" --elapsed-s "$elapsed" \
    ${log:+--log "$log"} || true
}

manifest_error() { python3 "$REPO/tools/run_manifest.py" error --path "$RUN_JSON" --message "$1" || true; }
classify() {
  python3 "$REPO/tools/run_manifest.py" classify --path "$RUN_JSON" \
    --classification "$1" --cause "${2:-}" || true
}
digest() { python3 "$REPO/tools/run_manifest.py" digest --file "$1" 2>/dev/null || echo ""; }
# INFRASTRUCTURE, said once. Every exit-3 site classifies before it leaves, so
# a rerun is a fact in the artifact rather than an exit code somebody has to
# have witnessed.
rerun() {
  echo "**INFRASTRUCTURE: $1**" >&2
  classify rerun "$1"
  exit 3
}

# robot1's runner reads the SAME env var name as robot2's -- YAHBOOM_ARENA_USD
# for sim_runner.py:41-46, RASPTANK_ARENA_USD for the rasptank runner. Both are
# exported; each runner reads only its own, so this is harmless and keeps one
# code path (fleet F9: simctl exports neither back to us).
export YAHBOOM_ARENA_USD="$ARENA"

export ROS_DOMAIN_ID="$DOMAIN"
export RASPTANK_ARENA_USD="$ARENA"
export PYTHONNOUSERSITE=1

# THE RUNNER'S OWN LOG, kept. Everything this script prints -- which SLAM
# attempt, whether bt_navigator was seen ACTIVE and when, what the watchdog did
# -- existed only in whatever terminal ran it. The 16:23 run could not be
# diagnosed afterwards for exactly that reason: bt_navigator rejected the goal
# as inactive, and whether the runner had waited for it was unanswerable.
exec > >(tee -a "$RUN_DIR/runner.log") 2>&1

phase "corridor profile run: $PROFILE ${GATED:+(GATED)}${GATED:-(reported only)}"
echo "  domain : $ROS_DOMAIN_ID"
echo "  robot  : $ROBOT"
echo "  arena  : $ARENA"
echo "  run    : $RUN_DIR"

# WHICH SCENARIO, not merely which paths. The arena and the manifest are hashed
# because they came apart once and no artifact could show it: runs after the
# rescale loaded a 12 m arena while planning from a 0.30-scale manifest, and
# every number they produced was about a world nobody meant to run.
manifest \
  --set "run_id=$RUN_ID" \
  --set "started_utc=$(date -u +%Y-%m-%dT%H:%M:%S+00:00)" \
  --set "robot=$ROBOT" \
  --set "profile=$PROFILE" \
  --set "domain=$DOMAIN" \
  --set "controller=$CONTROLLER" \
  --set "arena=$ARENA" \
  --set "arena_sha256=$(digest "$ARENA")" \
  --set "manifest=$MANIFEST" \
  --set "manifest_sha256=$(digest "$MANIFEST")" \
  --set "slam_params=${SLAM_PARAMS:-}" \
  --set "slam_params_sha256=$(digest "${SLAM_PARAMS:-/nonexistent}")" \
  --set "flags={\"gated\":$([ -n "$GATED" ] && echo true || echo false),\"dock\":$([ -n "$DOCK" ] && echo true || echo false),\"lens\":$([ "$LENS" = 1 ] && echo true || echo false),\"rviz\":$([ "$RVIZ_FLAG" = "--no-rviz" ] && echo false || echo true),\"allow_contract_fail\":$([ "$ALLOW_CONTRACT_FAIL" = 1 ] && echo true || echo false)}" \
  --set "sim_max_s=$SIM_MAX_S"
python3 - "$RUN_JSON" "$REPO" <<'PY' || true
import json, sys, pathlib
sys.path.insert(0, str(pathlib.Path(sys.argv[2]) / "tools"))
from run_manifest import git_commit, merge
merge(sys.argv[1], {"git": git_commit(pathlib.Path(sys.argv[2]))})
PY

# THE ARENA MUST BE THE SCENARIO THE RUN WILL PLAN, and this is checked before
# a single second of Isaac is spent on it.
#
# On 2026-08-12 it was not. The arenas on disk were the unscaled 12 m scene --
# B at (16.79, -8.0), no landmark post in the stage at all -- while the nav gate
# planned from the 0.30-scale manifest and put the goal at (4.11, -2.93).
# Nothing failed: the goal was accepted, Nav2 drove to it and reported
# SUCCEEDED, and the run recorded a 5.754 m delivery error and a landmark
# "confirmed" at 1.06 m in a stage that contains no post.
#
# Correcting the two defaults that caused it is not enough by itself; the next
# drift will come from somewhere else. This catches the class.
#
# .venv, not python3: `pxr` lives in this repo's venv, and the ROS workspace
# this script sources does not carry it.
phase "precondition: the arena is the scenario"
if ! "$REPO/.venv/bin/python" "$REPO/tools/check_arena_matches_manifest.py" \
     --arena "$ARENA" --manifest "$MANIFEST" --profile "$PROFILE" \
     --json "$RUN_DIR/arena-check.json"; then
  rerun "the arena and the manifest are different scenarios (see $RUN_DIR/arena-check.json)"
fi

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

# THE LAST RUN'S NODES ARE STILL ON THIS DOMAIN, and `occupants` cannot see
# them: it looks for the SIMULATOR, and these outlive it.
#
# Found 2026-08-12 14:30, on domain 67: the 13:16 run's own un-namespaced
# `behavior_server` still alive 84 minutes after its session ended, next to a
# `planner_server` from 02:42 (11 h 53 m) and nine `robot2_sim_bringup` launches
# 21-24 h old. behavior_server is the node that executes Spin, and a stale one
# shares node name AND action names with the new run's -- so a goal can be
# answered by a server holding a dead session's costmap and TF.
#
# Un-namespaced only. A /robot2-namespaced stack is a different graph and is
# somebody else's business; refusing on it would block the corridor on a fleet
# session that cannot collide with it.
residents() {
  pgrep -af '/opt/ros/[^ ]*/lib/(nav2_[a-z_]+|slam_toolbox)/|corridor_lens\.py' 2>/dev/null \
    | grep -v -- '__ns:=' || true
}
if [ -n "$(residents)" ]; then
  echo "un-namespaced ROS nodes are already alive and would share domain $DOMAIN:" >&2
  residents >&2
  echo "  A stale behavior_server offers the same recovery actions as this run's" >&2
  echo "  and can command the robot. Reap them, verify they are gone, start again." >&2
  # Infrastructure, not usage: the domain was dirty before this run existed, so
  # it is a rerun with a recorded cause rather than a bad command line.
  rerun "domain $DOMAIN already carries un-namespaced ROS nodes from an earlier run"
fi

# Machine-wide single-occupancy. The occupancy scan above only sees THIS
# machine's process list at one instant; the lock is what serialises sessions
# that start seconds apart. Exit 3 (infrastructure), never a robot result.
# shellcheck disable=SC1091
source "$REPO/tools/isaac_lock.sh"
isaac_lock_acquire "corridor-profile-run $PROFILE (domain $DOMAIN)" \
  || rerun "the machine-wide Isaac lock is held by another session"

set +u
# shellcheck disable=SC1090,SC1091
source "$WS_SETUP"
set -u

nav_pid=""
slam_pid=""
lens_pid=""
recorder_pid=""
probe_pid=""
watchdog_pid=""
WATCHDOG_FLAG="$(mktemp -u)"
stopped=0
teardown_verified=0
teardown() {
  [ "$stopped" = 1 ] && return 0
  stopped=1
  echo "=== stopping $PROFILE on domain $DOMAIN ==="
  [ -n "$nav_pid" ] && kill -TERM "$nav_pid" 2>/dev/null || true
  [ -n "$recorder_pid" ] && kill -TERM "$recorder_pid" 2>/dev/null || true
  [ -n "$probe_pid" ] && kill -TERM "$probe_pid" 2>/dev/null || true
  [ -n "$slam_pid" ] && kill -TERM "$slam_pid" 2>/dev/null || true
  # TERM, never -9: the lens writes its metric history on a clean stop.
  [ -n "$lens_pid" ] && kill -TERM "$lens_pid" 2>/dev/null || true
  [ -n "$watchdog_pid" ] && kill -TERM "$watchdog_pid" 2>/dev/null || true
  "$SIMCTL" stop --domain "$DOMAIN" || true
  # POLL, do not sleep once and hope. A fixed 3 s answered "is it dead?" with
  # "it was not dead 3 s ago" and then said nothing more: the 13:16 run's
  # behavior_server survived its own teardown and was still running 84 minutes
  # later, on the domain the next run would use.
  # CHECK FIRST, then sleep. Sleeping first spent 2 s on every teardown that
  # was already clean, and teardown measured 42.9 s of a 403 s run.
  for _ in $(seq 1 40); do
    [ -z "$(occupants)" ] && [ -z "$(residents)" ] && break
    sleep 0.5
  done
  if [ -n "$(occupants)" ]; then
    echo "!! SESSION NOT DEAD:" >&2; occupants >&2
    manifest_error "teardown left the simulator alive"
    return 1
  fi
  # The nodes THIS run launched are ours to bury, and leaving one behind is a
  # defect in this run, not the next one's problem.
  #
  # ESCALATE. TERM is not enough and three consecutive runs proved it: the same
  # two survive every time. `ros2 launch` does not reliably pass TERM down to
  # nav2_behaviors' behavior_server, and rclpy's own signal handlers shut the
  # ROS context down WITHOUT exiting the process, which is why the lens outlives
  # a clean stop. Politeness first -- the lens writes its metric history on a
  # graceful stop -- then KILL what is left, then verify.
  if [ -n "$(residents)" ]; then
    echo "  nodes still up after TERM; escalating to KILL" >&2
    residents | awk '{print $1}' | while read -r pid; do kill -KILL "$pid" 2>/dev/null || true; done
    sleep 2
  fi
  if [ -n "$(residents)" ]; then
    echo "!! NODES SURVIVED TEARDOWN:" >&2; residents >&2
    residents | while read -r line; do manifest_error "survived teardown: $line"; done
    return 1
  fi
  echo "  verified dead"
  teardown_verified=1
  reap_stale_carb_semaphores
  isaac_lock_release
}

# Isaac leaks one POSIX semaphore per session and never reclaims it.
#
#     /dev/shm/sem.carb-RStringInternals-<pid>
#
# `carb` is Omniverse's Carbonite core. After 30 sessions on 2026-08-13 there
# were 144, all owned by dead processes, and /dev/shm held 304 entries.
#
# WHY THIS IS SWEPT UP RATHER THAN LEFT AS UNTIDY. Immediately before the
# sweep, four consecutive runs died in bring-up with
# `controller_server/get_state service client: async_send_request failed` --
# the lifecycle manager aborting the whole nav stack 2.2 s in. Immediately
# after it, three consecutive runs came up clean with zero races. 4-of-4
# against 0-of-3 is p ~= 0.03, which is suggestive and is NOT a demonstrated
# cause: one thing was changed on a host that had been cycling Isaac all day,
# and a transient that cleared on its own is not excluded. Recorded as a
# correlation.
#
# The leak is worth reaping either way: an unattended session accumulates these
# without bound, and nothing else ever removes them.
#
# ONLY dead owners. The pid is the filename's suffix, and a live one belongs to
# somebody else's session -- possibly a concurrent Isaac this run must not
# touch. Failure is silent by design: a tidy-up that can abort a teardown is
# worse than the litter.
reap_stale_carb_semaphores() {
  local reaped=0 pid
  for path in /dev/shm/sem.carb-RStringInternals-*; do
    [ -e "$path" ] || continue
    pid="${path##*-}"
    case "$pid" in (*[!0-9]*|"") continue ;; esac
    kill -0 "$pid" 2>/dev/null && continue
    rm -f "$path" 2>/dev/null && reaped=$((reaped + 1))
  done
  [ "$reaped" -gt 0 ] && echo "  reaped $reaped stale Isaac semaphores from /dev/shm"
  return 0
}
# THE DEFAULT VERDICT IS `crash`, and that is the point.
#
# Classification is first-wins, so every path that knows what happened to it --
# the exit-3 sites, the watchdog, the normal ending -- has already said so by
# the time this runs, and this writes nothing. What it catches is the path that
# said nothing: a component dying mid-run used to leave a directory holding the
# PREVIOUS run's artifacts and no trace of the death at all. The
# joint-velocities-None crash was invisible exactly this way.
record_exit() {
  classify crash "run ended without a verdict (exit $1)"
  manifest --set "exit_status=$1" --set "teardown_verified=${teardown_verified:-0}"
}
# Removed on EVERY exit, not only on the map-scoring path it was created for.
# Twenty-five of these were sitting in /tmp on 2026-08-13, one per run that
# ended any other way. Defined before its caller and tolerant of an exit that
# happens before the marker is even created.
cleanup_marker() { [ -n "${SESSION_MARKER:-}" ] && rm -f "$SESSION_MARKER" 2>/dev/null; return 0; }
on_exit() { local code=$?; teardown || true; record_exit "$code"; cleanup_marker; }

# WHERE THE RUN IS, WRITTEN DOWN AS IT GOES.
#
# Three of the first twenty-four runs ended with no classification at all --
# every one a hand-kill -- and reconstructing the 2026-08-13 hang meant reading
# two launch logs against a runner log with no clock in it. Both are fixed
# here: every phase banner carries the time and the elapsed seconds, and the
# current phase is on disk so a death that skips the EXIT trap can still say
# where it was.
# INT/TERM MUST EXIT, and until 2026-08-13 they did not.
#
# The handler used to tear down and RETURN, on the theory that the watchdog's
# flag check further down would classify the run. That only holds if the signal
# arrives after the last `rerun` site: fire it inside the bt_navigator loop and
# execution resumes IN THE LOOP, burns its remaining iterations against a dead
# stack, and exits via `rerun "bt_navigator never reached ACTIVE"` -- the wrong
# cause, recorded as the only cause, because classification is first-wins.
#
# It also meant Ctrl-C could not stop a run. The operator pressed it, teardown
# ran, the script carried on, and the session had to be escalated to SIGKILL --
# which skips the EXIT trap and is exactly how a run ends up with no verdict.
on_signal() {
  local signal="$1"
  teardown || true
  if [ -f "$WATCHDOG_FLAG" ]; then
    rm -f "$WATCHDOG_FLAG"
    classify rerun "watchdog killed the session at the ${SIM_MAX_S}s cap, in phase '${PHASE}'"
    write_diagnosis "watchdog at the ${SIM_MAX_S}s cap"
  else
    classify rerun "$signal in phase '${PHASE}' -- operator abort"
    write_diagnosis "$signal (operator abort)"
  fi
  exit 3
}
trap on_exit EXIT
trap 'on_signal INT' INT
trap 'on_signal TERM' TERM

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
  echo "**WATCHDOG: session exceeded the ${SIM_MAX_S}s cap in phase" \
       "'$(cat "$RUN_DIR/.phase" 2>/dev/null || echo unknown)' -- tearing down**" >&2
  kill -TERM $$ 2>/dev/null ) &
watchdog_pid=$!
SESSION_START_S=$(date +%s)
echo "  watchdog armed: ${SIM_MAX_S}s cap covers bring-up AND the transit"

# THE SCAN FILTER NEEDS TO KNOW WHICH ROOM IT IS IN.
#
# The twin's scan_frame_relay drops phase-corrupted lidar revolutions by asking
# whether a beam returns from beyond the wall it should have hit -- against
# `segments_room()`, the stock 4 x 4 m yahboom arena, which this corridor is
# not. Measured across 56 of 62 -isaac-d67 sessions: /scan publishes NOTHING for
# the ~21 s it takes to fill the fail-open window, and then the filter disables
# itself and passes raw scans, corrupted revolutions included, for the rest of
# the run. That blackout is also the window in which SLAM has no scans and
# Nav2's costmaps are empty.
#
# The walls come from the MANIFEST, per profile: same source as the arena.
export SCAN_RELAY_WALLS_JSON="$RUN_DIR/scan-walls.json"
if ! "$REPO/.venv/bin/python" "$REPO/tools/export_scan_walls.py" \
     --manifest "$MANIFEST" --profile "$PROFILE" \
     --out "$SCAN_RELAY_WALLS_JSON"; then
  rerun "could not export the scan filter's wall model from $MANIFEST"
fi
# AND HOW MANY BEAMS COUNT AS A SCAN. The relay's other closed-room constant:
# it wants 200 valid returns of 360, which every beam in a 4 x 4 m box provides
# and an open corridor does not. Measured over 5293 scans of this scene: median
# 175 valid, mean 181, min 72 -- so that gate alone rejects 64.3% of good scans
# before geometry is even considered, which is why the filter still failed open
# after it was handed the right walls.
#
# 120 is measured, not chosen to pass: 96.5% of this scene's scans clear it, and
# the geometry test independently refuses to judge on fewer than 90 comparable
# beams, so the "mostly sentinel is junk" guard this replaces still holds.
export SCAN_RELAY_MIN_VALID_BEAMS=120
manifest --set "scan_walls=$SCAN_RELAY_WALLS_JSON" \
         --set "scan_min_valid_beams=$SCAN_RELAY_MIN_VALID_BEAMS"

phase "simctl start"
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
# --no-slam when we supply our own params: simctl's SLAM step hardcodes its
# launch string and takes no config hook, but slam_launch.py itself declares
# `params_file` (replay_slam_bag.py is the standing precedent for using it). So
# the corridor supplies corridor params without editing a single fleet file.
SLAM_FLAG=""
[ -n "$SLAM_PARAMS" ] && SLAM_FLAG="--no-slam"
"$SIMCTL" start --robot "$ROBOT" --backend isaac --domain "$DOMAIN" \
  --no-patrol $RVIZ_FLAG $SLAM_FLAG || {
  rerun "simctl start failed for $PROFILE"; }

# Contract numbers are PER-ROBOT and do not transfer. robot2 is checked with
# --imu-hz 60; robot1's checker carries its own WANT_HZ (scan 12 / odom_raw 11
# / imu 25, check_isaac_contract.py:51) and takes no rate flags at all.
# The two checkers do not share a CLI. robot2's takes --imu-hz and --json;
# robot1's takes neither -- its flags are only --seconds/--speed/--turn/--domain
# (check_isaac_contract.py:54-58) and it prints a human table, so its artifact
# is text and its verdict is the exit code.
if [ "$ROBOT" = robot1 ]; then
  # --speed 0 --turn 0: THE PRECONDITION MUST NOT DRIVE THE MISSION'S ROBOT.
  #
  # This is the startup circle, and it was never navigation. check_isaac_contract
  # drives vx 0.12 / wz 0.3 for the first half of its window to prove /cmd_vel
  # moves the robot -- a 0.4 m-radius arc, 15 s at the default --seconds 30 --
  # published straight to /cmd_vel, bypassing the governor, before SLAM or Nav2
  # exist. Measured in the bag of run 20260812-164717: /cmd_vel carries 802
  # moving commands at a constant (0.120, 0.300) from t=16.31 s to t=31.28 s,
  # integrating to 182 deg and 1.27 m, while /cmd_vel_raw's FIRST message of any
  # kind is at t=86.05 s. Ground truth turns 253 deg over 1.06 m and ends 0.2 m
  # behind spawn.
  #
  # Three sessions read that as a Nav2 recovery, a stale behavior_server, and a
  # DWB critic. It is the health check doing exactly what it says it does.
  #
  # WHAT IS LOST, stated rather than hidden: this run no longer proves /cmd_vel
  # moves the robot. That proof exists twice elsewhere and closer to the metal --
  # build_corridor_arena's forward-sign gate commands 0.2 m/s and measures ground
  # truth on every arena build, and the transit itself moves the robot 7 m. The
  # corridor also overrides this checker's verdict on every run
  # (--allow-contract-fail, scan runs 14-16 Hz against a declared 12), so what it
  # contributes here is a RATE REPORT, and a rate report does not need motion.
  # AND IT DOES NOT NEED THIRTY SECONDS. The checker defaults to --seconds 30
  # and measured 38.3-38.7 s of wall clock on every recorded run, for a rate
  # report whose verdict this run overrides every time. 8 s is ~100 scans and
  # ~200 IMU samples at the measured rates -- ample to catch a dead topic or a
  # rate that is wrong by more than a few percent, which is all this gate is
  # being asked. The full-length check remains available with --gate-seconds
  # style overrides if a rate question ever needs the precision.
  CONTRACT_ARGS=(--domain "$DOMAIN" --speed 0.0 --turn 0.0 --seconds 8)
  CONTRACT_OUT="$RUN_DIR/contract.txt"
else
  CONTRACT_ARGS=(--imu-hz 60 --json)
  CONTRACT_OUT="$RUN_DIR/contract.json"
fi
phase "precondition: $ROBOT contract (${CONTRACT_ARGS[*]})"
# stdout only into the JSON: the checker appends a human summary and its
# FAIL lines after the document, which made every artifact unparseable exactly
# when it mattered most -- on the failures.
if ! python3 "$CONTRACT" "${CONTRACT_ARGS[@]}" >"$CONTRACT_OUT" \
     2>"$RUN_DIR/contract.err"; then
  echo "**contract check failed for $ROBOT/$PROFILE; twin not fit to gate**" >&2
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
    rerun "contract precondition failed for $ROBOT/$PROFILE; twin not fit to gate"
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
if [ -n "$SLAM_PARAMS" ]; then
  # VERIFY, and retry once. slam_toolbox's lifecycle activation misses its
  # own service response intermittently -- "failed to send response to
  # /slam_toolbox/change_state (timeout)" -- and a SLAM that configured but
  # never activated publishes no map->odom, so the TF wait below then fails
  # after 120 s and the whole run is lost to a silent bringup miss. Same
  # defect and same bounded remedy as bt_navigator.
  slam_attempt=0
  slam_ready=0
  while [ "$slam_attempt" -lt 2 ] && [ "$slam_ready" != 1 ]; do
    slam_attempt=$((slam_attempt + 1))
    echo "=== slam_toolbox (corridor params: $(basename "$SLAM_PARAMS"), attempt $slam_attempt) ==="
    ros2 launch yahboomcar_config slam_launch.py "params_file:=$SLAM_PARAMS" \
      >"$RUN_DIR/slam-attempt$slam_attempt.log" 2>&1 &
    slam_pid=$!
    slam_deadline=$(( $(date +%s) + LIFECYCLE_DEADLINE_S ))
    while [ "$(date +%s)" -lt "$slam_deadline" ]; do
      # The LOG first, because it is the output that matters and it does not
      # depend on the ros2 daemon. See LIFECYCLE_DEADLINE_S for what the daemon
      # did to this loop's sibling below.
      if grep -q 'Aborting bringup' "$RUN_DIR/slam-attempt$slam_attempt.log" 2>/dev/null; then
        echo "  slam bringup aborted in its own log; not waiting out the deadline"
        break
      fi
      sstate=$(timeout "$ROS_CLI_TIMEOUT_S" ros2 lifecycle get /slam_toolbox 2>/dev/null | head -1) || true
      case "$sstate" in
        # `active*`, NOT `*active*`: the second matches "inactive [2]" as well,
        # and lifecycle_manager reports exactly that while a node is still
        # configuring. See the bt_navigator poll below for what it cost.
        active*) slam_ready=1; echo "  slam_toolbox active (attempt $slam_attempt)"; break ;;
      esac
      sleep 1
    done
    if [ "$slam_ready" != 1 ]; then
      echo "  ** slam_toolbox never reached ACTIVE (last state: ${sstate:-unknown}) **"
      kill -TERM "$slam_pid" 2>/dev/null || true
      for _ in $(seq 1 10); do kill -0 "$slam_pid" 2>/dev/null || break; sleep 0.5; done
    fi
  done
  if [ "$slam_ready" != 1 ]; then
    write_diagnosis "slam_toolbox never came up in $slam_attempt attempts"
    rerun "slam_toolbox never activated in $slam_attempt attempts"
  fi
fi

phase "waiting for the TF chain"
if ! python3 "$REPO/tools/wait_for_tf.py" --target map --source base_footprint --timeout 120; then
  rerun "map->base_footprint never appeared; twin TF is not up"
fi

# Let the box settle between SLAM and Nav2. The lifecycle service timeouts that
# abort this bringup are a contention symptom -- the EKF logs "Failed to meet
# update rate!" in the same window -- and everything was starting at once.
#
# RESTORED to 8 s on 2026-08-13. `86e5a01 perf(run): stop paying for time the
# run does not need` halved it to 4 to make runs shorter, and the comment
# written at the time said the quiet part out loud: "the contention it guards
# against is real and it is what the nav bringup aborts on". It is.
#
# Seven of 27 runs that day -- 26%, and 3 of the last 6 -- died in nav bring-up.
# The chain, read out of the launch logs rather than guessed:
#
#   local_costmap is slow to configure, under exactly this contention
#     -> controller_server comes up late
#     -> bt_navigator activates and waits 1.00 s for its "follow_path" action
#     -> the wait expires, the behaviour tree fails to load
#     -> the lifecycle manager aborts the WHOLE stack
#     -> the runner reports "bt_navigator never reached ACTIVE", three steps
#        downstream of the fault
#
# Both timeouts in that chain are Nav2 parameters and are fenced this session,
# so the settle is the lever available. This is NOT a controlled A/B -- every
# run measured was already at 4 s, so there is no clean before -- and the
# restoration is justified on the trade rather than on a proof: four seconds
# saved per run, against a quarter of runs dying at about 250 s each. That is a
# bad bargain at any plausible failure rate.
#
# Still a deliberate pause rather than a poll. The condition is now partly
# identified (costmap configuration latency) but not measurable from here, and
# the bring-up loops below hold their own wall-clock deadlines, so a settle that
# is still too short costs a bounded retry rather than a hang.
sleep 8

phase "nav stack"
if [ "$ROBOT" = robot1 ]; then
  NAV_LAUNCH="$REPO/config/robot1/robot1_nav_corridor_launch.py"
  case "$CONTROLLER" in
    dwb)  export CORRIDOR_NAV_PARAMS="$REPO/config/robot1/nav2_robot1_corridor.yaml" ;;
    mppi) export CORRIDOR_NAV_PARAMS="$REPO/config/robot1/nav2_robot1_corridor_mppi.yaml" ;;
    *) echo "unknown --controller: $CONTROLLER (dwb|mppi)" >&2; exit 2 ;;
  esac
  echo "  controller: $CONTROLLER ($(basename "$CORRIDOR_NAV_PARAMS"))"
else
  NAV_LAUNCH="fleet_bringup robot2_nav_sim_launch.py"
fi
# BOUNDED RETRY, twice at most. bt_navigator's activation is nondeterministic on
# this box: the lifecycle manager stalls at "Configuring planner_server" and
# aborts bringup, on runs whose TF and /map are both up and identical to runs
# that succeed. The root cause is not found; what IS established is that a fresh
# nav stack usually activates, and CLAUDE.md classes an infrastructure failure as
# a rerun rather than a result. So the stack -- never the robot, never a gate
# threshold -- is restarted once, and the attempt count is recorded.
nav_attempt=0
nav_ready=0
while [ "$nav_attempt" -lt 2 ] && [ "$nav_ready" != 1 ]; do
  nav_attempt=$((nav_attempt + 1))
  [ "$nav_attempt" -gt 1 ] && echo "  ** nav stack attempt $nav_attempt (previous never reached ACTIVE) **"
  # shellcheck disable=SC2086
  ros2 launch $NAV_LAUNCH \
    >"$RUN_DIR/nav-launch-attempt$nav_attempt.log" 2>&1 &
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
  echo "  waiting for bt_navigator to reach ACTIVE (deadline ${LIFECYCLE_DEADLINE_S}s)..."
  nav_deadline=$(( $(date +%s) + LIFECYCLE_DEADLINE_S ))
  nav_log="$RUN_DIR/nav-launch-attempt$nav_attempt.log"
  while [ "$(date +%s)" -lt "$nav_deadline" ]; do
    # THE LOG IS CHECKED FIRST, AND ON EVERY ITERATION.
    #
    # It used to be checked only from inside the `active*)` branch below, so an
    # attempt that aborted without ever reading active was invisible: on run
    # 20260813-002222 the manager wrote "Aborting bringup" three seconds after
    # launch and this loop kept polling for 115 s more. The verdict was already
    # on disk; nothing looked at it.
    if grep -q 'Aborting bringup' "$nav_log" 2>/dev/null; then
      echo "  the manager aborted this bringup (its own log says so); relaunching"
      break
    fi
    # AND THE BOND, which is the other thing that does not need the daemon.
    # bt_navigator prints this the moment it is genuinely up and managed.
    if grep -q 'Creating bond (bt_navigator)' "$nav_log" 2>/dev/null; then
      sleep 2
      if ! grep -q 'Aborting bringup' "$nav_log" 2>/dev/null; then
        nav_ready=1
        echo "  bt_navigator bonded to the manager (attempt $nav_attempt)"
        break
      fi
    fi
    # `|| true` is load-bearing under `set -e`: a bare assignment from a
    # command substitution that FAILS aborts the script, and `ros2 lifecycle
    # get` fails outright while the node is still coming up -- which is exactly
    # when this loop polls. Without it the run died on the first poll and
    # reported nothing, and the runs that appeared to "work" were the ones
    # where bt_navigator happened to be up before the first poll. That is the
    # whole of the nondeterminism this loop was blamed for.
    #
    # `timeout` is load-bearing for a different reason. On 20260813-002222 each
    # of these calls BLOCKED FOR ~13 s AND RETURNED NOTHING while bt_navigator
    # was active and bonded, because the CLI needs the ros2 daemon and simctl
    # stops the daemon at the end of every run. 14 iterations x (13 + 5) became
    # 255 s of silence. simctl's own comment says not to trust this call; the
    # loop now corroborates with it rather than depending on it.
    state=$(timeout "$ROS_CLI_TIMEOUT_S" ros2 lifecycle get /bt_navigator 2>/dev/null | head -1) || true
    case "$state" in
      # `active*`, NOT `*active*`. THE SECOND MATCHES "inactive".
      #
      # `ros2 lifecycle get` prints "active [3]", "inactive [2]",
      # "unconfigured [1]". The old glob matched the middle one, so every time
      # bt_navigator was still configuring this loop declared it ready, the goal
      # went out, and bt_navigator answered "Action server is inactive.
      # Rejecting the goal." Four of 2026-08-12's runs died exactly that way and
      # were read as a flaky bringup race; the stack was telling the truth and
      # the runner was mis-reading it. Measured on run 20260812-222023, whose
      # launch log shows "Configuring bt_navigator" and no activation at all,
      # while this poll reported ready on its first attempt.
      active*)
        # ACTIVE IS NOT ENOUGH ON ITS OWN. bt_navigator reaches active during
        # the transition and the lifecycle manager can still abort the bringup
        # a moment later -- "Failed to change state for node: bt_navigator.
        # Exception: ... async_send_request failed" then "Failed to bring up
        # all requested nodes. Aborting bringup" -- after which the server is
        # inactive again and rejects the goal. The state poll caught the
        # transient and reported a stack that was already dying.
        #
        # The manager's own abort line is definitive, so it is what decides.
        sleep 5
        if grep -q 'Aborting bringup' "$RUN_DIR/nav-launch-attempt$nav_attempt.log" 2>/dev/null; then
          echo "  bt_navigator went active then the manager aborted bringup"
          break
        fi
        nav_ready=1; echo "  bt_navigator active and bringup held (attempt $nav_attempt)"; break ;;
    esac
    sleep 1
  done
  if [ "$nav_ready" != 1 ]; then
    kill -TERM "$nav_pid" 2>/dev/null || true
    for _ in $(seq 1 10); do kill -0 "$nav_pid" 2>/dev/null || break; sleep 0.5; done
  fi
done
if [ "$nav_ready" != 1 ]; then
  tail -5 "$RUN_DIR/nav-launch-attempt$nav_attempt.log" | sed 's/^/    /' >&2
  write_diagnosis "bt_navigator never came up in $nav_attempt attempts"
  rerun "bt_navigator never reached ACTIVE in $nav_attempt attempts (last state: ${state:-unknown})"
fi
NAV_ATTEMPTS="$nav_attempt"

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
    rerun "bring-up used ${elapsed}s of the ${SIM_MAX_S}s cap; no window left to navigate"
  fi
  echo "  nav window: ${NAV_TIMEOUT}s (cap ${SIM_MAX_S}s, bring-up took ${elapsed}s)"
fi
# THE TRANSIT WINDOW IS SIZED TO THE TRANSIT, not to whatever the cap has left.
#
# GATE_SECONDS used to be NAV_TIMEOUT + 10, i.e. a budget remainder, which is
# how a 551 s window came to be requested for a 256 s transit -- and, before the
# rate basis was fixed, how a healthy 11.50 Hz matcher was reported as 5.35 Hz.
# It also made the run's LENGTH depend on how slow bring-up happened to be.
#
# 200 s is measured: A reached its closest approach to B at t+52.4, t+56.4,
# t+58.3 and t+109.9 s across tonight's runs, and everything after that is A
# holding position or driving past. 200 covers the worst of those with 90 s of
# margin. Still capped by the nav window so it can never outlive the watchdog.
TRANSIT_WINDOW_S=200
# The NAV window is what gets capped, and the recorder is then sized from it --
# never the other way round. Capping only the recorder broke the invariant its
# own comment states: it must OUTLIVE the nav gate, or a slow-but-successful
# delivery is truncated by the instrument watching it. Measured on run
# 20260812-182237, where a 200 s recorder sat inside a 429 s nav window.
if [ "$NAV_TIMEOUT" -gt "$TRANSIT_WINDOW_S" ]; then
  echo "  transit window: ${TRANSIT_WINDOW_S}s (nav window ${NAV_TIMEOUT}s is longer than the transit needs)"
  NAV_TIMEOUT="$TRANSIT_WINDOW_S"
fi
: "${GATE_SECONDS:=$((NAV_TIMEOUT + 10))}"
if [ "$LENS" = 1 ]; then
  python3 "$REPO/tools/lens/corridor_lens.py" --domain "$DOMAIN" \
    --manifest "$MANIFEST" \
    --dump "$RUN_DIR/lens.json" \
    >"$RUN_DIR/lens.log" 2>&1 &
  lens_pid=$!
  # Poll /healthz, which the lens serves for exactly this. A fixed sleep here
  # was both too long on a warm box and too short on a cold one.
  for _ in $(seq 1 40); do
    curl -sf --max-time 1 "http://127.0.0.1:8765/healthz" >/dev/null 2>&1 && break
    kill -0 "$lens_pid" 2>/dev/null || break
    sleep 0.25
  done
  echo "=== lens: http://127.0.0.1:8765/  (map, scan, 3 pose ghosts, landmark) ==="
fi

# WHO COMMANDED THAT? Started BEFORE the goal, because the question is about
# what happens before the goal. Nothing in either repository subscribed to
# /behavior_tree_log, so three explanations for A turning on the spot at startup
# have been offered across two sessions and none was ever checked against a log.
GOAL_MARKER="$RUN_DIR/.goal-sent"
rm -f "$GOAL_MARKER"
PROBE_READY="$RUN_DIR/.probe-ready"
rm -f "$PROBE_READY"
python3 "$REPO/tools/corridor_startup_probe.py" --seconds "$GATE_SECONDS" \
  --goal-marker "$GOAL_MARKER" --ready-marker "$PROBE_READY" \
  --out "$RUN_DIR/startup.json" \
  >"$RUN_DIR/startup.log" 2>&1 &
probe_pid=$!
# WAIT for it, do not hope. Its first live run reported goal_at_s = 0.081 --
# which is not "the goal came 81 ms in", it is "the marker was already there
# when I started", so the whole before-the-goal window was missed.
for _ in $(seq 1 40); do
  [ -f "$PROBE_READY" ] && break
  sleep 0.25
done
[ -f "$PROBE_READY" ] || manifest_error "the startup probe never signalled ready"

phase "T3.3a transit recorder (observe-only, ${GATE_SECONDS}s)"
python3 "$REPO/tools/corridor_sim_gate.py" --seconds "$GATE_SECONDS" \
  --profile "$PROFILE" --robot "$ROBOT" $GATED --observe-only \
  --manifest "$MANIFEST" \
  ${CONTRACT_CAVEAT:+--caveat "$CONTRACT_CAVEAT"} \
  --out "$RUN_DIR/gate.json" \
  >"$RUN_DIR/gate.log" 2>&1 &
recorder_pid=$!

phase "T3.3b governed Nav2 goal A->B"
: > "$GOAL_MARKER"
python3 "$REPO/tools/corridor_nav_gate.py" --profile "$PROFILE" --robot "$ROBOT" $GATED $DOCK \
  ${CONTRACT_CAVEAT:+--caveat "$CONTRACT_CAVEAT"} \
  --manifest "$MANIFEST" \
  --timeout "$NAV_TIMEOUT" \
  --out "$RUN_DIR/nav.json" || status=1

# The recorder's own verdict is a gate result too, so it is waited for rather
# than killed -- but a nav gate that ended early must not hang the run behind
# the recorder's full window.
#
# AND IT USED TO. GATE_SECONDS is 210 and the recorder ran every second of it
# whatever the robot did, while measured closest approach happens at t+52 to
# t+110 s. On 20260813-000546 the nav gate returned at +201 s and the run then
# sat behind the recorder for another 14 s; on a fast transit the dead tail is
# most of two minutes.
#
# The window is NOT shortened, because its 200 s is sized on measurement and
# the recorder must outlive the nav gate or a slow-but-successful delivery is
# truncated by the instrument watching it (measured, 20260812-182237). Instead
# the recorder is told the gate is done: `corridor_sim_gate.observe()` handles
# SIGTERM and writes a COMPLETE report, so this is a clean early finish rather
# than a kill. The settle keeps a few seconds of post-arrival data.
phase "transit recorder verdict"
if kill -0 "$recorder_pid" 2>/dev/null; then
  echo "  nav gate returned; letting the recorder settle ${RECORDER_SETTLE_S}s, then closing it"
  sleep "$RECORDER_SETTLE_S"
  kill -TERM "$recorder_pid" 2>/dev/null || true
fi
wait "$recorder_pid" || status=1

# "GOAL NOT ACCEPTED" MEANS TWO DIFFERENT THINGS, and only the robot can say
# which. Both were seen tonight, twenty minutes apart:
#
#   20260812-183327  bt_navigator was still inactive. The goal was refused, the
#                    robot moved 0.13 m, and the run recorded three true numbers
#                    about a robot that was never asked to do anything.
#   20260812-184220  the goal was ACCEPTED -- "Begin navigating from current
#                    location (0.00, 0.00) to (4.11, -2.93)" is in the launch
#                    log -- and A drove 7.865 m to within 0.178 m of the
#                    standoff. What went missing was the ACCEPTANCE RESPONSE,
#                    which corridor_nav_gate.py:270-274 already documents as a
#                    nav failure that never happened.
#
# So the question is not what the gate reported, it is whether the robot moved,
# and the recorder already measured that. Under the gate's own "barely moved"
# threshold this is infrastructure; over it, navigation happened and the lost
# response is recorded as an error against a run that still counts.
if [ -f "$RUN_DIR/nav.json" ] && grep -q '"failure": "goal not accepted"' "$RUN_DIR/nav.json"; then
  # 1.0 m is the transit gate's own "robot barely moved" threshold, read from
  # the same artifact, so the two cannot disagree about what moving means.
  moved=$("$REPO/.venv/bin/python" - "$RUN_DIR/gate.json" <<'PYEOF' 2>/dev/null || echo "0.0 no"
import json, sys
try:
    distance = float(json.load(open(sys.argv[1])).get("ground_truth_distance_m") or 0.0)
except Exception:
    distance = 0.0
print(f"{distance:.3f} {'yes' if distance >= 1.0 else 'no'}")
PYEOF
)
  if [ "${moved##* }" != "yes" ]; then
    rerun "the nav stack rejected the goal as inactive and the robot never moved (${moved%% *} m)"
  fi
  moved="${moved%% *}"
  echo "  nav reported 'goal not accepted' but the robot drove ${moved} m:" >&2
  echo "  the acceptance response was lost, not the goal (nav_gate.py:270-274)" >&2
  manifest_error "acceptance response lost: nav reported 'goal not accepted' while the robot drove ${moved} m"
fi
# Not a gate yet -- U2 measures first and decides after. Printed so the answer
# is in front of whoever watched the run.
kill -TERM "$probe_pid" 2>/dev/null || true
wait "$probe_pid" 2>/dev/null || true
if [ -f "$RUN_DIR/startup.json" ]; then
  echo "=== startup: what was commanded before the goal ==="
  sed -n "/\"summary\"/,/^  }/p" "$RUN_DIR/startup.log" | sed 's/^/    /'
fi
sed 's/^/    /' "$RUN_DIR/gate.log" | tail -20

# SAVE THE MAP OURSELVES when we own SLAM. simctl's map-save step belongs to
# simctl's own SLAM launch; under --no-slam it saves nothing, and the scorer
# then correctly reports that this session produced no map -- which would leave
# every corridor-params run unscoreable.
if [ -n "$SLAM_PARAMS" ]; then
  echo "=== saving the map (we own SLAM) ==="
  OWN_MAP="$RUN_DIR/map"
  timeout 60 ros2 run nav2_map_server map_saver_cli -f "$OWN_MAP" \
    --ros-args -p save_map_timeout:=20.0 \
    >"$RUN_DIR/map-save.log" 2>&1 \
    && echo "  saved $OWN_MAP.yaml" \
    || { echo "  **map save failed; see $RUN_DIR/map-save.log**" >&2
         manifest_error "map save failed"; }
fi

teardown || { status=1; manifest_error "teardown could not verify the session was dead"; }
# The teardown is done; the recording half of the exit trap is not. Handing back
# a bare `trap -` here left every artifact after this point outside the run's
# own record, including its exit status.
trap 'record_exit $?' EXIT
trap - INT TERM

# A traceback in a log is a component that DIED, and it must not be inferred
# from a missing artifact three days later. Recorded as manifest errors; it does
# not by itself decide the classification.
for log in "$RUN_DIR"/*.log; do
  [ -f "$log" ] || continue
  if grep -q 'Traceback (most recent call last)' "$log" 2>/dev/null; then
    manifest_error "traceback in $(basename "$log")"
  fi
done

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
phase "map quality"
SCORER=/home/alexmint/Development/robot-fleet/src/yahboomcar-ros2/tools/score_slam_map.py
if ! python3 "$SCORER" --self-test >"$RUN_DIR/map-selftest.txt" 2>&1; then
  echo "**INFRASTRUCTURE: map scorer self-test FAILED; its verdicts are not trustworthy**" >&2
  status=1
else
  # -newer $SESSION_MARKER, not `ls -t`: a session that saved no map would
  # otherwise silently score the PREVIOUS session's, and report its verdict as
  # this run's. That happened -- two consecutive runs reported an identical
  # 0.800 m duplicate-wall extent because the second produced no map at all.
  if [ -n "$SLAM_PARAMS" ] && [ -f "$RUN_DIR/map.yaml" ]; then
    SAVED_MAP="$RUN_DIR/map.yaml"
  else
  SAVED_MAP=$(find \
    "$HOME"/Development/MicroROS/MicroROS-assets/logs/sessions \
    "$HOME"/Development/robot-fleet/src/MicroROS/MicroROS-assets/logs/sessions \
    -maxdepth 2 -name 'map-*.yaml' -path "*-d$DOMAIN/*" -newer "$SESSION_MARKER" \
    2>/dev/null | head -1)
  fi
  rm -f "$SESSION_MARKER"
  if [ -z "$SAVED_MAP" ]; then
    echo "**THIS session saved no map on domain $DOMAIN: SLAM produced nothing to score**" >&2
    status=1
  else
    cp "$SAVED_MAP" "${SAVED_MAP%.yaml}.pgm" "$RUN_DIR/" 2>/dev/null || true
    # MASK THE SCENE'S OWN DOUBLE SURFACES FIRST, or the metric convicts the
    # corridor for being a corridor. It asks whether anything stands within
    # 0.40 m of the outermost wall, which is a fair question of the plain 4x4 m
    # room it was tuned in and a wrong one here: ADR 0019's corner screen stands
    # 0.33 m off the east wall and ADR 0018's stub protrudes from it. Measured
    # on the authored perfect-SLAM oracle -- no sensor, no drift -- the metric
    # reads 0.340 m against a 0.20 m limit. Masked, it reads 0.000 m.
    #
    # Masked, not subtracted: a subtracted floor keeps the blind spot AND moves
    # the threshold, so a run's number stops being comparable with every number
    # recorded before it. The polygons come from the MANIFEST, the same source
    # as the arena, and the blind spot is 0.42% of the map's cells.
    SCORED_MAP="$SAVED_MAP"
    if "$REPO/.venv/bin/python" "$REPO/tools/mask_authored_double_surface.py" \
         --map "$SAVED_MAP" --manifest "$MANIFEST" --profile "$PROFILE" \
         --frame robot_start --out "$RUN_DIR/map-masked.yaml" \
         --json "$RUN_DIR/map-mask.json"; then
      SCORED_MAP="$RUN_DIR/map-masked.yaml"
    else
      echo "  **masking failed; scoring the RAW map, which reads the corridor's" >&2
      echo "    own authored structures as duplicate wall**" >&2
      manifest_error "double-surface masking failed; map scored unmasked"
    fi
    echo "  scoring $SCORED_MAP"
    python3 "$SCORER" --map "$SCORED_MAP" \
      --json "$RUN_DIR/map-score.json" \
      | tee "$RUN_DIR/map-score.txt" || status=1
  fi
fi

# THE STARTUP CRITERION, from ground truth, on every run. The circle was
# diagnosed three times from the wrong signal -- twice from what something
# commanded and once from a topic the offending driver was not using -- so its
# acceptance is measured from what the robot actually did.
SESSION_BAG=$(find \
  "$HOME"/Development/MicroROS/MicroROS-assets/bags \
  "$HOME"/Development/robot-fleet/src/MicroROS/MicroROS-assets/bags \
  -maxdepth 1 -name "*-isaac-d$DOMAIN" -newer "$RUN_START_MARKER" 2>/dev/null | sort | tail -1)
if [ -n "$SESSION_BAG" ]; then
  echo "=== startup criterion (ground truth, $(basename "$SESSION_BAG")) ==="
  "$REPO/.venv/bin/python" "$REPO/tools/startup_acceptance.py" \
    --bag "$SESSION_BAG" --gate "$RUN_DIR/gate.json" \
    --out "$RUN_DIR/startup-acceptance.json" | sed 's/^/    /' || status=1
  manifest --set "session_bag=$SESSION_BAG"
else
  echo "  **no session bag found on domain $DOMAIN; startup criterion unmeasured**" >&2
  manifest_error "no session bag found; the startup criterion was not measured"
fi

# A run the watchdog killed is INFRASTRUCTURE, never a verdict about the robot:
# it was stopped mid-transit by the clock, so its gate failures describe an
# interrupted run and nothing else.
if [ -f "$WATCHDOG_FLAG" ]; then
  rm -f "$WATCHDOG_FLAG"
  echo "=== $PROFILE: **INFRASTRUCTURE -- killed at the ${SIM_MAX_S}s cap, not a result** ===" >&2
  classify rerun "watchdog killed the session at the ${SIM_MAX_S}s cap, in phase '${PHASE}'"
  write_diagnosis "watchdog at the ${SIM_MAX_S}s cap"
  exit 3
fi
# A RESULT, red or green. `pass` and `classification` are separate fields on
# purpose: a red gate is a statement about the robot, a rerun and a crash are
# statements about the session, and collapsing them is how an interrupted run
# came to be quoted as a verdict.
classify result "$([ "$status" = 0 ] && echo "gates green" || echo "at least one gate red")"
manifest \
  --set "pass=$([ "$status" = 0 ] && echo true || echo false)" \
  --set "stopped_utc=$(date -u +%Y-%m-%dT%H:%M:%S+00:00)" \
  --set "duration_s=$(( $(date +%s) - SESSION_START_S ))" \
  --set "nav_attempts=${NAV_ATTEMPTS:-0}" \
  --set "gate_seconds=${GATE_SECONDS:-0}" \
  --set "nav_timeout_s=${NAV_TIMEOUT:-0}" \
  --set "contract_caveat=${CONTRACT_CAVEAT:-}"

if [ "$status" = 0 ]; then
  echo "=== $PROFILE: PASS ==="
else
  echo "=== $PROFILE: **FAIL** (artifacts kept under $RUN_DIR) ==="
fi
echo "  run record: $RUN_JSON"
exit "$status"
