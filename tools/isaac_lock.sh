#!/usr/bin/env bash
# Machine-wide single-occupancy lock for Isaac Sim.
#
#     source tools/isaac_lock.sh
#     isaac_lock_acquire "corridor-profile-run nominal_m6_n3"   # blocks, polls, exits 3 on timeout
#     ...
#     isaac_lock_release                                        # idempotent; safe in traps
#
# WHY THIS FILE EXISTS
# --------------------
# CLAUDE.md's unattended rules require acquiring `/tmp/fleet-isaac.lock` before
# any `simctl start --backend isaac`. The v2 plan's F12 records that the lock is
# "doc-only protocol, zero code" across the whole fleet: the protocol has been
# written down three times and implemented never. simctl's only guard is a
# per-domain ROS discovery check, which a second Isaac on a DIFFERENT domain
# sails straight past -- and two Isaac instances can take down the whole
# machine, killing every other session's work, not just this one.
#
# So the lock is real here. It is deliberately a plain file rather than flock(1):
# the holder is often a background pipeline whose shell exits while the session
# lives on, so the lock has to outlive its acquiring shell and be judged by
# whether its recorded PID is still alive, not by an open file descriptor.
#
# STALE LOCKS: a lock whose PID is dead is stale and may be removed -- that is
# CLAUDE.md's rule, and it is what makes a crashed session recoverable without a
# human. A lock whose PID is ALIVE is never removed, no matter how old.

ISAAC_LOCK_FILE="${ISAAC_LOCK_FILE:-/tmp/fleet-isaac.lock}"
ISAAC_LOCK_POLL_S="${ISAAC_LOCK_POLL_S:-300}"     # 5 min between polls
ISAAC_LOCK_MAX_WAIT_S="${ISAAC_LOCK_MAX_WAIT_S:-2700}"  # 45 min, then park

_isaac_lock_held_by_us=0

# Holder PID, or empty if the lock is absent or unreadable.
isaac_lock_holder_pid() {
  [ -f "$ISAAC_LOCK_FILE" ] || return 0
  awk -F= '/^pid=/{print $2; exit}' "$ISAAC_LOCK_FILE" 2>/dev/null | tr -d ' '
}

isaac_lock_status() {
  if [ ! -f "$ISAAC_LOCK_FILE" ]; then
    echo "free"
    return 0
  fi
  local pid
  pid="$(isaac_lock_holder_pid)"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    echo "held"
  else
    echo "stale"
  fi
}

# Acquire, polling while another LIVE session holds it. Exits 3 (the runner's
# infrastructure code) rather than returning, because every caller treats a
# lock timeout as "park this unit", never as a robot result.
isaac_lock_acquire() {
  local owner="${1:-unnamed session}" waited=0 state pid

  while :; do
    state="$(isaac_lock_status)"
    case "$state" in
      free) break ;;
      stale)
        pid="$(isaac_lock_holder_pid)"
        echo "  isaac lock: removing STALE lock (pid ${pid:-?} is dead)"
        rm -f "$ISAAC_LOCK_FILE"
        break
        ;;
      held)
        pid="$(isaac_lock_holder_pid)"
        if [ "$waited" -ge "$ISAAC_LOCK_MAX_WAIT_S" ]; then
          echo "**isaac lock: still held by pid $pid after ${waited}s -- PARKING this unit**" >&2
          sed 's/^/    /' "$ISAAC_LOCK_FILE" >&2 2>/dev/null || true
          return 3
        fi
        echo "  isaac lock: held by pid $pid; polling again in ${ISAAC_LOCK_POLL_S}s (waited ${waited}s)"
        sleep "$ISAAC_LOCK_POLL_S"
        waited=$((waited + ISAAC_LOCK_POLL_S))
        ;;
    esac
  done

  # Belt and braces: the lock says nobody owns Isaac, so nothing Isaac-shaped
  # should be running. If something is, the lock is lying and we do not start.
  #
  # Matched on the process EXECUTABLE, not the command line. A cmdline match
  # false-positived three times on shells that merely MENTIONED the pattern --
  # this repo's own paths contain "omniverse", and any caller that types
  # sim_runner.py on the same line puts it in their own cmdline. Excluding
  # ancestors is not enough either, because such a shell can be a sibling. A
  # real twin runs as python3 or kit; a shell pretending to be one runs as
  # bash. That distinction is exact and does not depend on process topology.
  local strays=""
  local candidate comm
  for candidate in $(pgrep -f 'sim_runner\.py|rasptank_twin_runner\.py|isaac_5_1_ros_camera|isaac-sim|/kit/kit' 2>/dev/null); do
    [ "$candidate" = "$$" ] && continue
    comm="$(cat "/proc/$candidate/comm" 2>/dev/null || true)"
    case "$comm" in
      bash|sh|dash|zsh|""|pgrep|grep) continue ;;
    esac
    strays="$strays$candidate $comm"$'\n'
  done
  if [ -n "$strays" ]; then
    echo "**isaac lock: lock is free but Isaac-shaped processes are running -- refusing**" >&2
    printf '%s' "$strays" | sed 's/^/    /' >&2
    return 3
  fi

  printf 'pid=%s\nowner=%s\nstarted=%s\nhost=%s\n' \
    "$$" "$owner" "$(date -Iseconds)" "$(hostname)" > "$ISAAC_LOCK_FILE"
  _isaac_lock_held_by_us=1
  echo "  isaac lock: acquired by pid $$ ($owner)"
}

# Idempotent, and safe to call from a trap on any exit path. Only ever removes
# a lock this process owns -- releasing somebody else's lock would be worse
# than never having taken one.
isaac_lock_release() {
  [ "$_isaac_lock_held_by_us" = 1 ] || return 0
  local pid
  pid="$(isaac_lock_holder_pid)"
  if [ "$pid" = "$$" ]; then
    rm -f "$ISAAC_LOCK_FILE"
    echo "  isaac lock: released by pid $$"
  else
    echo "  isaac lock: NOT releasing -- held by pid ${pid:-?}, not us ($$)" >&2
  fi
  _isaac_lock_held_by_us=0
}
