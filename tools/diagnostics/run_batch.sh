#!/usr/bin/env bash
# Sequential corridor runs, one after another, with a summary at the end.
#
#     bash tools/diagnostics/run_batch.sh 5 nominal_m6_n3 [nominal_m6_n3 ...]
#
# SEQUENTIAL ON PURPOSE. Isaac Sim is single-occupancy machine-wide -- two
# instances can take down the whole host, not just the second one -- so this
# waits for each run to release /tmp/fleet-isaac.lock before starting the next.
# There is no parallel mode and there should not be one.
#
# A batch exists because single runs have been over-read all week. The decoy
# study says 4 of 7 replayed bags arm on the wrong object and the first live
# run armed on the right one; neither of those is a rate, and the only way to
# get one is to run it repeatedly and count.
set -u

COUNT="${1:?usage: run_batch.sh <count> <profile> [profile ...]}"
shift
PROFILES=("$@")
[ ${#PROFILES[@]} -eq 0 ] && PROFILES=(nominal_m6_n3)

REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOGDIR="${BATCH_LOGDIR:-/tmp/corridor-batch-$STAMP}"
mkdir -p "$LOGDIR"
echo "batch of $COUNT x ${PROFILES[*]}  ->  $LOGDIR"

for profile in "${PROFILES[@]}"; do
  for i in $(seq 1 "$COUNT"); do
    # Never start on top of a live session.
    waited=0
    while [ -f /tmp/fleet-isaac.lock ]; do
      pid="$(awk 'NR==1{print $1}' /tmp/fleet-isaac.lock 2>/dev/null)"
      if [ -n "$pid" ] && ! kill -0 "$pid" 2>/dev/null; then
        echo "  stale lock from dead pid $pid -- removing"; rm -f /tmp/fleet-isaac.lock; break
      fi
      sleep 10; waited=$((waited + 10))
      if [ "$waited" -ge 2700 ]; then echo "  lock held 45 min; parking"; exit 2; fi
    done

    log="$LOGDIR/$profile-$i.log"
    echo "=== [$(date +%H:%M:%S)] $profile run $i/$COUNT -> $log"
    bash "$REPO/tools/corridor_profile_run.sh" --profile "$profile" --robot robot1 \
      --domain 67 --allow-contract-fail >"$log" 2>&1
    echo "    exit $?  $(grep -oE '=== [a-z0-9_]+: \*\*(PASS|FAIL)\*\*' "$log" | tail -1)"
  done
done

echo
echo "=== batch complete; summarise with: ==="
echo "  python3 tools/diagnostics/batch_summary.py $LOGDIR"
