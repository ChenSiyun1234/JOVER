#!/usr/bin/env bash
# Run R4 on a Colab CPU VM through the official Colab CLI (in WSL), robust to the local side dying.
#
#   cli_launch.sh start   RUN_ID [TOTAL_TIMESTEPS]   new session, upload, start vm_run.py in the background on the VM
#   cli_launch.sh collect RUN_ID                     pull live files and checkpoints every 45 s; when the VM job has
#                                                    exited, download run.tar.gz, metrics.json and vm.log, then stop
#
# The VM job does not depend on a client connection, and only `collect` stops the session, after the results are
# local. If `collect` dies, run it again: it is idempotent. VM files live only on the VM (/content/vdrive is a
# VM-local stand-in for Drive), so nothing is kept unless `collect` has downloaded it.
set -uo pipefail
MODE=${1:?usage: cli_launch.sh start|collect RUN_ID [TOTAL_TIMESTEPS]}
RUN_ID=${2:?run id}
TOTAL=${3:-3000000}
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SCRATCH=${ROVER_SCRATCH:-$HOME/rover_scratch}
BUNDLE=${ROVER_BUNDLE:-$SCRATCH/rover-r4.zip}
CLI="${COLAB_CLI:-colab} --auth oauth2"
S=rover-rl
VD=/content/vdrive/MyDrive/RoverSim
RUN=/content/run/$RUN_ID
WATCH=$SCRATCH/watch_colab/$RUN_ID/live
OUT=$SCRATCH/colab_runs/$RUN_ID
mkdir -p "$WATCH" "$OUT/checkpoints"
stamp() { date -u +%FT%TZ; }

start() {
    echo "== new $(stamp)"
    timeout 900 $CLI new -s $S --high-mem || timeout 900 $CLI new -s $S || { echo "NEW FAILED"; exit 2; }
    printf 'from pathlib import Path\nfor d in ("%s/code", "%s"):\n    Path(d).mkdir(parents=True, exist_ok=True)\n' "$VD" "$RUN" > "$OUT/prep.py"
    timeout 120 $CLI exec -s $S -f "$OUT/prep.py"
    timeout 300 $CLI upload -s $S "$BUNDLE" "$VD/code/rover-r4.zip" \
        && timeout 120 $CLI upload -s $S "$BUNDLE.sha256" "$VD/code/rover-r4.zip.sha256" \
        && timeout 120 $CLI upload -s $S "$REPO_ROOT/colab/vm_run.py" "/content/vm_run.py" \
        || { echo "UPLOAD FAILED"; timeout 120 $CLI stop -s $S; exit 3; }
    # Start vm_run.py detached from the kernel request: it keeps running with no client attached and
    # writes $RUN/vm.exit when it ends, whatever the outcome.
    cat > "$OUT/starter.py" <<EOF
import os, subprocess, sys
env = dict(os.environ, DRIVE_ROOT="/content/vdrive", RUN_ID="$RUN_ID", TOTAL_TIMESTEPS="$TOTAL", N_ENVS="8",
           CHECKPOINT_EVERY="${CHECKPOINT_EVERY:-250000}", TRAIN_ARGS="${TRAIN_ARGS:-}")
cmd = f"cd /content && {sys.executable} -u /content/vm_run.py > $RUN/vm.log 2>&1; echo \$? > $RUN/vm.exit"
subprocess.Popen(["bash", "-c", cmd], env=env, start_new_session=True,
                 stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print("STARTED", "$RUN_ID", "$TOTAL")
EOF
    timeout 120 $CLI exec -s $S -f "$OUT/starter.py"
    echo "== started $(stamp)"
}

pull_dir() {   # remote dir, local dir, name pattern; skips files already present (they never change)
    local names name
    names=$(timeout 60 $CLI ls -s $S "$1" 2>/dev/null) || return 0
    for name in $names; do
        case "$name" in *.part|*.download|.|..) continue ;; esac
        case "$name" in $3) ;; *) continue ;; esac
        case "$name" in meta.json|progress.csv) ;; *) [ -f "$2/$name" ] && continue ;; esac
        if timeout 120 $CLI download -s $S "$1/$name" "$2/$name.download" >/dev/null 2>&1; then
            mv -f "$2/$name.download" "$2/$name"
        else
            rm -f "$2/$name.download"
        fi
    done
}

collect() {
    echo "== collect $(stamp)"
    while true; do
        pull_dir "$RUN/live" "$WATCH" "@(ep_*.npz|ref_*.npz|meta.json|progress.csv)"
        pull_dir "$RUN/checkpoints" "$OUT/checkpoints" "ppo_*.zip"
        if timeout 60 $CLI ls -s $S "$RUN" 2>/dev/null | grep -qx "vm.exit"; then break; fi
        if ! timeout 60 $CLI sessions 2>/dev/null | grep -q "$S"; then echo "SESSION GONE $(stamp)"; exit 4; fi
        sleep 45
    done
    echo "== vm job exited $(stamp)"
    pull_dir "$RUN/live" "$WATCH" "@(ep_*.npz|ref_*.npz|meta.json|progress.csv)"
    pull_dir "$RUN/checkpoints" "$OUT/checkpoints" "ppo_*.zip"
    status=0
    timeout 120 $CLI download -s $S "$RUN/vm.exit" "$OUT/vm.exit" || status=1
    timeout 300 $CLI download -s $S "$RUN/vm.log" "$OUT/vm.log" || status=1
    for f in run.tar.gz metrics.json; do
        timeout 1800 $CLI download -s $S "$VD/runs/$RUN_ID/$f" "$OUT/$f" && echo "got $f $(stat -c %s "$OUT/$f") bytes" || status=1
    done
    echo "vm exit code: $(cat "$OUT/vm.exit" 2>/dev/null)"
    if [ "$status" = 0 ]; then
        echo "== stop $(stamp)"; timeout 120 $CLI stop -s $S
    else
        echo "DOWNLOAD INCOMPLETE: session left running; run collect again"
    fi
    timeout 60 $CLI usage
    exit $status
}

shopt -s extglob
case "$MODE" in
    start) start ;;
    collect) collect ;;
    *) echo "unknown mode $MODE"; exit 2 ;;
esac
