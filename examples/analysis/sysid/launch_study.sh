#!/usr/bin/env bash
# Launch a parallel Optuna sysid study.
#
# Usage:
#   ./launch_study.sh <bag.npz|bags.txt> <stage> [num_workers] [trials_per_worker] [base_params.yaml]
#
# First arg is either a single .npz bag, or a .txt file containing one NPZ
# path per line (blank lines and `#`-comments ignored). With a .txt file, the
# study name is derived from the list filename stem, and every worker gets all
# bags via repeated --bag flags.
#
# Total trials = num_workers * trials_per_worker. Each worker logs to a separate
# wandb run under project 'f1tenth-sysid', grouped by the auto-derived study name.
# Stage names come from STAGE_SPACES in search_spaces.py.
#
# Examples:
#   ./launch_study.sh examples/analysis/bags/rosbag2_2026_05_04-17_54_17_100Hz.npz stage1
#   ./launch_study.sh examples/analysis/bags/rosbag2_2026_05_04-17_54_17_100Hz.npz vehicle_dyn 4 125
#   ./launch_study.sh examples/analysis/bags/rosbag2_2026_05_04-17_54_17_100Hz.npz stage2 4 250 \
#       gymkhana/envs/params/f1tenth_std_optuna_stage1.yaml
#   ./launch_study.sh examples/analysis/bags/may26_set.txt stage1 4 125

set -euo pipefail

# Anchor all relative paths (studies/, wandb/) to the repo root so the layout
# is the same regardless of where the script was invoked from.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO_ROOT"

if [[ $# -lt 2 ]]; then
    echo "Usage: $0 <bag.npz|bags.txt> <stage> [num_workers=4] [trials_per_worker=125] [base_params.yaml]"
    exit 1
fi

BAG_ARG="$1"
STAGE="$2"
NUM_WORKERS="${3:-4}"
TRIALS_PER_WORKER="${4:-125}"
BASE_PARAMS="${5:-}"

if [[ ! -f "$BAG_ARG" ]]; then
    echo "Error: not found: $BAG_ARG" >&2
    exit 1
fi

# Two input modes: single .npz, or .txt bag-list file (one NPZ per line). The
# bag-list filename stem becomes the study name so it's both deterministic and
# meaningful (e.g. `may26_set.txt` -> study name `may26_set_stage1`).
BAGS=()
if [[ "$BAG_ARG" == *.txt ]]; then
    while IFS= read -r line || [[ -n "$line" ]]; do
        # Strip leading/trailing whitespace; skip blanks and `#`-comments.
        line="${line#"${line%%[![:space:]]*}"}"
        line="${line%"${line##*[![:space:]]}"}"
        [[ -z "$line" || "$line" == \#* ]] && continue
        BAGS+=("$line")
    done < "$BAG_ARG"
    if [[ ${#BAGS[@]} -eq 0 ]]; then
        echo "Error: bag-list file is empty: $BAG_ARG" >&2
        exit 1
    fi
    NAME_STEM=$(basename "$BAG_ARG" .txt)
else
    BAGS=("$BAG_ARG")
    NAME_STEM=$(basename "$BAG_ARG" .npz)
fi

for b in "${BAGS[@]}"; do
    if [[ ! -f "$b" ]]; then
        echo "Error: bag not found: $b" >&2
        exit 1
    fi
done

# Source of truth: STAGE_SPACES keys in search_spaces.py.
VALID_STAGES=$(python -c "from examples.analysis.sysid.search_spaces import STAGE_SPACES; print(' '.join(STAGE_SPACES))")
if ! [[ " $VALID_STAGES " == *" $STAGE "* ]]; then
    echo "Error: stage '$STAGE' not in STAGE_SPACES ($VALID_STAGES)" >&2
    exit 1
fi

NAME="${NAME_STEM}_${STAGE}"
TOTAL=$(( NUM_WORKERS * TRIALS_PER_WORKER ))

mkdir -p studies

# Build the repeated `--bag` flags consumed by study.py.
BAG_FLAGS=()
for b in "${BAGS[@]}"; do
    BAG_FLAGS+=(--bag "$b")
done

EXTRA_ARGS=()
if [[ -n "$BASE_PARAMS" ]]; then
    EXTRA_ARGS+=(--base-params "$BASE_PARAMS")
fi

echo "================================================================"
if [[ ${#BAGS[@]} -eq 1 ]]; then
    echo "  bag:               ${BAGS[0]}"
else
    echo "  bags (${#BAGS[@]}):"
    for b in "${BAGS[@]}"; do
        echo "    - $b"
    done
fi
echo "  stage:             $STAGE"
echo "  workers:           $NUM_WORKERS"
echo "  trials per worker: $TRIALS_PER_WORKER"
echo "  total trials:      $TOTAL"
echo "  study name:        $NAME"
echo "  storage:           studies/${NAME}.journal (Optuna JournalStorage)"
echo "  wandb:             project f1tenth-sysid, group $NAME"
[[ -n "$BASE_PARAMS" ]] && echo "  base params:       $BASE_PARAMS"
echo "================================================================"

# JournalStorage is race-free for concurrent workers — no DDL, no schema, no
# alembic. Workers can spawn back-to-back against a fresh storage file.
PIDS=()
for ((i=1; i<=NUM_WORKERS; i++)); do
    LOG="studies/${NAME}_w${i}.log"
    echo "[worker $i] logging to $LOG"
    python -m examples.analysis.sysid.study \
        "${BAG_FLAGS[@]}" --stage "$STAGE" --n-trials "$TRIALS_PER_WORKER" \
        --study-name "$NAME" \
        "${EXTRA_ARGS[@]}" \
        > "$LOG" 2>&1 &
    PIDS+=($!)
done

echo "PIDs: ${PIDS[*]}"
echo "Tail logs with:  tail -f studies/${NAME}_w*.log"
echo "Live progress available on wandb dashboard"

# Launch optuna-dashboard against the journal file. It reads live while workers
# write, and we tear it down on exit so it doesn't outlive the script.
DASHBOARD_PID=""
cleanup() {
    if [[ -n "$DASHBOARD_PID" ]] && kill -0 "$DASHBOARD_PID" 2>/dev/null; then
        kill "$DASHBOARD_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if command -v optuna-dashboard >/dev/null 2>&1; then
    # Pre-create the journal so the dashboard never races the first worker write.
    touch "studies/${NAME}.journal"
    DASHBOARD_LOG="studies/${NAME}_dashboard.log"
    DASHBOARD_PORT=8080
    optuna-dashboard --port "$DASHBOARD_PORT" "studies/${NAME}.journal" \
        > "$DASHBOARD_LOG" 2>&1 &
    DASHBOARD_PID=$!
    echo "optuna-dashboard PID: $DASHBOARD_PID  (log: $DASHBOARD_LOG)"
    echo "Dashboard:         http://127.0.0.1:${DASHBOARD_PORT}"
else
    echo "optuna-dashboard not installed; skipping (pip install optuna-dashboard)"
fi

echo "Waiting for workers..."
# `|| true` so one failing worker doesn't trip set -e and abort the others
# mid-run (which would also kill the dashboard via the EXIT trap).
wait "${PIDS[@]}" || true
echo "All workers done."
echo "Best params YAML: gymkhana/envs/params/f1tenth_std_optuna_${STAGE}.yaml"
