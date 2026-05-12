#!/usr/bin/env bash
# Launch a parallel Optuna sysid study.
#
# Usage:
#   ./launch_study.sh <bag.npz> <stage> [num_workers] [trials_per_worker] [base_params.yaml]
#
# Total trials = num_workers * trials_per_worker. Each workers logs to a separate wandb run
# Stage names come from STAGE_SPACES in search_spaces.py.
#
# Examples:
#   ./launch_study.sh examples/analysis/bags/rosbag2_2026_05_04-17_54_17_100Hz.npz stage1
#   ./launch_study.sh examples/analysis/bags/rosbag2_2026_05_04-17_54_17_100Hz.npz vehicle_dyn 4 125
#   ./launch_study.sh examples/analysis/bags/rosbag2_2026_05_04-17_54_17_100Hz.npz stage2 4 250 \
#       gymkhana/envs/params/f1tenth_std_optuna_stage1.yaml
#
# Total trials = num_workers * trials_per_worker. Each worker logs to wandb
# under project 'f1tenth-sysid', grouped by the auto-derived study name.

set -euo pipefail

# Anchor all relative paths (studies/, wandb/) to the repo root so the layout
# is the same regardless of where the script was invoked from.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO_ROOT"

if [[ $# -lt 2 ]]; then
    echo "Usage: $0 <bag.npz> <stage> [num_workers=4] [trials_per_worker=125] [base_params.yaml]"
    exit 1
fi

BAG="$1"
STAGE="$2"
NUM_WORKERS="${3:-4}"
TRIALS_PER_WORKER="${4:-125}"
BASE_PARAMS="${5:-}"

if [[ ! -f "$BAG" ]]; then
    echo "Error: bag not found: $BAG" >&2
    exit 1
fi
# Source of truth: STAGE_SPACES keys in search_spaces.py.
VALID_STAGES=$(python -c "from examples.analysis.sysid.search_spaces import STAGE_SPACES; print(' '.join(STAGE_SPACES))")
if ! [[ " $VALID_STAGES " == *" $STAGE "* ]]; then
    echo "Error: stage '$STAGE' not in STAGE_SPACES ($VALID_STAGES)" >&2
    exit 1
fi

BAG_STEM=$(basename "$BAG" .npz)
NAME="${BAG_STEM}_${STAGE}"
TOTAL=$(( NUM_WORKERS * TRIALS_PER_WORKER ))

mkdir -p studies

EXTRA_ARGS=()
if [[ -n "$BASE_PARAMS" ]]; then
    EXTRA_ARGS+=(--base-params "$BASE_PARAMS")
fi

echo "================================================================"
echo "  bag:               $BAG"
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
        --bag "$BAG" --stage "$STAGE" --n-trials "$TRIALS_PER_WORKER" \
        --study-name "$NAME" \
        "${EXTRA_ARGS[@]}" \
        > "$LOG" 2>&1 &
    PIDS+=($!)
done

echo "PIDs: ${PIDS[*]}"
echo "Tail logs with:  tail -f studies/${NAME}_w*.log"
echo "Live progress available on wandb dashboard"
echo "Waiting for workers..."

wait
echo "All workers done."
echo "Best params YAML: gymkhana/envs/params/f1tenth_std_optuna_${STAGE}.yaml"
