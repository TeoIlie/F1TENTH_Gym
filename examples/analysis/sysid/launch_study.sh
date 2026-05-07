#!/usr/bin/env bash
# Launch a parallel Optuna sysid study.
#
# Usage:
#   ./launch_study.sh <bag.npz> <stage:1|2> [num_workers] [trials_per_worker] [base_params.yaml]
#
# Examples:
#   ./launch_study.sh examples/analysis/bags/rosbag2_2026_05_04-17_54_17_100Hz.npz 1
#   ./launch_study.sh examples/analysis/bags/rosbag2_2026_05_04-17_54_17_100Hz.npz 1 4 125
#   ./launch_study.sh examples/analysis/bags/rosbag2_2026_05_04-17_54_17_100Hz.npz 2 4 125 \
#       gymkhana/envs/params/f1tenth_std_optuna_stage1.yaml
#
# Total trials = num_workers * trials_per_worker. Each worker logs to wandb
# under project 'f1tenth-sysid', grouped by the auto-derived study name.

set -euo pipefail

if [[ $# -lt 2 ]]; then
    echo "Usage: $0 <bag.npz> <stage:1|2> [num_workers=4] [trials_per_worker=125] [base_params.yaml]"
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
if [[ "$STAGE" != "1" && "$STAGE" != "2" ]]; then
    echo "Error: stage must be 1 or 2 (got $STAGE)" >&2
    exit 1
fi
if [[ "$STAGE" == "2" && -z "$BASE_PARAMS" ]]; then
    echo "Error: stage 2 requires base_params yaml (5th argument)" >&2
    exit 1
fi

BAG_STEM=$(basename "$BAG" .npz)
NAME="${BAG_STEM}_stage${STAGE}"
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
echo "  storage:           sqlite:///studies/${NAME}.db"
echo "  wandb:             project f1tenth-sysid, group $NAME"
[[ -n "$BASE_PARAMS" ]] && echo "  base params:       $BASE_PARAMS"
echo "================================================================"

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
echo "Check progress:  sqlite3 studies/${NAME}.db 'SELECT COUNT(*), MIN(value) FROM trials WHERE state=\"COMPLETE\";'"
echo "Waiting for workers..."

wait
echo "All workers done."
echo "Best params YAML: gymkhana/envs/params/f1tenth_std_optuna_stage${STAGE}.yaml"
