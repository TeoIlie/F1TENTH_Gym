#!/usr/bin/env bash
# Run beta_r_avg_plot.py for all learned controller configurations sequentially.
set -e

cd "$(dirname "$0")/../.."
SCRIPT="python examples/analysis/beta_r_avg_plot.py"

# Stanley baseline (generates recovery states for learned controller comparison)
$SCRIPT --controller_type stanley --desc "stanley"

# STMPC baseline
$SCRIPT --controller_type --desc "Single-track MPC controller with acados + CasAdi, ported from ForzaETH"

# Drift models
# $SCRIPT --controller_type learned --learned_type drift --run_id 178a1a5l --desc "drift model - CW & CCW on Drift_large, with sparse_width_obs = True"
# $SCRIPT --controller_type learned --learned_type drift --run_id iza03vyw --desc "drift model - CW & CCW on Drift_large, with sparse_width_obs = False"
$SCRIPT --controller_type learned --learned_type drift --run_id bsoh5xyb --desc "drift model - CW & CCW on Drift_large, with sparse_width_obs = True"

# Recover models
$SCRIPT --controller_type learned --learned_type recover --run_id p13d1mdz --desc "recovering model - original with Euclidean reward"
$SCRIPT --controller_type learned --learned_type recover --run_id irdqwnhp --desc "recovering model - no Euclidean reward"
$SCRIPT --controller_type learned --learned_type recover --run_id 8m5f957h --desc "recovering model - Euclidean reward, larger beta, r ranges"
$SCRIPT --controller_type learned --learned_type recover --run_id 50x16c1d --desc "recovering model - Euclidean reward, curriculum learning, smaller beta, r ranges"
$SCRIPT --controller_type learned --learned_type recover --run_id qhj88o3r --desc "recovering model - Euclidean reward, curriculum learning, larger beta, r ranges"
$SCRIPT --controller_type learned --learned_type recover --run_id sysea5vx --desc "recovering model - no Euclidean reward, curriculum learning, 200 success reward, smaller beta, r ranges"
$SCRIPT --controller_type learned --learned_type recover --run_id qh54psj2 --desc "recovering model - original with Euclidean reward, no curriculum, small beta,r variations"
$SCRIPT --controller_type learned --learned_type recover --run_id koa3rljd --desc "recovering model - Euclidean reward, curriculum learning, larger beta, r ranges"
$SCRIPT --controller_type learned --learned_type recover --run_id w7bkr26u --desc "recovering model - no Euclidean reward, curriculum learning, 200 success reward, smaller beta, r ranges"
$SCRIPT --controller_type learned --learned_type recover --run_id g3w88oqx --desc "recovering model - drift model 178a1a5l retrained with Fine-Tuning with Fresh Optimizer + LR Reset + log_std reset with --m f. No curriculum learning, small beta-r initial ranges, no Euclidean reward"
$SCRIPT --controller_type learned --learned_type recover --run_id bwcm7l05 --desc "recovering model - drift model 178a1a5l retrained by loading and continuing training with --m c. No curriculum learning, small beta-r initial ranges, no Euclidean reward"
$SCRIPT --controller_type learned --learned_type recover --run_id pbmnxwcc --desc "recovering model - drift model 178a1a5l retrained with Fine-Tuning with Fresh Optimizer + LR Reset + log_std reset + Critic Reinitialization. No curriculum learning, small beta-r initial ranges, no Euclidean reward"

echo "All evaluations complete!"
