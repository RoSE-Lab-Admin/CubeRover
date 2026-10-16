#!/usr/bin/env bash
# Fixed arcs + maneuvers (model_eval_test.py) for four configurations, all with the old MPPI
# settings (temperature 0.3, wz_std 0.3) and the rate limiter off:
#   kin       kinematics (no model)
#   nn        active learning                        rosbags_10_14_26_nn_old_mppi      trial50
#   nn_sym    active learning + symmetry             rosbags_10_16_26_nn_old_mppi_symmetry trial50
#   nn_hist4  active learning + symmetry + history 4 rosbags_10_16_26_nn_hist4_symmetry trial50
# nn_history_steps is set per configuration from each model file, so the yaml value doesn't matter.
#
# Needs: Nav2 running (ros2 launch nav2_stack nav2.launch.py), rover up, clocks in sync (clockcheck).
# Pause / resume:
#   - battery below --min-voltage (13.7 V): it stops before the next run and waits for Enter
#   - touch ~/AL_scripts/model_eval_1016/PAUSE  -> pauses before the next run until the file is deleted
#   - Ctrl-C any time, then rerun this script: finished runs are skipped
set -eo pipefail
REPO=~/CubeRover_waypoint
OUT=~/AL_scripts/model_eval_1016
W=from_scratch_weights/mlp128_trial50.pt
NN=~/AL_scripts/rosbags_10_14_26_nn_old_mppi/$W
NN_SYM=~/AL_scripts/rosbags_10_16_26_nn_old_mppi_symmetry/$W
NN_HIST=~/AL_scripts/rosbags_10_16_26_nn_hist4_symmetry/$W

for m in "$NN" "$NN_SYM" "$NN_HIST"; do
  [[ -f "$m" ]] || { echo "missing $m -- has that active-learning run finished all 50 trials?"; exit 1; }
done

source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"
cd "$REPO"

python3 scripts/arc_test/model_eval_test.py --out "$OUT" \
  --config kin=kinematics \
  --config nn="$NN" \
  --config nn_sym="$NN_SYM" \
  --config nn_hist4="$NN_HIST" \
  --temperature 0.3 --wz-std 0.3 \
  --repeats 2

python3 scripts/arc_test/model_eval_analyze.py "$OUT" | tee "$OUT/analysis_$(date +%m%d_%H%M).txt"
