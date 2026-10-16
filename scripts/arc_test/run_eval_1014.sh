#!/usr/bin/env bash
# Fixed test drives for the "NN + fixed limiter" model (rosbags_10_13_26_nn_limiter_v2),
# plus the baseline runs still missing from 10/13, all into ~/AL_scripts/model_eval_1013
# so one analysis compares baseline / old limiter (10/13) / fixed limiter.
#
# Needs: Nav2 running (ros2 launch nav2_stack nav2.launch.py), rover up, charged battery,
# clocks in sync (clockcheck). Resumable: rerun the same script after an abort.
set -euo pipefail
REPO=~/CubeRover_waypoint
OUT=~/AL_scripts/model_eval_1013
BASELINE=~/AL_scripts/rosbags_10_09_26_al_nn
LIMITER_V2=~/AL_scripts/rosbags_10_13_26_nn_limiter_v2

set +u   # the ROS setup files use unset variables
source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"
set -u
cd "$REPO"

newest=$(ls "$LIMITER_V2/from_scratch_weights" | sort -V | tail -1)
echo "limiter_v2 model: $newest"
case "$newest" in
  mlp128_trial5[0-9].pt) ;;
  *) read -r -p "newest checkpoint is $newest (expected trial50+) -- continue anyway? [y/N] " ok
     [[ "$ok" == [yY] ]] || exit 1 ;;
esac

# baseline: limiter off (only its 3 missing runs are driven -- the rest are in runs.csv)
# limiter_v2: fixed limiter on (all 10 tests x 2)
python3 scripts/arc_test/model_eval_test.py --out "$OUT" \
  --config baseline="$BASELINE":off \
  --config limiter_v2="$LIMITER_V2":on \
  --repeats 2

python3 scripts/arc_test/model_eval_analyze.py "$OUT" | tee "$OUT/analysis_$(date +%m%d_%H%M).txt"
