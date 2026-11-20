#!/bin/bash
# Builds the bench install (install_bench/, an overlay on install/) for scripts/bench/run_bench.py.
# The live install/ is never touched. The bench copy of nav2_param2.yaml runs the
# controller on plain kinematics on the CPU (rover_sim has no learned dynamics, and
# parallel cases must not compete for the 4 GB GPU).
#
#   scripts/bench/build_bench.sh                      # nav2_stack + both vendored forks
#   scripts/bench/build_bench.sh nav2_stack           # just these packages
set -e
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
PKGS=${@:-nav2_stack nav2_smac_planner_custom nav2_mppi_controller}
[ -f "$HOME/cuda_end/bin/activate" ] && source "$HOME/cuda_end/bin/activate"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select $PKGS --build-base build_bench --install-base install_bench \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
Y=install_bench/nav2_stack/share/nav2_stack/config/nav2_param2.yaml
if [ -f "$Y" ]; then
  sed -i 's/^\(\s*dynamics_mode:\s*\)"[a-z_]*"/\1"kinematics"/; s/^\(\s*nn_use_cuda:\s*\)true/\1false/' "$Y"
  echo "bench yaml: $(grep -E '^\s*dynamics_mode:' $Y | head -1 | xargs) / $(grep -E '^\s*nn_use_cuda:' $Y | head -1 | xargs)"
fi
