set -euo pipefail

BAG_DIR="${BAG_DIR:-ros_bags}"
DURATION="${DURATION:-15}"

BAG_NAME="$(date '+%Y-%m-%d_%H-%M-%S')"
BAG_PATH="$BAG_DIR/$BAG_NAME"

source /opt/ros/jazzy/setup.bash

mkdir -p "$BAG_PATH"

echo "BAG_PATH=$BAG_PATH"

ros2 bag record -a -o "$BAG_PATH" & BAG_PID=$!

sleep "$DURATION"

kill -SIGINT "$BAG_PID"

wait "$BAG_PID"

echo "Finished Recording Ros2 Bag"
