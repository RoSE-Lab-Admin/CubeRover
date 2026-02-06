
BAG_DIR="ros_bags"
BAG_NAME="$(date '+%Y-%m-%d_%H-%M-%S')"
BAG_PATH="$BAG_DIR/$BAG_NAME"

DURATION=15

cd /mnt/d/

mkdir -p "$BAG_PATH"

echo "Starting ROS2 Bag Recording"

ros2 bag record -a -o "$BAG_PATH" & BAG_PID=$!

sleep "$DURATION"

kill -SIGINT "$BAG_PID"

wait "$BAG_PID"

echo "Finished Recording Ros2 Bag"
