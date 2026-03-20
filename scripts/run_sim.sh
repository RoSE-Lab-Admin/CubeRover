#!/usr/bin/env bash

# usage
# ./run_sim.sh       #optitrack mode
# ./run_sim.sh --ekf #ekf mode

USE_OPTI=true

for arg in "$@"; do
    [[ $arg == "--ekf" ]] && USE_OPTI=false
done

end_nav() {
    echo "Stopping sim..."
    pkill -2 -f rosey_gz.launch.py
    sleep 1
    echo "Stopping nav2..."
    pkill -2 -f waypoint.launch.py
    echo "Ended nodes."
    exit 130
}

trap 'end_nav' INT TERM

source /opt/ros/jazzy/setup.bash
source ~/CubeRover/install/setup.bash

gnome-terminal -- bash -c '
    source /opt/ros/jazzy/setup.bash
    source ~/CubeRover/install/setup.bash
    ros2 launch rosey_sim rosey_gz.launch.py
    exec bash
'

until ros2 topic list | grep -q "/cmd_vel"; do
    sleep 1
    echo "waiting for sim to boot..."
done

echo "Starting Nav2..."
ros2 launch nav2_stack waypoint.launch.py use_opti:=$USE_OPTI