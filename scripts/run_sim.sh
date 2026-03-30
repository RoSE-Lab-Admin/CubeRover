#!/usr/bin/env bash

# usage
# ./run_sim.sh       #optitrack mode
# ./run_sim.sh --ekf #ekf mode

USE_OPTI=true

for arg in "$@"; do
    [[ $arg == "--ekf" ]] && USE_OPTI=false
done

# end_nav() {
#     echo "Stopping sim..."
#     pkill -2 -f rosey_gz.launch.py
#     sleep 1
#     echo "Stopping nav2..."
#     pkill -2 -f waypoint.launch.py
#     echo "Ended nodes."
#     exit 130
# }
end_nav(){
    echo ""
    
    echo "Stopping nav2..."
    if [ ! -z "$NAV_PID" ]; then
        kill -2 $NAV_PID
        sleep 3
        kill -9 $NAV_PID 2>/dev/null
    fi

    sleep 2

    echo "Stopping sim..."

    if [ ! -z "$SIM_PID" ]; then
        kill -2 $SIM_PID 2>/dev/null
        sleep 2
        kill -9 $SIM_PID 2>/dev/null
    fi

    echo "Ended nodes."
    exit 130
}

trap end_nav INT TERM

echo "Sourcing ROS..."
source /opt/ros/jazzy/setup.bash
source ~/CubeRover/install/setup.bash

# gnome-terminal -- bash -c '
#     source /opt/ros/jazzy/setup.bash
#     source ~/CubeRover/install/setup.bash
#     ros2 launch rosey_sim rosey_gz.launch.py
#     exec bash
# '
echo "Starting Gazebo..."
ros2 launch rosey_sim rosey_gz.launch.py & SIM_PID=$!
echo "Sim PID: $SIM_PID"

echo "Waiting for sim to boot..."
until ros2 node list | grep -q "gz"; do
    sleep 1
    echo "waiting..."
done
echo "Sim ready. Waiting for nodes to stabilize..."

echo "Waiting for TF..."

until ros2 topic list | grep -q "/tf"; do
    sleep 1
done

echo "Waiting for clock..."

until ros2 topic list | grep -q "/clock"; do
    sleep 1
done

sleep 3

echo "Starting Nav2..."
ros2 launch nav2_stack waypoint.launch.py use_opti:=$USE_OPTI & NAV_PID=$!

echo "Nav2 PID: $NAV_PID"

wait $NAV_PID