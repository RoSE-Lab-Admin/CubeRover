#!/usr/bin/env bash

# end waypoint following cleanup function
end_nav() {
    echo "Shutting down waypoint following..."
    ssh rosey@192.168.2.50 -i ./ssh/id_rsa_ansible "pkill -2 -f hardware_startup.launch.py"
    pkill -2 -f waypoint.launch.py
    echo "ended nodes"
    exit 130
}

trap 'end_nav' INT TERM

gnome-terminal -- bash -c '
ssh rosey@192.168.2.50 -i ./ssh/id_rsa_ansible "bash -lc \"
source ~/CubeRover/install/setup.bash &&
ros2 launch roseybot_control hardware_startup.launch.py
\"";
exec bash
'

sleep 1
source /opt/ros/jazzy/setup.bash
source ~/CubeRover-testing/install/setup.bash #check this is right name

until ros2 topic list | grep -q "/cmd_vel"; do
    sleep 1
    echo "waiting for roseybot_controller to boot"
done

echo "starting waypoint follower..."

ros2 launch nav2_stack waypoint.launch.py







