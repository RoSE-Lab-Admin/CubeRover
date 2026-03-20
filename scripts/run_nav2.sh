#!/usr/bin/env bash

# end waypoint following cleanup function
# usages
# ./run_nav2.sh        #on rosey with optitrack mode
# ./run_nav2.sh --ekf  #on rosey with ekf mode
# ./run_nav2.sh --flat #on flat rosey (10.42.0.1, pw ssh)
ROVER_TYPE="rosey"
USE_OPTI=true

for arg in "$@"; do
    case $arg in
        --flat) ROVER_TYPE="flat" ;;
        --ekf) USE_OPTI=false     ;;
    esac
done

case $ROVER_TYPE in
    rosey)
        SSH_CMD="ssh rosey@192.168.2.50 -i ./ssh/id_rsa_ansible"
        ROBOT_ARG="robot:=rosey"
        ;;
    flat)
        SSH_CMD="ssh rosey@10.42.0.1"
        ROBOT_ARG="robot:=flat_rosey"
        ;;
esac

ROVER_WS="~/CubeRover/install/setup.bash"

end_nav() {
    echo "Shutting down hardware..."
    $SSH_CMD "pkill -2 -f hardware_startup.launch.py"
    sleep 1
    echo "Shutting down waypoint following..."
    #ssh rosey@192.168.2.50 -i ./ssh/id_rsa_ansible "pkill -2 -f hardware_startup.launch.py"
    pkill -2 -f waypoint.launch.py
    echo "ended nodes"
    exit 130
}

trap 'end_nav' INT TERM

#test remove
#ssh rosey@192.168.2.50 -i ./ssh/id_rsa_ansible "bash -lc \"
#source ~/CubeRover/install/setup.bash &&, ros2 launch roseybot_control hardware_startup.launch.py

gnome-terminal -- bash -c "
    $SSH_CMD 'bash -lc \"
        source $ROVER_WS &&
        ros2 launch roseybot_control hardware_startup.launch.py $ROBOT_ARG
    \"'
    exec bash
"

sleep 1
source /opt/ros/jazzy/setup.bash
source ~/CubeRover-testing/install/setup.bash #check this is right name

until ros2 topic list | grep -q "/cmd_vel"; do
    sleep 1
    echo "waiting for roseybot_controller to boot"
done

echo "starting waypoint follower..."

ros2 launch nav2_stack waypoint.launch.py use_opti:=$USE_OPTI







