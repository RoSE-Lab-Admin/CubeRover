#!/usr/bin/env bash

# end waypoint following cleanup function
# usages
# ./run_nav2.sh        #on rosey with optitrack mode
# ./run_nav2.sh --ekf  #on rosey with ekf mode
# ./run_nav2.sh --flat #on flat rosey (10.42.0.1, pw ssh),default to EKF

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
        LOCAL_WS=~/CubeRover-testing/install/setup.bash #slade (check correct name - or _)
        ;;
    flat)
        SSH_CMD="ssh rosey@10.42.0.1"
        ROBOT_ARG="robot:=flat_rosey"
        USE_OPTI=false
        LOCAL_WS=~/CubeRover/install/setup.bash #flat rosey test desktop
        ;;
esac

ROVER_WS="~/CubeRover/install/setup.bash" #pi (should be same on both)


end_nav() {
    echo ""

    echo "Stopping hardware..."
    if [ ! -z "$HW_PID" ]; then
        $SSH_CMD "pkill -2 -f hardware_startup.launch.py" 2>/dev/null
        kill -9 $HW_PID 2>/dev/null  # disconnect SSH immediately to stop terminal output flood
        sleep 3                       # give remote process time to shut down cleanly
        $SSH_CMD "pkill -9 -f hardware_startup.launch.py" 2>/dev/null  # force kill if still running
    fi
    sleep 2

    echo "Stopping nav2..."
    if [ ! -z "$NAV_PID" ]; then
        kill -2 $NAV_PID
        sleep 3
        kill -9 $NAV_PID 2>/dev/null
    fi
    sleep 2

    echo "ended nodes."
    exit 130
    # echo "Shutting down hardware..."
    # $SSH_CMD "pkill -2 -f hardware_startup.launch.py"
    # sleep 1
    # echo "Shutting down waypoint following..."
    # #ssh rosey@192.168.2.50 -i ./ssh/id_rsa_ansible "pkill -2 -f hardware_startup.launch.py"
    # pkill -2 -f waypoint.launch.py
    # echo "ended nodes"
    # exit 130
}

trap end_nav INT TERM

echo "Sourcing ROS..."
source /opt/ros/jazzy/setup.bash
source "$LOCAL_WS"

echo "Starting rosey hardware..."
$SSH_CMD "bash -lc \"
    source $ROVER_WS &&
    ros2 launch roseybot_control hardware_startup.launch.py $ROBOT_ARG
\"" & HW_PID=$!
echo "Hardware PID: $HW_PID"

echo "Waiting for roseybot_controller to boot..."
until ros2 topic list | grep -q "/cmd_vel"; do
    sleep 1
    echo "waiting..."
done

echo "Hardware ready. Waiting for nodes to stabilize..." #safety wait for service setups
sleep 3

echo "Starting nav2..."
ros2 launch nav2_stack waypoint.launch.py use_opti:=$USE_OPTI & NAV_PID=$!
echo "nav2 PID: $NAV_PID"
wait $NAV_PID
#test remove
#ssh rosey@192.168.2.50 -i ./ssh/id_rsa_ansible "bash -lc \"
#source ~/CubeRover/install/setup.bash &&, ros2 launch roseybot_control hardware_startup.launch.py

# gnome-terminal -- bash -c "
#     $SSH_CMD 'bash -lc \"
#         source $ROVER_WS &&
#         ros2 launch roseybot_control hardware_startup.launch.py $ROBOT_ARG
#     \"'
#     exec bash
# "

# sleep 1
# source /opt/ros/jazzy/setup.bash
# source "$LOCAL_WS"

# until ros2 topic list | grep -q "/cmd_vel"; do
#     sleep 1
#     echo "waiting for roseybot_controller to boot"
# done

# echo "starting waypoint follower..."

# ros2 launch nav2_stack waypoint.launch.py use_opti:=$USE_OPTI







