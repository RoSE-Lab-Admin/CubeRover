USERNAME=rosey
HOST=10.42.0.1
SCRIPT="
cd CubeRover;
source install/setup.bash;
ros2 launch roseybot_control hardware_startup.launch.py
"

ssh -t -l ${USERNAME} ${HOST} "${SCRIPT}"