# Instructions to Run on Raspberry Pi
1. Connect computer to the same network as the Raspberry Pi
2. On the lab computer, open a terminal:
    i. SSH into the Raspberry Pi
    ```bash
    ssh [username]@[IP_address]
    ```
    ii. From project root, source ROS2:
    ```bash
    source install/setup.bash
    ```
    iii. Launch ROS2 hardware:
    ```bash
    ros2 launch roseybot_control hardware_startup.launch.py
    ```
3. On the lab computer, navigate to the testing folder:
    i. From project root, navigate to `test` folder:
    ```bash
    cd src/roseybot_control/test
    ```
    ii. Run test:
    ```bash
    # Bash
    sh end_to_end.sh

    # Python - May have bug which prevents from ending properly
    python3 test_end_to_end.py
    ```