# Hardware Testing

## DAQ Gui

### Setup

- Install python virtual environment
    ```bash
    sudo apt install python3.12-venv
    ```

- Create the virtual environment at the project root.
    ```bash
    python3 -m venv venv
    ```

- Activate the virtual environment.
    ```bash
    source venv/bin/activate
    ```

- Install dependencies
    ```bash
    pip install -r Testing/requirements.txt
    ``` 

### Run the Test GUI
    ```bash
    python3 Testing/testing_gui.py
    ```


## Instructions for Updating Raspberry Pi Code

1. Login to the Raspberry Pi via a connected keyboard, mouse, and monitor.
2. Connect the Raspberry Pi to the internet.
3. Open a terminal and navigate to the codebase folder. 
    ```bash
    cd Cuberover
    ```
4. Switch to the `dev` branch.
    ```bash
    git switch dev
    ```
5. Pull the latest `dev` branch.
    ```bash
    git pull --all
    ```


## Instructions to Run End-to-End Test
1. Connect the lab computer to the same network as the Raspberry Pi.
2. On the lab computer, open a terminal and run the automated hardware startup script:
    i. From project root, navigate to `Testing` folder:
    ```bash
    cd Testing
    ```
    ii. Start the hardware:
    ```bash
    sh start_rosey.sh
    ```

    <details>
    <summary>Manual startup steps (for reference)</summary>

    i. SSH into the Raspberry Pi
    ```bash
    ssh [username]@[IP_address]
    ```
    ii. Navigate to the codebase folder.
    ```bash
    cd Cuberover
    ```
    iii. From project root, source ROS2:
    ```bash
    source install/setup.bash
    ```
    iv. Launch ROS2 hardware:
    ```bash
    ros2 launch roseybot_control hardware_startup.launch.py
    ```

    </details>

3. (Optional) On the lab computer, open a terminal and start the monitoring script.

    i. From the project root, run the script:
    ```bash
    python3 Testing/PySerial/tester.py
    ```
    ii. Enter the port (likely `ACM0` or another port indicating that it is the Teensy).

4. On the lab computer, open a terminal and start the end-to-end test:

    i. From project root, navigate to `Testing` folder:
    ```bash
    cd Testing
    ```
    ii. Run test:
    ```bash
    sh end_to_end.sh
    ```
    iii. Kill test process using `Ctrl` + `C`.