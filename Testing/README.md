# Hardware Testing

The `Testing` folder contains tools for testing the hardware and gathering data from the hardware.

### Testing Components Documentation
* **[PySerial DAQ](PySerial/DAQ/README.md)**: Info on Arduino setup and the symlink in the DAQ folder.
* **[Profile Configurations](test_profiles/README.md)**: Info on adding test profile configuration files using the "Step" and "Pass Critera" classes.
  - **[Profile Step Info](test_engine/steps/README.md)**: Info on adding additional step options.
  - **[Profile Pass Criteria Info](test_engine/pass_criteria/README.md)**: Info on adding additional pass criteria options.

## DAQ Gui

### Setup

- Install dependencies.
    ```bash
    pip install --no-cache-dir --break-system-packages -r Testing/requirements.txt
    ```
- Add the installation location to the path:
    ```bash
    export PATH="/home/devuser/.local/bin:$PATH"
    ```

### Run the Test GUI
    
- The following will automatically attempt to detect the Teensy port and will provide a warning in the terminal if it is unable to find a Teensy:
    ```bash
    python3 Testing/test_gui.py
    ```

- Manually specify the port:
    ```bash
    python3 test_gui.py --port /dev/ttyACM1
    ```

- Run the application using mock data:
    ```bash
    python3 test_gui.py --mock
    ```

### Test Profiles

- The test profile options provided in the Test GUI are populated based on `.yml` files in the [`Testing/test_profiles`](./test_profiles/) folder. 
- Information on creating new test profiles can be found in the [`Testing/test_profiles/README.md`](./test_profiles/README.md).

## Hardware Scripts

To use these scripts, connect the lab computer to the same network as the Raspberry Pi.


### Scripts Setup

- Run the following to make the `*.sh` files executable:
    ```bash
    chmod +x Testing/*.sh
    ```

- Install rsync (for file syncing):
    ```bash
    sudo apt-get update && sudo apt-get install -y rsync
    ```


### Current Scripts:

Run any of these scripts using `bash <script-name>.sh`.

- `config.sh`: The configuration variables for the other scripts in this list.
- `sync_and_launch.sh`: All-in-one sync, build, and launch.
- `sync.sh`: Sync the current code to the Pi.
- `launch.sh`: Build and launch the ROS hardware node on the Pi.
- `stop.sh`: Helper script to stop the ROS nodes.
- `clean.sh`: Helper script to assist with fixing any `colcon` build issues.

### Legacy Scripts:

- `end_to_end.sh`: Starts the motors. The GUI application now can do this.
- `start_rosey.sh`: Similar to the `launch.sh`. Starts the hardware with the current code in the referenced directory without building first.

## Additional How-To Instructions

### Hardware Startup
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

### Run the Motors

On the lab computer, open a terminal and start the end-to-end test:

1. From project root, navigate to `Testing` folder:
    ```bash
    cd Testing
    ```
2. Run test:
    ```bash
    sh end_to_end.sh
    ```
3. Kill test process using `Ctrl` + `C`.

### CLI Monitoring Script

On the lab computer, open a terminal and start the monitoring script.

1. From the project root, run the script:
    ```bash
    python3 Testing/PySerial/tester.py
    ```
2. Enter the port (likely `/dev/ttyACM1` or another port indicating that it is the Teensy).
