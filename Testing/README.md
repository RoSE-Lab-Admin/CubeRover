# Hardware Testing

The `Testing` folder contains tools for testing the hardware and gathering data from the hardware.

### Testing Components Documentation
* **[PySerial DAQ](PySerial/DAQ/README.md)**: Info on Arduino setup and the symlink in the DAQ folder.
* **[Profile Configurations](test_profiles/README.md)**: Info on adding test profile configuration files using the "Step" and "Pass Critera" classes.
  - **[Profile Step Info](test_engine/steps/README.md)**: Info on adding additional step options.
  - **[Profile Pass Criteria Info](test_engine/pass_criteria/README.md)**: Info on adding additional pass criteria options.


## Quickstart

**⚠️ Note:** All commands listed below are for executing the script from the **project root directory** (not from inside the `Testing` folder) unless explicitly stated otherwise.

1. Connect the lab computer to the internet and download the latest code.
    ```bash
    # Pull all latest code
    git pull -all

    # Switch to the target branch (`dev` in this example)
    git switch origin/dev
    ```
2. Connect the lab computer to the same network as the Raspberry Pi (e.g., `roseyhotspot`).
3. On the lab computer, open a terminal (VS Code: `Ctrl` + `Shift` + `~`) and run the automated hardware startup script. Enter the Pi's password when prompted.
    ```bash
    bash Testing/hardware_start.sh
    ```
    - The current code in your branch will be synced to the Pi, built, and the hardware launched.
    - The hardware successfully started when the final message is something like the following: `[INFO] [spawner-x]: process has finished cleanly [pid: xxxx]`.
4. On the lab computer, open a terminal and run the `gui_start.py` script.
    ```bash
    python3 Testing/gui_start.py
    ```
    - The gui should open in the lab computer's browser.
5. Select a test from the gui's drowndown test menu and click the start button to run the test.


## DAQ Gui

### Run the Test GUI
    
- The following will automatically attempt to detect the Teensy port and will provide a warning in the terminal if it is unable to find a Teensy:
    ```bash
    python3 Testing/gui_start.py
    ```

- Manually specify the port:
    ```bash
    python3 Testing/gui_start.py --port /dev/ttyACM1
    ```

- Run the application using mock data:
    ```bash
    python3 Testing/gui_start.py --mock
    ```

### Test Profiles

- The test profile options provided in the Test GUI are populated based on `.yml` files in the [`Testing/test_profiles`](./test_profiles/) folder. 
- Information on creating new test profiles can be found in the [`Testing/test_profiles/README.md`](./test_profiles/README.md).


## Hardware Scripts

To use these scripts, connect the lab computer to the same network as the Raspberry Pi. Run any of the scripts using `bash <path>/<script-name>.sh`.

### Scripts:

Located in the `Testing/scripts` folder:

- `config.sh`: The configuration variables for the other scripts in this list.
    - `ssh_config`: Not an executable script! Specifies the OpenSSH configuration.
- `sync_build_launch.sh`: All-in-one sync, build, and launch.
    - `sync.sh`: Sync the current code to the Pi.
    - `build.sh`: Build the synced code on the Pi.
    - `launch.sh`: Launch the synced code as a ROS node on the Pi.
- General helper:
    - `stop.sh`: Helper script to stop the ROS nodes (shouldn't be needed in most cases).
    - `clean.sh`: Helper script to assist with fixing any `colcon` build issues.

### Legacy Scripts:

Located in the `Testing/scripts/legacy` folder:

- `end_to_end.sh`: Starts the motors. The GUI application now can do this.
- `start_rosey.sh`: Similar to the `launch.sh`. Starts the hardware with the code in the referenced directory without building first.


## Additional How-To Instructions

### Hardware Startup
1. Connect the lab computer to the same network as the Raspberry Pi.
2. On the lab computer, open a terminal and run the automated hardware startup script:

    i. From project root, edit the `Testing/scripts/legacy/start_rosey.sh` to target the desired folder on the Pi.
    
    ii. Start the hardware:
    ```bash
    bash Testing/scripts/legacy/start_rosey.sh
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

1. From project root, open a terminal and start the `end-to-end.sh` script:
    ```bash
    bash Testing/scripts/legacy/end_to_end.sh
    ```
3. Kill the script process using `Ctrl` + `C`.

### CLI Monitoring Script

On the lab computer, open a terminal and start the monitoring script.

1. From the project root, run the script:
    ```bash
    python3 Testing/PySerial/tester.py
    ```
2. Enter the Teensy's port (likely `/dev/ttyACM0` or similar).
