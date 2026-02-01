# CubeRover
ROS Source Build files for the RoSE Lab CubeRover


[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)
[![Dev Containers](https://img.shields.io/badge/Dev%20Containers-2A6DB0?logo=visualstudiocode&logoColor=white)](https://code.visualstudio.com/docs/devcontainers/containers)
[![License](https://img.shields.io/github/license/RoSE-Lab-Admin/CubeRover)](LICENSE)


## 🚀 Quick Start

This repository is configured as a **Dev Container**, providing a fully integrated environment for ROS 2 Jazzy.

### 1. Prerequisites
- **Docker Desktop**: [Download and Install](https://www.docker.com)
  - *Windows Users:* Ensure **"Use the WSL 2 based engine"** is enabled in Settings.
- **Visual Studio Code**: [Download and Install](https://code.visualstudio.com/)
- **Dev Containers Extension**: [Install from Marketplace](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### 2. Choose Your OS & Launch the Environment

> [!WARNING]
> Ensure the Docker daemon is active before proceeding!

> [!NOTE]
> The first container startup may take several minutes as dependencies are downloaded.

<details>
<summary><strong>🪟 Windows Users (WSL 2)</strong></summary>

> [!IMPORTANT]
> **Performance Warning:** Clone this repo into your **WSL filesystem** (e.g., `\\wsl$\Ubuntu\home\user`).
> Cloning onto the Windows (C:) drive will kill build performance and cause permission errors.

1. Open a **WSL Terminal** (Ubuntu).
2. Clone and open:
    ```bash
    git clone https://github.com/RoSE-Lab-Admin/CubeRover.git
    cd CubeRover
    code .
    ```
3. When VS Code opens, check the bottom-left green corner.
    * If it says **WSL: Ubuntu**, you are good.
    * Press `Ctrl+Shift+P` and select **Dev Containers: Reopen in Container**.
</details>

<details>
<summary><strong>🍎 macOS Users (M1/M2/Intel)</strong></summary>

> [!NOTE]
> **Apple Silicon Note:** Dev Containers automatically handle architecture translation on M1/M2 Macs.  
> You do *not* need to manually set `--platform linux/amd64`.

**(Optional) One-Time Setup (X11 Forwarding):**
X11 is required only if the container launches GUI applications (e.g., RViz, Gazebo).

1. Install XQuartz:
    ```bash
    brew install --cask xquartz
    ```
2. Open XQuartz > **Settings > Security** > Check **"Allow connections from network clients"**.
3. **Restart XQuartz** (Quit and reopen) for this to take effect.

**Running the Repo:**
1. Open a **Mac terminal**.
2. Clone and open:
    ```bash
    git clone https://github.com/RoSE-Lab-Admin/CubeRover.git
    cd CubeRover
    code .
    ```
3. (Optional) Run the command (allows the container to show GUI windows):
    ```bash
    xhost +localhost
    ```
4. Press `Cmd+Shift+P` and select **Dev Containers: Reopen in Container**.
</details>

<details>
<summary><strong>🐧 Linux Users</strong></summary>

1. Open a terminal.
2. Ensure your user is in the `docker` group (avoids using `sudo`):
    ```bash
    sudo usermod -aG docker $USER
    # Log out and back in for this to take effect!
    ```

> [!NOTE]
> If you see an error like “Cannot connect to the Docker daemon” when running `docker ps`,
> your system may require enabling the Docker service:
>
> ```bash
> sudo systemctl enable --now docker
> ```

3. Clone and open:
    ```bash
    git clone https://github.com/RoSE-Lab-Admin/CubeRover.git
    cd CubeRover
    code .
    ```
4. Press `Ctrl+Shift+P` and select **Dev Containers: Reopen in Container**.
</details>

### 3. First-Time Setup
Once inside the container, you must build the workspace to generate the setup files.

1.  **Build the workspace:**
    ```bash
    colcon build --symlink-install
    ```
2.  **Source the workspace (First time only):**
    ```bash
    source install/setup.bash
    ```

> [!NOTE]
> Future terminals inside the container will **automatically** source the workspace if a build exists.


## 📡 Network & Robot Connection
To communicate with the physical CubeRover, your Dev Container and the Raspberry Pi must share the same network settings.

1.  **Shared Network:** Ensure your computer and the Raspberry Pi are on the same Wi-Fi or Ethernet network.
2.  **Network Mode:** This container is set to `ROS_LOCALHOST_ONLY=0`, allowing it to communicate with external robots.
3.  **ROS Domain ID:** This project uses `ROS_DOMAIN_ID=42`.
    - **Check the Robot:** SSH into the Pi and run `echo $ROS_DOMAIN_ID`.
    - **Set the Robot:** If it is not 42, run: `echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc && source ~/.bashrc`
4.  **Verify Connection:**
    - On the Pi: `ros2 topic pub /ping std_msgs/msg/String "data: hello" -1`
    - In Dev Container: `ros2 topic echo /ping`
    - If you see "hello", the connection is working.


## 🖥️ Graphical Interface (GUI)
This container uses GUI forwarding (WSLg on Windows / X11 via XQuartz on macOS). Tools like RViz2 and Gazebo appear as native windows on your desktop.

1. Launch a tool from the container terminal:
    ```bash
    ros2 run rviz2 rviz2
    ```
2. **Notes:**
    - Windows users with WSLg will see windows immediately. WSLg provides GPU acceleration automatically on supported systems.
    - Mac users must have XQuartz running.


## 🧪 Running Tests
- **All:** `colcon test`
    - **Live console output:** `colcon test --packages-select roseybot_control --event-handlers console_direct+`
- **Specific package:** `colcon test --packages-select <package_name>`
- **Hardware only:** `colcon test --pytest-args -m hardware`
- **Software only:** `colcon test --pytest-args -m "not hardware"`


## 🔄 Developer Workflow
- **Smart Terminal:**
    - **Auto-Sourcing:** New terminals automatically find and source your workspace setup file.
    - **Tab Completion:** `colcon` commands support tab completion (e.g., type `colcon b` + `Tab` → `colcon build`).
- **Adding Dependencies:** If you modify `package.xml`, run:
    ```bash
    sudo apt update && rosdep install --from-paths src -y --ignore-src
    ```
- **Symlinks:** We use `colcon build --symlink-install`. You only need to rebuild if you add new files or change C++ code; existing Python scripts update instantly.


## 🔌 Hardware Setup (Microcontroller USB)

#### 🪟 Windows (WSL 2)
- Native USB passthrough is not supported by Docker Desktop. You must bridge devices from Windows:
    1.  Install [usbipd-win v4.0+](https://github.com/dorssel/usbipd-win/releases).
    2. Open **PowerShell (Admin)** and list devices: `usbipd list`.
    3. Bind the device (once): `usbipd bind --busid <BUSID>`.
    4. Attach and auto-reconnect: `usbipd attach --wsl --busid <BUSID> --auto-attach`.
    5. The device will appear at `/dev/ttyUSB*` or `/dev/ttyACM*` inside the container.

> [!NOTE]
> Once attached, the device stays attached until unplugged or the WSL session ends.

#### 🍎 macOS
- Ensure your Mac host has the necessary drivers (CH340/CP210x).
- **Docker Desktop 4.35+** includes experimental USB passthrough, but many serial microcontrollers are not yet supported. If unsupported, use `socat` to bridge the serial port.


## 🛠️ Troubleshooting
- **Slow Builds (Windows):** Verify you cloned into `/home/<user>/...` and NOT `/mnt/c/...`
- **GUI not appearing (Mac):** Ensure you ran `xhost +localhost` on the Mac host.
- **Permissions:** The devuser is pre-added to the `dialout` group for serial access. If denied, run `ls -l /dev/ttyUSB*` to check ownership.
- **Linux Permissions:** Ensure your user is in the `docker` group: `sudo usermod -aG docker $USER`.
    - After adding yourself to the `docker` group, restart your shell session for the change to take effect.
    - This applies to Linux hosts and WSL users running Docker inside WSL.
- **USB Visibility:** Run `lsusb` in the container terminal to verify host-to-container connection.
    - If `lsusb` is missing, install it with `sudo apt install usbutils`.
- **IntelliSense:** If parsing is slow, refer to the pre-configured `C_Cpp` settings in `.devcontainer/devcontainer.json`. 


## 🧰 Environment Details
### 📦 Pre-Installed System Tools
These tools are baked into the Docker image, so you don't need to install them:
* **ROS 2 Jazzy Desktop:** Includes core ROS tools, `rviz2`, and standard libraries.
* **Controllers:** `diff_drive_controller`, `ros2_control`, `ros2_controllers`.
* **Build Tools:** `colcon` (with tab completion enabled), `rosdep`, `git`, `pip`.
* **Utilities:** `usbutils` (for checking USB connections), `openssh-client`.

### 🧩 VS Code Extensions
These extensions install automatically when the container launches:
* **Python:** Full IntelliSense and debugging support.
* **C/C++:** IntelliSense configured for ROS 2 includes.
* **XML (RedHat):** Formatting and syntax checking for `package.xml` and launch files.
* **Ranch-Hand Robotics:** Specialized tools for this project.