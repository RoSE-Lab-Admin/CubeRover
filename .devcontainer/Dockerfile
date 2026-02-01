FROM osrf/ros:jazzy-desktop

# Setup non-root user 'devuser'
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create devuser and delete default ubuntu user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        groupdel ubuntu || true; \
    fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add devuser to the dialout group to access serial ports
    && usermod -aG dialout $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install development tools
RUN apt-get update && apt-get install -y \
    git \    
    openssh-client \
    usbutils \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rviz2 \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep as root
RUN rosdep init || true

# Switch to devuser
USER $USERNAME
ENV SHELL /bin/bash

# Update rosdep as devuser
RUN rosdep update

# Bake Git settings into the image
RUN git config --global --add safe.directory "*" \
    && git config --global core.fileMode false

# Initialize bashrc
# 1. Source the ROS 2 underlay
# 2. Source the colcon tab completion
# 3. Dynamic Overlay Sourcing: Finds any workspace setup file under /workspaces
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "for f in /workspaces/*/install/setup.bash; do if [ -f \"\$f\" ]; then source \"\$f\"; fi; done" >> ~/.bashrc