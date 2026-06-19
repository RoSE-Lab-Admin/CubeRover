sudo apt-get update
sudo apt-get install curl lsb-release gnupg

source /opt/ros/jazzy/setup.bash
sudo apt install \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup 

sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update

cd ~/Cuberover

rosdep install --from-paths src -y --ignore-src