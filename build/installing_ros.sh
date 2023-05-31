#!/bin/bash
echo "Set up your keys"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo "Install ROS Desktop"
sudo apt update
sudo apt install ros-noetic-desktop-full

echo "Check available packages"
apt search ros-noetic

echo "Environment setup"
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

echo "Environment setup"
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Build packages"
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

echo "Initialize rosdep"
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

echo "Creating a workspace for catkin"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
echo "Build a catkin workspace"
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Check ROS path"
echo $ROS_PACKAGE_PATH

echo "Install Mavros"
sudo apt-get -y install ros-noetic-mavros*
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
sudo apt install -y ros-noetic-ddynamic-reconfigure*

echo "Install all ros packages"
sudo apt -y install ros-noetic-realsense2-camera-dbgsym

echo "Install C4S_v2 ros packages"
sudo apt install -y git
cd ~
if [ ! -f  "/Lorett" ]; then
    git clone https://github.com/petayyyy/Lorett.git
fi
cd Lorett
mv packages/* ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
