#!/bin/bash
path="dependence"
path_cam="realsense_dependence"

echo "Install packeges for Realsense"
#sudo apt -y install  ros-noetic-realsense2-camera-dbgsym
sudo apt -y install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

echo "Create dependence directory"
cd ~
if [ ! -f  "/"$path ]; then
    mkdir $path
fi
cd $path
if [ ! -f  "/"$path_cam ]; then
    mkdir $path_cam
fi
cd $path_cam

echo "Install and build realsense driver and toolkit"
git clone -b v2.50.0 https://github.com/IntelRealSense/librealsense.git
cd librealsense
bash ./scripts/setup_udev_rules.sh
#bash ./scripts/patch-realsense-ubuntu-lts.sh
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_TM2=true
sudo make uninstall && make clean && make && sudo make install

sudo udevadm control --reload-rules && udevadm trigger
sudo modprobe uvcvideo

# echo "Install and build Intel Realsense packages"
# sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
# sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
# sudo apt update
# sudo apt-get install -y librealsense2*
# sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
# sudo apt-get install -y ros-noetic-librealsense*
