#!/bin/bash
path="dependence"
path_cam="realsense_dependence"

echo "Install packeges for Realsense"
sudo apt -y install  ros-noetic-realsense2-camera-dbgsym
sudo apt -y install librealsense2*
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

