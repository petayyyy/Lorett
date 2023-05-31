#!/bin/bash
path="~/Lorett/build"

echo "Build Rock dependence"
sudo apt install -y wget
wget -O - apt.radxa.com/focal-stable/public.key | sudo apt-key add -
sudo apt update
sudo apt-get install -y rockchip-overlay
sudo apt-get install -y broadcom-wifibt-firmware
sudo apt install libsndfile1-dev -y
sudo apt-get install libsoapysdr-dev -y
sudo apt install libspdlog-dev -y
sudo apt-get -y install python3-pip -y

echo "Build Realsense"
cd $path
bash ./installing_realsense.sh

echo "Build ROS noetic"
cd $path
bash ./installing_ros.sh

echo "Build SDR dependence"
cd $path
bash ./installing_sdr.sh

echo "Build Satdump" 
cd $path
bash ./installing_satdump.sh
