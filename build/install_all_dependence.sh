#!/bin/bash
pyv='3'

echo "Build Rock dependence"
sudo apt install -y wget
wget -O - apt.radxa.com/focal-stable/public.key | sudo apt-key add -
sudo apt update
sudo apt-get install -y rockchip-overlay
sudo apt-get install -y broadcom-wifibt-firmware

echo "Build Realsense"
bash ./installing_realsense.sh

echo "Build ROS noetic"
bash ./installing_ros.sh

echo "Build SDR dependence"
bash ./installing_sdr.sh

echo "Build Satdump" 
bash ./installing_satdump.sh
