#!/bin/bash
path="~/Lorett/build"

echo "Build Rock dependence"
sudo apt install -y wget
sudo apt update

sudo apt install libsndfile1-dev -y
sudo apt install libspdlog-dev -y
sudo apt-get -y install python3-pip -y
sudo apt-get -y install bsdmainutils
sudo apt-get -y install hdparm
sudo apt install -y cmake make

pip3 install flask
pip3 install pyorbital
pip3 install prettytable
pip3 install bs4

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

echo "Connect flight controller"
sudo usermod -a -G dialout $USER
