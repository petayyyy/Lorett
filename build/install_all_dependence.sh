#!/bin/bash
path_in_git="/Lorett/build"
path_to_build_script=$(pwd)
path="~/Lorett/build"
if [[ ! -f install_all_dependence.sh ]]
then
    echo "<installing_realsense.sh> exists on your filesystem."
fi

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
cd $path_to_build_script
bash ./installing_realsense.sh

echo "Build ROS noetic"
cd $path_to_build_script
bash ./installing_ros.sh

echo "Build SDR dependence"
cd $path_to_build_script
bash ./installing_sdr.sh

echo "Build Satdump" 
cd $path_to_build_script
bash ./installing_satdump.sh

echo "Connect flight controller"
sudo usermod -a -G dialout $USER
