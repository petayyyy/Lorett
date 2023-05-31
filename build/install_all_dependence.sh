#!/bin/bash
pyv='3'

echo "Build Rock dependence"
sudo apt install -y wget
wget -O - apt.radxa.com/focal-stable/public.key | sudo apt-key add -
sudo apt update
sudo apt-get install -y rockchip-overlay
sudo apt-get install -y broadcom-wifibt-firmware

echo "Build ROS noetic"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
apt search ros-noetic
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
sudo mkdir catkin_ws
cd catkin_ws/
mkdir src
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "Build Lorett packages"
sudo apt install -y git
cd ~
git clone https://github.com/petayyyy/Lorett.git
cd Lorett
mv packages/* ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

echo "Install and build Intel Realsense packages"
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt update
sudo apt-get install -y librealsense2*
sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install -y ros-noetic-librealsense*

echo "Install Mavros"
sudo apt-get -y install ros-noetic-mavros*
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
sudo apt install -y ros-noetic-ddynamic-reconfigure*

echo "Final build"
cd ~/catkin_ws
catkin_make

echo "Installing all dependence for SDR"
sudo apt-get update
sudo apt-get install -y --fix-missing airspy
sudo apt-get install -y --fix-missing soapysdr-module-airspy
sudo apt-get install -y --fix-missing soapysdr-tools
sudo apt-get install -y --fix-missing libairspy-dev

echo "Install libs for python$pyv SDR soft"
sudo apt-get install -y --fix-missing python$pyv-scipy
sudo apt-get install -y --fix-missing python$pyv-matplotlib
sudo apt-get install -y --fix-missing python$pyv-soapysdr
sudo apt-get install -y --fix-missing python$pyv-numpy
sudo apt-get install -y --fix-missing python$pyv-docopt

echo "Install all packeges and libs for SatDump"
sudo apt install -y --fix-missing git build-essential cmake g++ pkgconf libfftw3-dev  libjpeg-dev libpng-dev 
sudo apt install -y --fix-missing libvolk2-dev
sudo apt install -y --fix-missing libvolk1-dev
sudo apt-get install -y --fix-missing librtlsdr-dev libhackrf-dev libairspy-dev libairspyhf-dev                          
sudo apt-get install -y --fix-missing libglew-dev libglfw3-dev   
sudo apt-get install -y --fix-missing libzstd-dev   
sudo apt-get install -y --fix-missing xorg-dev
sudo apt-get install -y --fix-missing libglu1-mesa-dev freeglut3-dev mesa-common-dev       
pip$pyv install Mako

echo "Intall SoapySDR"
cd ~/catkin_ws/src
git clone https://github.com/pothosware/SoapySDR.git 2>&1 

cd SoapySDR
git pull origin master 2>&1
mkdir build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
cd ../..
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr -DLIB_INSTALL_DIR:PATH=lib64 -DLIB_SUFFIX=64 -DSOAPY_SDR_ROOT=/usr ..
sudo add-apt-repository -y ppa:myriadrf/drivers

cd ~/catkin_ws/src
git clone https://github.com/pothosware/SoapyAirspy.git 2>&1 
cd SoapyAirspy
mkdir build
cd build
cmake ..
make
sudo make install

echo "Check installing SoapySDR"
SoapySDRUtil --info
echo "Check installing SoapyAirspy"
SoapySDRUtil --probe="driver=airspy"

echo "Install nng"
cd ~/catkin_ws/src
git clone https://github.com/nanomsg/nng.git 2>&1 
cd nng
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr .. # Linux
make -j4
sudo make install
#rm -rf nng

echo "Install SatDump"
cd ~/catkin_ws/src
git clone https://github.com/altillimity/satdump.git
cd satdump
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_LIVE=ON .. # Linux
make -j4
ln -s ../pipelines . 
ln -s ../resources . 
chmod +x ./satdump

echo "Make Satdum global name"
cd ~
text="export PATH=$PATH:~/catkin_ws/src/satdump/build"
name=`tail -1 '.bashrc'`
if [[ $name == $text ]] ; then
    echo "OK"
else 
    echo $text >> '.bashrc'
fi
source ~/.bashrc
