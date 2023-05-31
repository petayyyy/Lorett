#!/bin/bash
path="Lorett"
path_installation=`pwd`
pyv='3'

move_Lorett () {
    echo "Move to installing directory"
    cd ~
    cd catkin_ws/src/
    if [ ! -f  "/"$path ]; then
        mkdir $path
    fi
    cd $path
}
start_buid_internet () {
    echo "Installing all dependence"
    sudo apt-get update
    sudo apt-get install -y --fix-missing airspy
    sudo apt-get install -y --fix-missing soapysdr-module-airspy
    sudo apt-get install -y --fix-missing soapysdr-tools
    sudo apt-get install -y --fix-missing libairspy-dev
    
    echo "Install libs for python$pyv soft"
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
}
start_buid_without_internet () {
    echo "Installing all dependence and libs without internet"
    sudo apt-get install -y --fix-missing ./Dependence/airspy
    sudo apt-get install -y --fix-missing ./Dependence/soapysdr-module-airspy
    sudo apt-get install -y --fix-missing ./Dependence/soapysdr-tools
    sudo apt-get install -y --fix-missing ./Dependence/libairspy-dev 
    
    echo "Install libs for python$pyv soft"
    sudo apt-get install -y --fix-missing ./Dependence/python$pyv-scipy
    sudo apt-get install -y --fix-missing ./Dependence/python$pyv-matplotlib
    sudo apt-get install -y --fix-missing ./Dependence/python$pyv-soapysdr
    sudo apt-get install -y --fix-missing ./Dependence/python$pyv-numpy
    sudo apt-get install -y --fix-missing ./Dependence/python$pyv-docopt

    echo "Install all packeges and libs for SatDump"
    sudo apt-get install -y --fix-missing ./Dependence/git
    sudo apt-get install -y --fix-missing ./Dependence/build-essential
    sudo apt-get install -y --fix-missing ./Dependence/cmake
    sudo apt-get install -y --fix-missing ./Dependence/g++
    sudo apt-get install -y --fix-missing ./Dependence/pkgconf
    sudo apt-get install -y --fix-missing ./Dependence/libfftw3-dev
    sudo apt-get install -y --fix-missing ./Dependence/libvolk2-dev
    sudo apt-get install -y --fix-missing ./Dependence/libjpeg-dev
    sudo apt-get install -y --fix-missing ./Dependence/libpng-dev
    sudo apt-get install -y --fix-missing ./Dependence/librtlsdr-dev
    sudo apt-get install -y --fix-missing ./Dependence/libhackrf-dev
    sudo apt-get install -y --fix-missing ./Dependence/libairspy-dev
    sudo apt-get install -y --fix-missing ./Dependence/libairspyhf-dev                          
    sudo apt-get install -y --fix-missing ./Dependence/libglew-dev
    sudo apt-get install -y --fix-missing ./Dependence/libglfw3-dev   
    sudo apt-get install -y --fix-missing ./Dependence/libzstd-dev   
    sudo apt-get install -y --fix-missing ./Dependence/xorg-dev
    sudo apt-get install -y --fix-missing ./Dependence/libglu1-mesa-dev
    sudo apt-get install -y --fix-missing ./Dependence/freeglut3-dev
    sudo apt-get install -y --fix-missing ./Dependence/mesa-common-dev       

    sudo pip$pyv install ./Dependence/Mako$pyv/Mako* --no-index
}

echo -n "Checking Internet Connection..."
ERR=`ping 8.8.8.8 -c 2 2>&1 > /dev/null` && start_buid_internet || { start_buid_without_internet; }

echo "Move to installing directory"
move_Lorett

echo "Intall SoapySDR"
git clone https://github.com/pothosware/SoapySDR.git 2>&1 || cd $path_installation

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

git clone https://github.com/pothosware/SoapyAirspy.git 2>&1 || cd $path_installation
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

cd ../..

echo "Install nng"
git clone https://github.com/nanomsg/nng.git 2>&1 || cd $path_installation
cd nng
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr .. # Linux
make -j4
sudo make install
cd ../..
#rm -rf nng

echo "Install SatDump"
git clone https://github.com/altillimity/satdump.git || cd $path_installation
cd satdump
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_LIVE=ON .. # Linux
make -j4
ln -s ../pipelines . 
ln -s ../resources . 
chmod +x ./satdump

echo "Make Satdum global name"
cd ~
text="export PATH=$PATH:/home/pi/catkin_ws/src/Lorett/satdump/build"
name=`tail -1 '.bashrc'`
if [[ $name == $text ]] ; then
    echo "OK"
else 
    echo $text >> '.bashrc'
fi
source ~/.bashrc
