#!/bin/bash
path="Lorett"
start_buid_internet () {
    echo "Installing all dependence and libs"
    sudo apt-get update
    sudo apt-get install python3-soapysdr -y
    sudo apt-get install python3-numpy -y
    sudo apt-get install airspy -y
    sudo apt-get install soapysdr-module-airspy -y
    sudo apt install soapysdr-tools -y
    echo "Check installing SoapySDR"
    SoapySDRUtil --info

    echo "Install SoapySDR"
    cd ~
    cd catkin_ws/src/
    if [ ! -f  "/"$path ]; then
        mkdir $path
    fi
    cd $path
    git clone https://github.com/pothosware/SoapySDR.git
    cd SoapySDR
    git pull origin master
    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install
    sudo ldconfig

    cd ../..
    cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr -DLIB_INSTALL_DIR:PATH=lib64 -DLIB_SUFFIX=64 -DSOAPY_SDR_ROOT=/usr ..
    sudo add-apt-repository -y ppa:myriadrf/drivers
    sudo apt-get update
    sudo apt-get install airspy libairspy-dev -y
    
    git clone https://github.com/pothosware/SoapyAirspy.git
    cd SoapyAirspy
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

    echo "Check installing SoapyAirspy"
    SoapySDRUtil --probe="driver=airspy"

    echo "Install libs for python3 soft"
    sudo apt-get install python3-scipy -y
    sudo apt-get install python3-matplotlib -y

    echo "Install libs for python3 soft"
    sudo apt-get install python-scipy -y
    sudo apt-get install python-matplotlib -y
}
start_buid_without_internet () {
    echo "You in ass"
    echo "Connect lan or modem-phone"
}
echo -n "Checking Internet Connection..."
ERR=`ping 8.8.8.8 -c 2 2>&1 > /dev/null` && start_buid_internet || { start_buid_without_internet; }
