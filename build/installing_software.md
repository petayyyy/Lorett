#!/bin/bash
path="Lorett"
path_installation=`pwd`

check_python_version () {
    echo "Check Python version"
    echo "You have: "
    pyv=$(python -V 2>&1 | grep -Po '(?<=Python )(.+)')
    if [[ -z "$pyv" ]]
    then
        pyv="$(python3 -V)" 
    fi
    echo $pyv
    if [[ *"2."* != "$pyv" ]] ;then
    pyv=''
    else
    pyv='3'
    fi
}
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
}

check_python_version
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
