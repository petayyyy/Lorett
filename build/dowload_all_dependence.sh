#!/bin/bash

cd ~/
echo "Create path for all dependence and libs"
mkdir Dependence && cd Dependence
path=`pwd`

echo "Preparation before installing packages and libs"
sudo apt update

echo "Downloading all packages and libs for sdr"
sudo apt download -y airspy soapysdr-module-airspy soapysdr-tools libairspy-dev 

echo "Download libs for python 3 for sdr"
sudo apt download -y python3-scipy python3-matplotlib python3-soapysdr python3-numpy python3-mako python3-pyorbital python3-docopt

echo "Download all packeges and libs for SatDump"
sudo apt download -y git build-essential cmake g++ pkgconf libfftw3-dev libvolk2-dev libjpeg-dev libpng-dev librtlsdr-dev libhackrf-dev libairspy-dev libairspyhf-dev libglew-dev libglfw3-dev libzstd-dev xorg-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev  

echo "Download all git repositories"
sudo apt-get install -y git
git clone https://github.com/pothosware/SoapySDR.git
git clone https://github.com/pothosware/SoapyAirspy.git
git clone https://github.com/nanomsg/nng.git
git clone https://github.com/altillimity/satdump.git

echo "Create tar for moving path with all dependence"
cd ..
tar cvfz Dependence.tgz Dependence
