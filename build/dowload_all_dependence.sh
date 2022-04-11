#!/bin/bash

echo "Create path for all dependence and libs"
mkdir Dependence && cd Dependence
path=`pwd`
sudo chown -R _apt:root /var/lib/apt/lists

echo "Downloading all dependence and libs for sdr"
sudo apt download -y --fix-missing airspy
sudo apt download -y --fix-missing soapysdr-module-airspy
sudo apt download -y --fix-missing soapysdr-tools
sudo apt download -y --fix-missing libairspy-dev 

echo "Download libs for python 2 and 3 for sdr"
sudo apt download -y --fix-missing python-scipy
sudo apt download -y --fix-missing python-matplotlib
sudo apt download -y --fix-missing python-soapysdr
sudo apt download -y --fix-missing python-numpy

sudo apt download -y --fix-missing python3-scipy
sudo apt download -y --fix-missing python3-matplotlib
sudo apt download -y --fix-missing python3-soapysdr
sudo apt download -y --fix-missing python3-numpy

echo "Download all packeges and libs for SatDump"
sudo apt download -y --fix-missing git
sudo apt download -y --fix-missing build-essential
sudo apt download -y --fix-missing cmake
sudo apt download -y --fix-missing g++
sudo apt download -y --fix-missing pkgconf
sudo apt download -y --fix-missing libfftw3-dev
sudo apt download -y --fix-missing libvolk2-dev
sudo apt download -y --fix-missing libjpeg-dev
sudo apt download -y --fix-missing libpng-dev
sudo apt download -y --fix-missing librtlsdr-dev
sudo apt download -y --fix-missing libhackrf-dev
sudo apt download -y --fix-missing libairspy-dev
sudo apt download -y --fix-missing libairspyhf-dev                          
sudo apt download -y --fix-missing libglew-dev
sudo apt download -y --fix-missing libglfw3-dev   
sudo apt download -y --fix-missing libzstd-dev   
sudo apt download -y --fix-missing xorg-dev
sudo apt download -y --fix-missing libglu1-mesa-dev
sudo apt download -y --fix-missing freeglut3-dev
sudo apt download -y --fix-missing mesa-common-dev       

echo "Download Mako for SatDump"
sudo pip download -d $path/Mako Mako
sudo pip3 download -d $path/Mako3 Mako

echo "Download Pyorbital for generate list of satellite"
sudo pip download -d $path/pyorbital pyorbital
sudo pip3 download -d $path/pyorbital3 pyorbital

echo "Download all git repositories"
git clone https://github.com/pothosware/SoapySDR.git
git clone https://github.com/pothosware/SoapyAirspy.git
git clone https://github.com/nanomsg/nng.git
git clone https://github.com/altillimity/satdump.git
