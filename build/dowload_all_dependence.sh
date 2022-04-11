#!/bin/bash

echo "Create path fpr all dependence and libs"
mkdir Dependence && cd Dependence
sudo chown -R _apt:root /var/lib/apt/lists

echo "Downloading all dependence and libs without internet"
sudo apt download -y --fix-missing airspy
sudo apt download -y --fix-missing soapysdr-module-airspy
sudo apt download -y --fix-missing soapysdr-tools
sudo apt download -y --fix-missing libairspy-dev 

echo "Download libs for python 2 and 3 soft"
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
pip3 download -y Mako
pip download -y Mako
