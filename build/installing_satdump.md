#!/bin/bash

sudo apt install git build-essential cmake g++ pkgconf libfftw3-dev libvolk2-dev libjpeg-dev libpng-dev 
sudo apt install librtlsdr-dev libhackrf-dev libairspy-dev libairspyhf-dev                             
sudo apt install libglew-dev libglfw3-dev   
sudo apt install libzstd-dev   

sudo apt install xorg-dev
pip3 install Mako
pip install Mako 
sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev
sudo apt-get install -y libglew-dev
sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev       

git clone https://github.com/nanomsg/nng.git
cd nng
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON ..                             # MacOS
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr .. # Linux
make -j4
sudo make install
cd ../..
rm -rf nng

git clone https://github.com/altillimity/satdump.git
cd satdump
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_LIVE=ON .. # Linux
make -j4
ln -s ../pipelines . 
ln -s ../resources . 

chmod +x ./satdump

cd ~
text="alias ./satdump='sh /catkin_ws/src/Lorett/satdump/satdump'"

name=`tail -1 '.bashrc'`
if [[ $name == $text ]] ; then
    echo "OK"
else 
    echo $text >> '.bashrc'
fi

source ~/.bashrc
