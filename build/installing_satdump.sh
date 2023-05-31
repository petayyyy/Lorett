#!/bin/bash
path="sdr_dependence"

echo "Install all packeges and libs for SatDump"
sudo apt install -y git build-essential cmake g++ pkgconf libfftw3-dev  libjpeg-dev libpng-dev 
sudo apt install -y libvolk2-dev
sudo apt install -y libvolk1-dev
sudo apt install -y libluajit
sudo apt install -y --fix-missing librtlsdr-dev libhackrf-dev libairspy-dev libairspyhf-dev libglew-dev libglfw3-dev libzstd-dev                           
sudo apt install -y xorg-dev
sudo apt install -y libglu1-mesa-dev freeglut3-dev mesa-common-dev       

pip3 install Mako

cd ~
if [ ! -f  "/"$path ]; then
    mkdir $path
fi
cd $path

echo "Install nng"
git clone https://github.com/nanomsg/nng.git
cd nng
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr .. # Linux
make -j4
sudo make install
cd ../..
rm -rf nng

echo "Install SatDump"
git clone https://github.com/altillimity/satdump.git
cd satdump
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr .. # Linux
make -j`nproc`
#make -j4
ln -s ../pipelines .        # Symlink pipelines so it can run
ln -s ../resources .        # Symlink resources so it can run
ln -s ../satdump_cfg.json . # Symlink settings so it can run
sudo make install
chmod +x ./satdump

echo "Make Satdum global name"
cd ~
text="export PATH=$PATH:~/$path"
name=`tail -1 '.bashrc'`
if [[ $name == $text ]] ; then
    echo "OK"
else 
    echo $text >> '.bashrc'
fi
source ~/.bashrc
