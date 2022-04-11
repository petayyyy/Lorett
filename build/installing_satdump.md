#!/bin/bash
path="Lorett"

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

check_python_version

echo "Install all packeges and libs for SatDump"
sudo apt install -y --fix-missing git build-essential cmake g++ pkgconf libfftw3-dev libvolk2-dev libjpeg-dev libpng-dev 
sudo apt install -y --fix-missing librtlsdr-dev libhackrf-dev libairspy-dev libairspyhf-dev                          
sudo apt install -y --fix-missing libglew-dev libglfw3-dev   
sudo apt install -y --fix-missing libzstd-dev   
sudo apt install -y --fix-missing xorg-dev
sudo apt-get install -y --fix-missing libglu1-mesa-dev freeglut3-dev mesa-common-dev       

pip$pyv install Mako

cd ~
cd catkin_ws/src/
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
