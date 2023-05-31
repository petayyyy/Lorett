#!/bin/bash
path="dependence"
path_sdr="sdr_dependence"

echo "Installing all dependence for SDR"
sudo apt-get install -y airspy soapysdr-module-airspy soapysdr-tools libairspy-dev

echo "Install libs for python3 SDR soft"
sudo apt-get install -y --fix-missing python3-scipy python3-matplotlib python3-soapysdr python3-numpy python3-docopt

echo "Create dependence directory"
cd ~
if [ ! -f  "/"$path ]; then
    mkdir $path
fi
cd $path
if [ ! -f  "/"$path_sdr ]; then
    mkdir $path_sdr
fi
cd $path_sdr

echo "Intall SoapySDR"
git clone https://github.com/pothosware/SoapySDR.git 2>&1 
cd SoapySDR
git pull origin master 2>&1
mkdir build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
sudo add-apt-repository -y ppa:myriadrf/drivers
# cd ../..
# cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr -DLIB_INSTALL_DIR:PATH=lib64 -DLIB_SUFFIX=64 -DSOAPY_SDR_ROOT=/usr ..

cd ..
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
