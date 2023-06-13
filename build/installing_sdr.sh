#!/bin/bash
path="dependence"
path_sdr="sdr_dependence"

echo "Installing all dependence for SDR"
#sudo apt-get install -y airspy soapysdr-module-airspy soapysdr-tools libairspy-dev python3-soapysdr 
#sudo apt-get install libsoapysdr-dev -y

echo "Install libs for python3 SDR soft"
sudo apt-get install -y python3-scipy python3-matplotlib python3-numpy python3-docopt python3-prettytable python3-pyorbital
pip3 install thread6 -y
#pip3 install -y matplotlib>=3.3 requests>=2.26.0 pyorbital>=1.6.1 beautifulsoup4>=4.10.0 prettytable>=2.2.0

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
# cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local -DLIB_INSTALL_DIR:PATH=lib64 -DLIB_SUFFIX=64 -DSOAPY_SDR_ROOT=/usr/local ..
make -j4
sudo make install
sudo ldconfig
sudo add-apt-repository -y ppa:myriadrf/drivers

cd ../..
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
