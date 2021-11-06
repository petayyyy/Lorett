# Lorett

Для запуска кода необходимо устаноить все зависимости и библиотеки:

#core framework and toolkits (required)
```bash
sudo add-apt-repository -y ppa:pothosware/framework
```
#support libraries for pothos (required)
```bash
sudo add-apt-repository -y ppa:pothosware/support
```
#supplies soapysdr, and drivers (optional)
```bash
sudo add-apt-repository -y ppa:myriadrf/drivers
```
#needed when using bladerf devices
```bash
sudo add-apt-repository -y ppa:bladerf/bladerf
```
#needed when using usrp devices
sudo add-apt-repository -y ppa:ettusresearch/uhd

#always update after adding PPAs
sudo apt-get update

sudo apt-get install pothos-all

#or install bindings for python3
sudo apt-get install python3-pothos

#install development files for python blocks
sudo apt-get install pothos-python-dev

#soapy sdr runtime and utilities
sudo apt-get install soapysdr-tools

#python3 language bindings
sudo apt-get install python3-soapysdr python3-numpy

#using soapy sdr for remote device support?
sudo apt-get install soapysdr-module-remote soapysdr-server

#osmo sdr support:
sudo apt-get install osmo-sdr soapysdr-module-osmosdr

#rtl sdr support:
sudo apt-get install rtl-sdr soapysdr-module-rtlsdr

#blade rf support:
sudo apt-get install bladerf soapysdr-module-bladerf

#hack rf support:
sudo apt-get install hackrf soapysdr-module-hackrf

#usrp support:
sudo apt-get install uhd-host uhd-soapysdr soapysdr-module-uhd

#miri SDR support:
sudo apt-get install miri-sdr soapysdr-module-mirisdr

#rf space support:
sudo apt-get install soapysdr-module-rfspace

#airspy support:
sudo apt-get install airspy soapysdr-module-airspy

#print information about the install
PothosUtil --system-info
SoapySDRUtil --info

#run the design GUI -- there should also be a menu shortcut
PothosFlow

sudo apt-get install \
    cmake g++ \
    libpython-dev python-numpy swig

git clone https://github.com/pothosware/SoapySDR.git
cd SoapySDR

git pull origin master

mkdir build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig #needed on debian systems
SoapySDRUtil --info

cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr -DLIB_INSTALL_DIR:PATH=lib64 -DLIB_SUFFIX=64 -DSOAPY_SDR_ROOT=/usr ..

sudo add-apt-repository -y ppa:myriadrf/drivers
sudo apt-get update
sudo apt-get install airspy libairspy-dev

git clone https://github.com/pothosware/SoapyAirspy.git
cd SoapyAirspy
mkdir build
cd build
cmake ..
make
sudo make install

SoapySDRUtil --probe="driver=airspy"
