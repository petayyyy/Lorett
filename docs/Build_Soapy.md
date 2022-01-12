# Сборка SoapySdr
### Для запуска кода необходимо устаноить все зависимости и библиотеки:
always update
```bash
sudo apt-get update
```
```bash
sudo apt-get install python3-pothos
```
python3 language bindings
```bash
sudo apt-get install python3-soapysdr python3-numpy
```
airspy support:
```bash
sudo apt-get install airspy soapysdr-module-airspy
```
print information about the install
```bash
PothosUtil --system-info
SoapySDRUtil --info
```
```bash
cd catkin_ws/src/
git clone https://github.com/pothosware/SoapySDR.git
cd SoapySDR
git pull origin master
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
## Проверка все ли зависимости встали SoapySdr
```bash
sudo ldconfig #needed on debian systems
SoapySDRUtil --info
```
## Подготовка к установке SoapyAirspy
```bash
cd ../..
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr -DLIB_INSTALL_DIR:PATH=lib64 -DLIB_SUFFIX=64 -DSOAPY_SDR_ROOT=/usr ..
sudo add-apt-repository -y ppa:myriadrf/drivers
sudo apt-get update
sudo apt-get install airspy libairspy-dev
```
## Установка SoapyAirspy
```bash
git clone https://github.com/pothosware/SoapyAirspy.git
cd SoapyAirspy
mkdir build
cd build
cmake ..
make
sudo make install
```
## Проверка все ли зависимости встали SoapyAirspy
```bash
SoapySDRUtil --probe="driver=airspy"
```
## Установка недостающих библиотек
```bash
pip3 install thread6
sudo apt-get install python3-matplotlib
sudo apt-get install python3-pothos
sudo apt-get install python3-soapysdr python3-numpy
sudo apt-get install python3-scipy
```
