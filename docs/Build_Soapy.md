# Сборка SoapySdr
Обновим зависимости и установим pothos
```bash
sudo apt-get update
sudo apt-get install python3-pothos
```
Поставим soapysdr и numpy для python3
```bash
sudo apt-get install python3-soapysdr python3-numpy
```
Устновим airspy со всеми зависимостями
```bash
sudo apt-get install airspy soapysdr-module-airspy
```
Проверим все ли установилось
```bash
PothosUtil --system-info
SoapySDRUtil --info
```
Поставим soapysdr на систему для дальнейшей записи данных
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
### Проверка все ли зависимости встали soapysdr
```bash
sudo ldconfig
SoapySDRUtil --info
```
Подготовка к установке SoapyAirspy
```bash
cd ../..
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr -DLIB_INSTALL_DIR:PATH=lib64 -DLIB_SUFFIX=64 -DSOAPY_SDR_ROOT=/usr ..
sudo add-apt-repository -y ppa:myriadrf/drivers
sudo apt-get update
sudo apt-get install airspy libairspy-dev
```
Установка SoapyAirspy
```bash
git clone https://github.com/pothosware/SoapyAirspy.git
cd SoapyAirspy
mkdir build
cd build
cmake ..
make
sudo make install
```
### Проверка все ли зависимости встали SoapyAirspy
```bash
SoapySDRUtil --probe="driver=airspy"
```
### Установка недостающих библиотек необходимых для записи сигнала
```bash
pip3 install thread6
sudo apt-get install python3-matplotlib
sudo apt-get install python3-soapysdr python3-numpy
sudo apt-get install python3-scipy
```
