# Using a rock pi S as a networked RSP remote with SoapyRemote

device: Radxa Rock Pi S (`https://wiki.radxa.com/RockpiS`)

known limitation: 100Mbit/s network bandwidth will most likely be the bottleneck rather than the CPU (eg. a 2.048Mhz sample rate pushes ~65Mbit/s network traffic, while `sdrplay_apiServ` and `SoapySDRServer` don't use more than 40% of a *single* core).

Steps:

Install armbian on the device and make sure it works properly:

- download armbian for the rock pi S from `https://www.armbian.com/radxa-rockpi-s/`
- copy the image file to the SD card: follow steps 1 to 3 from `https://wiki.radxa.com/RockpiS/getting_started`
- ssh in the device (see `https://docs.armbian.com/User-Guide_Getting-Started/`). Note: check your router/dhcp server's leases file to find the device's assigned IP.

Then cross-compile SoapyRemote and copy the files to the device as explained below.

## Cross-compiling SoapyRemote 

### Armbian toolchain

Get/extract the toolchain (from `https://dl.armbian.com/_toolchains/`):

```
mkdir $HOME/devel/
cd $HOME/devel/
wget https://dl.armbian.com/_toolchains/gcc-linaro-7.4.1-2019.02-x86_64_aarch64-linux-gnu.tar.xz
tar --strip-components=1 --one-top-level=gcc_aarch64 -xJf gcc-linaro-7.4.1-2019.02-x86_64_aarch64-linux-gnu.tar.xz
```

Copy the content below to `~/devel/toolchains.cmake`:

```
set(CMAKE_SYSTEM_NAME Linux)
set(buildpath /home/user/devel)
set(CMAKE_PREFIX_PATH ${buildpath}/SDRplay_RSP_API-ARM64/inc ; ${buildpath}/SDRplay_RSP_API-ARM64/aarch64)
set(SoapySDR_DIR ${buildpath}/target/usr/local/share/cmake/SoapySDR)
set(CMAKE_C_COMPILER ${buildpath}/gcc_aarch64/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${buildpath}/gcc_aarch64/bin/aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

### Building SDRplay API

https://www.sdrplay.com/arm64dl2.php

```
cd $HOME/devel/
chmod a+x SDRplay_RSP_API-ARM64-3.06.1.run
./SDRplay_RSP_API-ARM64-3.06.1.run --target SDRplay_RSP_API-ARM64
ln -s -r SDRplay_RSP_API-ARM64/aarch64/libsdrplay_api.so.3.06 SDRplay_RSP_API-ARM64/aarch64/libsdrplay_api.so
```


### Building SoapySDR

```
cd $HOME/devel/
git clone https://github.com/pothosware/SoapySDR.git
mkdir SoapySDR/build
cd SoapySDR/build
cmake -DCMAKE_TOOLCHAIN_FILE:PATH="../../toolchains.cmake" ..
make
make install DESTDIR=~/devel/target/
```


### Building SoapySDRPlay

```
cd $HOME/devel/
git clone -b API3+RSPduo https://github.com/fventuri/SoapySDRPlay ./SoapySDRPlay
mkdir ./SoapySDRPlay/build
cd ./SoapySDRPlay/build
cmake -DCMAKE_TOOLCHAIN_FILE:PATH="../../toolchains.cmake" ..
make
make install DESTDIR=~/devel/target/
```

### Building SoapyRemote

```
cd $HOME/devel/
git clone https://github.com/pothosware/SoapyRemote.git
mkdir SoapyRemote/build
cd SoapyRemote/build
cmake -DCMAKE_TOOLCHAIN_FILE:PATH="../../toolchains.cmake" ..
make
make install DESTDIR=~/devel/target/
```

## Installation on the Rock Pi S

- copy/install the SDR play API on the rock pi
- copy the content of `$HOME/devel/target` to the rock pi root fs.

Make libraries in `/usr/local/lib` available system-wide: copy the content below to `/etc/ld.so.conf.d/local.conf`:

```
/usr/local/lib
```

and run `sudo ldconfig`.

Check that libraries are properly found with `sudo ldconfig -p | grep Soapy` and/or `ldd /usr/local/bin/SoapySDRUtil`

Tweak rmem/wmem sysctl as per SoapyRemote's wiki: add the content below to `/etc/sysctl.d/sdr.conf`:

```
net.core.rmem_max=104857600
net.core.wmem_max=104857600
```

Add a systemd unit file for SDRplay API server: copy the content below to `/usr/local/lib/systemd/system/sdrplay_apiService.service`:

```
[Unit]
Description=SDRplay API server
Before=SoapySDRServer.service

[Service]
ExecStart=/usr/local/bin/sdrplay_apiService
KillMode=process
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable/start the servers:

```
sudo systemctl daemon-reload
sudo systemctl enable sdrplay_apiService.service
sudo systemctl start sdrplay_apiService.service
sudo systemctl enable SoapySDRServer
sudo systemctl start SoapySDRServer
```


Finally, check that everything's OK:

`SoapySDRUtil --find`:

```
######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################

Found device 0
  driver = remote
  label = SDRplay Dev0 RSPdx 191104XXXX
  remote = tcp://1.2.3.4:55132
  remote:driver = sdrPlay

Found device 1
  driver = sdrPlay
  label = SDRplay Dev0 RSPdx 191104XXXX
```



# Copter for space

## Команды:  
```bash
sudo apt-get install osmo-sdr soapysdr-module-osmosdr  
sudo apt-get install cmake g++ libpython3-dev python3-numpy swig
```
## Специальное оборудование 
Приёмная часть состоит из :  
* **Облучателя L-диапазона** (ряд металлизированных дисков разных размеров, закрепленных при помощи шпильки на некотором расстоянии друг от друга), малошумящего усилителя (МШУ) соответствующих частот, тройника смешения (специальное устройство, позволяющее подавать питание на МШУ и принимать сигнал по одному проводу) и программируемого радиоприемника (airspy sdr).  
  
* **МШУ** (Малошумящий усилитель) – устройство, входящее в состав базовой станции (БС) и используемое для повышения чувствительности приемника в восходящем направлении UL (Uplink).  
  
* **Software-defined radio/SDR** (рус. Программно определяемая радиосистема) — радиопередатчик и/или радиоприёмник, использующий технологию, позволяющую с помощью программного обеспечения устанавливать или изменять рабочие радиочастотные параметры, включая, в частности, диапазон частот, тип модуляции или выходную мощность, за исключением изменения рабочих параметров, используемых в ходе обычной предварительно определённой работы с предварительными установками радиоустройства, согласно той или иной спецификации или системы.  
  
* **Блок питания МШУ** – регуляция входного и выходного напряжений.  
  
* **Облучатель** − сосредоточенный элемент параболической антенны, находящийся в её фокусе (фазовом центре) или фокальной плоскости, формирующий диаграмму направленности и поляризацию антенны.  

## Программное обеспечение.
На основе открытого программного кода был создан автоматический демодулятор-декодер (далее Д-Д), который имеет название "SatDump". Позднее возможности Д-Д были оптимизированы для Raspberry Pi с некоторыми дополнительными функциями. Например, автоматическая демодуляция и декодирование данных с метеорологических спутников с возможностью просмотра спутниковых снимков в браузере.  
  
https://github.com/petayyyy/Lorett// – исходный код,  
https://gitlab.com/lpmrfentazis/HRPTAutoDecoder – автодекодер,  
https://gitlab.com/lpmrfentazis/lorettorbital/-/blob/develop/lorettOrbital/orbital.py – библиотека «LoReTT Orbital» для расчёта расписания пролётов спутников и создания траектории перемещения облучателя дроном в фокальной плоскости «зеркала»,  
https://github.com/petayyyy/Lorett// – сборник ПО для функционирования комплекса-конструктора, описанного в «ГитБуке».

### Документация.
Концепция конструктора подразумевает возможность изучения и освоения обучающимися определенных компетенций в областях инжиниринга, программирования, радиотехники, летающей робототехники и работы с данными дистанционного зондирования Земли (ДЗЗ). Для комфортного изучения мы подготовили все необходимые учебные материалы:  
https://disk.yandex.ru/i/hJjB1w0ekQ0Lug – руководство по эксплуатации комплекса,  
https://petayyyy.gitbook.io/copter-for-space/ – инструкция по сборке, настройке и эксплуатации комплекса в удобном формате «ГитБука»,  
https://disk.yandex.ru/d/WOpfbO74N2cmPQ – инструкция по демодуляции и декодированию сигнала с метеорологических спутников, а также анализу полученных космических снимков.

Для приобретения специального оборудования обратитесь к представителям Инженерной компании «Лоретт» ввиду эксклюзивности данных компонентов.   

Наши контакты: ООО «Лоретт», Россия, г. Москва, Инновационный центр «Сколково», Большой бульвар, 42, стр. 1, офис 334, 121205, +7 (985) 727-7630 contact@lorett.org. 
![image](https://user-images.githubusercontent.com/47917455/192191680-17f320b2-5eeb-4cb9-b11b-8b5cc5905faa.png)

## Ссылки:
* Ссылка на вебинары: https://disk.yandex.ru/d/V-6W4ZBmIXW2ig
* Ссылка на файлы симулятора Gazebo: https://disk.yandex.ru/d/ZhsWwNCgPYwffQ
* Ссылка на установку симулятора Gazebo: https://github.com/petayyyy/Nti/tree/main/Gazebo
* Ссылка на статью о прошлом проекта: https://lorett.org/news/31_08_2021_intera_2021_sostoialsia_final_napravlieniia_kosmichieskaia_razviedka_
