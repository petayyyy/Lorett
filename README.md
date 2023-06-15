# Copter for space
## Установка ПО:
Обновление всех пакетов микрокомпьютера
```bash
sudo apt update
```
Установка пакетов комплекса
```bash
sudo apt install wget -y
sudo apt install git -y
git clone https://github.com/petayyyy/Lorett.git
```
Recommend installing avahi on Ubuntu (used for device discovery):
```bash
sudo apt-get install avahi-daemon libavahi-client-dev 
```
## Команды: 
Подключеник к wifi сети:
```bash
sudo su
```
```bash
xargs rm < install_manifest.txt
```
```bash
nmcli r wifi on
```
Example, wifi_name is your SSID and wifi_password is password of network
```bash
nmcli dev wifi connect "wifi_name" password "wifi_password"
```
```bash
nmcli dev wifi connect "216_5G" password "KVANT216"
```
```bash
sudo apt-get install osmo-sdr soapysdr-module-osmosdr  
sudo apt-get install cmake g++ libpython3-dev python3-numpy swig
```
```bash
sudo make uninstall && make clean
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
