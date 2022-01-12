# Руководство по установке ПО необходимого для работы комплекса
## 1. Установка образа Clover.
Первым делом необходимо установить образ для микрокомпьютера на свой компьтер, для этого переходим по ссылке(https://github.com/CopterExpress/clover/releases).
После этого выбираем версию (21.2 на melodic и имеет 2 и 3 python, 22.0 на noetic и имеет только 3 python) и устанавливаем её (Рис.1). 
![Screenshot 2022-01-12 152939](https://user-images.githubusercontent.com/47917455/149140584-21deaff7-a31d-41aa-a9a1-460f88e38719.png)
Рис.1 
  

Устанавлваем BalenaEtcher с официального сайта (https://www.balena.io/etcher/) (Рис.2).
![1111](https://user-images.githubusercontent.com/47917455/149141589-f634c9c3-120a-42ad-84e5-a0847fee4e3a.png)
Рис.2 
  
Вставляем Sd карточку в ноутбук и открываем BalenaEtcher. После этого в поле "Flash from file" выбираем установленный образ, а в "Select target" выбираем Sd  карточку на 32 гб (если карточка не отображается обновите драйверы или поменяйте переходник) и нажимаем на "Flash!", после этого начнется процесс записи образа (Рис.3).
![1111](https://user-images.githubusercontent.com/47917455/149143810-6cef51eb-1ff8-4ce0-99ec-dfdedf833457.png)
Рис.3  

Более подробно данный этап описан в докуметации Clover (https://clover.coex.tech/ru/image.html)
## 2. Установка зависимостей  на дроне.
По данному пункту есть отдельное поссобие [Build_Soapy](Build_Soapy.md). Оно описывает установку всех зависимостей для записи данных с помощью SDR.
## 3. Установка необходимых файлов на дрон.
Для установки файлов с ПО необходимо установить репозиторий проекта.
```bash
git clone https://github.com/petayyyy/Lorett.git
```
После необходимо в него перейти, для работы с комплексом.
```bash
cd Lorett
```
В этом репозитории лежат файлы:  
*Зависания в зените спутника [fly_zzen.py](https://github.com/petayyyy/Lorett/blob/main/fly_zzen.py).  
*Пролет по траектории спутника [fly_track.py](https://github.com/petayyyy/Lorett/blob/main/fly_track.py).  
*Создание трека, в соответствии с выбранным спутником для определенного места (данные широты, долготы и высоты над уровнем моря изменяются в файле)  [getPass_file.py](https://github.com/petayyyy/Lorett/blob/main/getPass_file.py).  
*Запись данных с помощью SDR (данные спутника изменяются в файле) [sdr_python3.py](https://github.com/petayyyy/Lorett/blob/main/sdr_python3.py).  
*Параметры полетного контроллера [Stable_param.params](https://github.com/petayyyy/Lorett/blob/main/FCU/Stable_param.params).  
*В папке "docs" лежат все документации с которыми стоит ознакомиться для корректной работы с Комплексом [docs](https://github.com/petayyyy/Lorett/tree/main/docs).
## 4. Установка ПО на ноутбук.
Для удобного взаимодействия с дроном необходимо установить ряд приложений на ваше устройство:  
* QGroundControl http://qgroundcontrol.com/downloads/  (Рис.4)
![1111](https://user-images.githubusercontent.com/47917455/149153282-13fe328c-e0c2-4188-8e9c-b607d4075244.png)
Рис.4  
* WinSCP https://winscp.net/eng/download.php (Рис.5)
![1111](https://user-images.githubusercontent.com/47917455/149153486-6111996a-a5e6-42cb-83a8-2918117dade3.png)
Рис.5  
