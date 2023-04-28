## Запуск скрипта для установки всех зависимостей, библиотек и настроек.  
Для установки, необходимо ввести команду:  
```bash
bash ./install_all_dependence.sh 
```
## Запуск скрипта для установки зависимостей, библиотек и настроек для записи данных с SDR.  
Для установки, необходимо ввести команду:  
```bash
bash ./install_software.sh 
```
## Запуск скрипта для установки Satdump на Raspberry.  
Для установки, необходимо ввести команду:  
```bash
bash ./install_satdump.sh 
```
## Запуск скрипта для скачивания и упаковки всех зависимостей, библиотек и настроек в tar файл.   
Для скачивания, необходимо ввести команду:  
```bash
bash ./download_all_dependence.sh 
```
Все файлы закакованны в Dependence.tgz . Вы можете посмотреть его содержимое перейде я в директорию Dependence командой:
```bash
cd Dependence
```
А также в случае транпортировки *.tgz* файла вы можете его разархивировать его на другом устройстве командой:
```bash
tar zxvf Dependence.tgz
```
## Запуск скрипта для просмотра системной версии python.   
Для скачивания, необходимо ввести команду:  
```bash
bash ./check_python_version.sh
```
![image](https://user-images.githubusercontent.com/47917455/172495093-6718f3ff-1421-4856-92b2-6ddf1f74c5bd.png)
