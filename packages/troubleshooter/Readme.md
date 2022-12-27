Если во время сборки вылезает ошибка связанная с невозможностью найти файлы с форматом ".h" то необходимо заменить папку ```lorett_c4s```:
```bash
sudo rm -r ~/catkin_ws/devel/include/lorett_c4s
mv lorett_c4s ~/catkin_ws/devel/include/lorett_c4s
```
