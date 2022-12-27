# Aruco_odom
## Aruco Intel Realsense t265
Данный модуль для ROS, проверенн и работает на симуляторе версии 0.3 в Ubuntu 18.04. Позволит вам на основе данных одонометрии с камеры Intel Realsense t265 координироваться на основе карты с аруко маркерами использую пакет [clover](https://github.com/CopterExpress/clover) и [odom_tf2](https://github.com/Elur12/odom_tf2)

## Запуск 
Запуск пакета осуществляется из launch файла:
```bash
roslaunch aruco_odom aruco_map_odometry.launch
```
Либо есть возможность добавить запуск пакета в ваш launch файл. Для этого необходимо добвить:
```bash
<!-- Start create odom by aruco and intel t265 -->
<include file="$(find aruco_odom)/launch/aruco_map_odometry.launch"/>
```
## Использование
Пакет добавляет новый frame_id. Если указать "aruco_odom_map" в параметр frame_id. То позиционирование дрона будет осуществляться на основе топика /camera/odom/sample и данных с камер /camera/fisheye1 и /camera/fisheye2 по карте маркеров. 
Для использования обязательно находиться над полем. Проверить распознаны ли все маркеры можно в топиках /aruco_tf2_1/debug_camera_1 и /aruco_tf2_2/debug_camera_2

## Примеры использования
1. В коде
```bash
import rospy
from lorett_c4s import srv

point = rospy.ServiceProxy('lorett/point', lorett_c4s.publishPose)
point(x=0.5, y=0.5, z=1.5, frame_id='aruco_odom_map')

```

2. Через консоль
```bash
rosservice call /lorett/point "{x: 0.5, y: 0.5, z: 1.0, frame_id: 'aruco_odom_map'}" 
```
