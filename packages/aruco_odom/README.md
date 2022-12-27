# Aruco_odom
## Aruco Intel Realsense t265
Данный модуль для ROS, проверенн и работает на симуляторе версии 0.3 в Ubuntu 18.04. Позволит вам на основе данных одонометрии с камеры Intel Realsense t265 координироваться на основе карты с аруко маркерами использую пакет [clover](https://github.com/CopterExpress/clover) и [odom_tf2](https://github.com/Elur12/odom_tf2)
## Установка
Загрузите и установите пакет
```bash
cd ~/catkin_ws/src
git clone https://github.com/Elur12/aruco_odom
cd aruco_odom
chmod +x nodes/aruco_tf2.py
chmod +x nodes/odom_and_aruco.py
cd ~/catkin_ws
catkin_make
sourse ~/.bashrc
```

Добавьте эти строки в файл [clover.launch](https://github.com/CopterExpress/clover/blob/master/clover/launch/clover.launch)
```bash
<node name="aruco_tf2_1" pkg="aruco_odom" type="aruco_tf2.py" respawn="false" output="screen" >
        <param name="num" type = "string" value="1"/>
        <param name="map_file" type = "string" value="/home/clover/catkin_ws/src/aruco_odom/nodes/C4S_map.txt"/>
        <param name="fx" type = "string" value="285.222808837891"/>
        <param name="fy" type = "string" value="285.393310546875"/>
        <param name="ppx" type = "string" value="424.528411865234"/>
        <param name="ppy" type = "string" value="405.992706298828"/>
        <param name="c1" type = "string" value="-0.0040830047801137"/>
        <param name="c2" type = "string" value="0.0394832715392113"/>
        <param name="c3" type = "string" value="-0.0364591591060162"/>
        <param name="c4" type = "string" value="0.00567311979830265"/>
    </node>
    <node name="aruco_tf2_2" pkg="aruco_odom" type="aruco_tf2.py" respawn="false" output="screen" >
        <param name="num" type = "string" value="2"/>
        <param name="map_file" type = "string" value="/home/clover/catkin_ws/src/aruco_odom/nodes/C4S_map.txt"/>
        <param name="fx" type = "string" value="285.219696044922"/>
        <param name="fy" type = "string" value="285.371307373047"/>
        <param name="ppx" type = "string" value="426.107391357422"/>
        <param name="ppy" type = "string" value="402.454803466797"/>
        <param name="c1" type = "string" value="-0.0029170720372349"/>
        <param name="c2" type = "string" value="0.0377450212836266"/>
        <param name="c3" type = "string" value="-0.0351304709911346"/>
        <param name="c4" type = "string" value="0.0052577848546207"/>
    </node>
    <node name="odom_and_aruco" pkg="aruco_odom" type="odom_and_aruco.py" respawn="false" output="screen"/>
```

## Использование
Пакет добавляет новый frame_id. Если указать "aruco_odom_map" в параметр frame_id. То позиционирование дрона будет осуществляться на основе топика /camera/odom/sample и данных с камер /camera/fisheye1 и /camera/fisheye2 по карте маркеров. 
Для использования обязательно находиться над полем. Проверить распознаны ли все маркеры можно в топиках /aruco_tf2_1/debug_camera_1 и /aruco_tf2_2/debug_camera_2
## Примеры использования
1. В коде
```bash
import rospy
from clover import srv

navigate = rospy.ServiceProxy('navigate', srv.Navigate)

navigate(x=0.5, y=0.5, z=1.5, frame_id='aruco_odom_map', auto_arm=True)
```

2. Через консоль
```bash
rosservice call /navigate "{x: 0.5, y: 0.5, z: 1.0, yaw: 0.0, yaw_rate: 0.0, speed: 0.0, frame_id: 'aruco_odom_map', auto_arm: true}" 
```

## Настройка
Для установки файла карты аруко, поместите его в любое место и в файле [clover.launch](https://github.com/CopterExpress/clover/blob/master/clover/launch/clover.launch) в ДВУХ местах укаите путь к файлу.