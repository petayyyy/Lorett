# -*- coding: utf-8 -*-
import rospy
import math
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0.6, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def fly_drone(pos):
    for p in pos:
        navigate_wait(x = p[0], y = p[1], z = p[2])
        print("x = {} y = {} z = {}".format(p[0],p[1],p[2]))

print navigate(x=0, y=0, z=2, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(4)

position = [[0, 0, 0.6], [0.5, 0, 0.6], [0, 0.5, 0.6], [-0.5, 0, 0.6], [0, -0.5, 0.6]]
fly_drone(position)

navigate(x=0, y=0, z=2, speed=0.5, frame_id='map')
rospy.sleep(4)

navigate(x=0, y=0, z=1, speed=0.5, frame_id='aruco_6')
rospy.sleep(4)
land()

print("End")
