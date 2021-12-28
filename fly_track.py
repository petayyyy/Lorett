import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
import logging
import sys
import os
import subprocess
from config import *
from datetime import datetime

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

path = "drone_logs/"

def get_logger(name=__file__, file='log.txt', encoding='utf-8'):
    log = logging.getLogger(name)
    log.setLevel(logging.INFO)

    formatter = logging.Formatter(' %(message)s')

    fh = logging.FileHandler(file, encoding=encoding)
    fh.setFormatter(formatter)
    log.addHandler(fh)

    sh = logging.StreamHandler(stream=sys.stdout)
    sh.setFormatter(formatter)
    log.addHandler(sh)

    return log

navigate(x=0, y=0, z=1.3, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)

navigate(x=1, y=0, z=0, speed=0.5, frame_id='body')
rospy.sleep(2)

# Start Sdr calibration
#process = subprocess.Popen('/usr/bin/python3 ~/sdr_2_python3.py')
#process.start()

navigate(x=0, y=0, z=zz, speed=0.5, frame_id='aruco_map')
rospy.sleep(3)
print("Start Calibrate sdr please")

#navigate(x=x_apogee, y=y_apogee, z=zz, speed=0.5, frame_id='aruco_map')
#rospy.sleep(3)

#print(sateline_name + "_" + "_".join([str(j) for j in data[0][:3]]) + ".log")
#log = get_logger(file= path+ sateline_name + "_" + "_".join([str(j) for j in data[0][:3]]) + ".log")
#log.info("Corrent coordinat is: x= {0}, y= {1}, z= {2}".format(x_apogee,y_apogee,zz))
#log.info("   x (del_x)         y (del_y)           z (del_z)        z_lazer")


for f in data:
	navigate(x = f[3], y = f[4], z = zz, speed=0.5, frame_id = "aruco_map")
	while not rospy.is_shutdown():
		t = (f[0] - datetime.now().time().hour)*3600 + (f[1] - datetime.now().time().minute)*60 + (f[2] - datetime.now().time().second)
		if t == 0:
			rospy.sleep(0.1)
			break
		elif t < 0: break
		navigate(x=f[3], y=f[4], z=zz, frame_id="aruco_map", speed = 0.5)
		print("Delta time is {} seconds".format(t))
		rospy.sleep(0.2)
"""
for i in range(time_delta*2):
        navigate(x=x_apogee, y=y_apogee, z=zz, speed=0.25, frame_id='aruco_map')
        rospy.sleep(0.5)
        telemetry = get_telemetry("aruco_map")
        log.info("{0} ({3})     {1} ({4})       {2} ({5})".format(round(telemetry.x, 3), round(telemetry.y, 3), round(telemetry.z, 3), abs(round(telemetry.x-x_apogee,3)), abs(round(telemetry.y - y_apogee,3)), abs(round(telemetry.z - zz,3))))
"""
land()
