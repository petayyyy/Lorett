import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
import logging
import sys


rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

path = "drone_logs/"
#////////////////////////
# Fly 0 0
#xx, yy, zz = 0.02, 0.05, 0.81
#file_name = "calibr.log"
# METOP-B
xx, yy, zz = 0, 0, 0.81
file_name = path+"METOP.log"
# FENGYUN 3B
#xx, yy, zz = 0, 0, 0.81
#file_name = "FENGYUN_3B.log"
# FENGYUN 3C
#xx, yy, zz = 0, 0, 0.81
#file_name = "FENGYUN_3C.log"

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

navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)

navigate(x=1, y=0, z=0, speed=0.5, frame_id='body')
rospy.sleep(2)

navigate(x=0, y=0, z=zz, speed=0.5, frame_id='aruco_map')
rospy.sleep(3)

log = get_logger(file= file_name)
log.info("Corrent coordinat is: x= {0}, y= {1}, z= {2}".format(xx,yy,zz))
log.info("   x (del_x)         y (del_y)           z (del_z)        z_lazer")

for i in range(240):
        navigate(x=xx, y=yy, z=zz, speed=0.25, frame_id='aruco_map')
        rospy.sleep(0.5)
        telemetry = get_telemetry("aruco_map")
        log.info("{0} ({4})     {1} ({5})       {2} ({6})       {3}".format(round(telemetry.x, 3), round(telemetry.y, 3), round(telemetry.z, 3), round(rospy.wait_for_message('rangefinder/range', Range).range, 3), abs(round(telemetry.x-xx,3)), abs(round(telemetry.y - yy,3)), abs(round(telemetry.z - zz,3))))
land()
