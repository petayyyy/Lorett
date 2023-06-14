import rospy
from datetime import datetime


from tracks.msg import TrackCoord, info


def pos_callback(msg):
    print(datetime.utcnow().timestamp() - msg.z)

rospy.init_node("fly")

rospy.Subscriber('/coords', TrackCoord, pos_callback)
rospy.spin()