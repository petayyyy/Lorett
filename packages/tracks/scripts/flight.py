import rospy
from lorett_c4s import srv as lorett_srv
import tracks.msg
from std_srvs import srv as std_srv

rospy.init_node('flight')

navigate = rospy.ServiceProxy('navigate', lorett_srv.Navigate)
land = rospy.ServiceProxy('land', std_srv.Trigger)

# Parameters
tkfAlt = 1.46
x_center = 1.77
bool_start_track = False

def main(data):
    global bool_start_track
    if data.status == 'calling':
        tkf(data.x, data.y, data.z)
        bool_start_track = True
    if data.status == 'receiving':
        if bool_start_track == False: 
            tkf(data.x, data.y, data.z)
            bool_start_track = True
        flght(data.x, data.y, data.z)
    if data.status == 'stop':
        virubay(data.x, data.y, data.z)
        bool_start_track = False

def tkf(x, y, z):
    global tkfAlt, x_center
    navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    print('Armed, liftoff to 1m done')
    rospy.sleep(2)

    navigate(x=0, y=0, z=tkfAlt, frame_id='map')
    rospy.sleep(4)

    navigate(x=x_center, y=0, z=tkfAlt, frame_id='map')
    rospy.sleep(4)
    print('Succesfully tookoff, proceeding to the flight')

def flght(x, y, z):
    global tkfAlt, x_center
    navigate(x=x+x_center, y=y, z=tkfAlt, frame_id='map')
    print("Drone flight in position x:{} y:{}".format(x+x_center, y))
    rospy.sleep(0.3)
    navigate(x=x+x_center, y=y, z=tkfAlt, frame_id='map')
    print('.')

def virubay(x, y, z):
    global tkfAlt
    navigate(x=0, y=0, z=tkfAlt, frame_id='map')
    print('moved to home point')
    rospy.sleep(6)
    land()
    print('We landed, kind of...')

if __name__ == "__main__":
    rospy.Subscriber('/coords', tracks.msg.TrackCoord, main)
    rospy.spin()
