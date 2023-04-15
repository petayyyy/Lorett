#!/usr/bin/env python3
import rospy
import SDRReader
import os
import datetime
from sdrr.msg import sdr_log

from tracks.msg import TrackCoord





def calibrate(req):
    global sdr
    if sdr.load_config(req):
        noise_level = sdr.calibrate()
        return sdr.sdr_calibtrate("OK", noise_level)
    else:
        return sdr.sdr_calibtrate("ERR", 0) 

def recorder(req):
    global sdr, path
    fileName = datetime.utcnow().strftime("%Y%m%d_%H%M%S_") + sdr.config_name 

    sdr.start(os.path.join(path, fileName + ".iq"), os.path.join(path, fileName + ".log"))

    rospy.time.sleep(req.time_recording)

    sdr.stop()

def track(req):
    if(req.status == 'cal'):
        calibrate(req.satname)
    elif(req.status == 'pr'):
        fileName = datetime.utcnow().strftime("%Y%m%d_%H%M%S_") + sdr.config_name 

        sdr.start(os.path.join(path, fileName + ".iq"), os.path.join(path, fileName + ".log"))
    elif(req.status == 'stop'):
        sdr.stop()



if __name__ == "__main__":
    global sdr, path 

    rospy.init_node('sdr_auto')


    path = '~/records/'

    pub = rospy.Publisher('sdr_log', sdr_log, queue_size=10)
    sdr = SDRReader.OSMO_SDR(SDRReader.SDR_CONFIGS, pub)

    rospy.Subscriber('/coords', TrackCoord, track)


    rospy.spin()