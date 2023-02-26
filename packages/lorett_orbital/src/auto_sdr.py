import SDRReader
import rospy
from lorett_orbital.srv import sdr_calibrate, sdr_recorder
from lorett_orbital.msg import sdr_log

if __name__ == "__main__":
    global sdr
    rospy.init_node('sdr_auto')
    pub = rospy.Publisher('sdr_log', sdr_log, queue_size=10)
    sdr = SDRReader.OSMO_SDR(SDRReader.SDR_CONFIGS, pub)
    rospy.spin()