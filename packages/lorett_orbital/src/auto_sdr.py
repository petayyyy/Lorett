import SDRReader
import rospy
from lorett_orbital.srv import sdr_calibrate, sdr_calibrateResponse, sdr_recorder, sdr_recorderResponse
from lorett_orbital.msg import sdr_log

def calibrate(req):
    global sdr
    if sdr.load_config(req.satellite):
        noise_level = sdr.calibrate()
        return sdr_calibrateResponse("OK", noise_level)
    else:
        return sdr_calibrateResponse("ERR", 0) 

def recorder(req):
    global sdr, path
    fileName = SDRReader.datetime.utcnow().strftime("%Y%m%d_%H%M%S_") + sdr.config_name 

    # Start recording signal
    sdr.start(SDRReader.os.path.join(path, fileName + ".iq"), SDRReader.os.path.join(path, fileName + ".log"))
    # Wait untill we see satellite
    SDRReader.time.sleep(req.time_recording)
    # Stop recording, end of all process
    sdr.stop()


if __name__ == "__main__":
    global sdr, path 

    rospy.init_node('sdr_auto')

    path = '~/records/'

    pub = rospy.Publisher('sdr_log', sdr_log, queue_size=10)
    sdr = SDRReader.OSMO_SDR(SDRReader.SDR_CONFIGS, pub)

    rospy.Service("sdr_calibrate", sdr_calibrate, calibrate)
    rospy.Service("sdr_recorder", sdr_recorder, recorder)

    rospy.spin()