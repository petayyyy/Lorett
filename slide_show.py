from turtle import pu
import cv2 
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from docopt import docopt
bridge = CvBridge()

USAGE = '''

slaid_show - Script for watching decoding images from meteorological satellites.

Example:
  rosrun lorett slaid_show.py --in 'Noaa 19' --t 1
  rosrun lorett slaid_show.py -a | --auto
  rosrun lorett slaid_show.py -h | --help
  ***Don't forget about " or ' then you have space in naming. Also you can see example, use --h or -help then you start program.

Usage:
  sdr_python3.py [--in='<name>'] [--t=<sec>] [--p=<name>] [--out_iq=<bool>] [--out_con=<bool>]
  sdr_python3.py -h | --help
  sdr_python3.py -a | --auto
  sdr_python3.py -l | --last

Options:
  -h, --help             Show correct format parameters.
  -a, --auto             Flag working in autonov secions.
  -l, --last             Show latest create path in path.
  --in='<name>'          Name of file or path, that you need to see [default: /home/clover/Desktop/Metop].
  --t=<sec>              Time see one pictures, in seconds [default: 1].
  --p=<name>             Name of tracks path [default: /home/clover/tracks].
  --out_topic=<bool>     Flag publish pictures in topic [default: True].
  --out_con=<bool>       Flag printing console useless data [default: True].
'''
def sdr_work_server(req):
    global sdrr, satellite, time_recording, path
    print(req)
    try:
        if req.action == "calibrate":
            # Start work with airspy-sdr
            sdrr = OSMO_SDR(SDR_CONFIGS)
            # Configurate/calibrate sdr by name of satellite
            if req.satellite != '': 
                if sdrr.load_config(req.satellite): sdrr.calibrate()
            else: 
                if sdrr.load_config(satellite): sdrr.calibrate()
            return srv.sdr_recorder_rosResponse(process = "calibrate")
        elif req.action == "start":
            try:
                if sdrr == None:
                    # Start work with airspy-sdr
                    sdrr = OSMO_SDR(SDR_CONFIGS)
                    # Configurate/calibrate sdr by name of satellite
                    if req.satellite != '': 
                        if sdrr.load_config(req.satellite): sdrr.calibrate()
                    else: 
                        if sdrr.load_config(satellite): sdrr.calibrate()
                    time.sleep(1)
            except: 
                # Start work with airspy-sdr
                sdrr = OSMO_SDR(SDR_CONFIGS)
                # Configurate/calibrate sdr by name of satellite
                if req.satellite != '': 
                    if sdrr.load_config(req.satellite): sdrr.calibrate()
                else: 
                    if sdrr.load_config(satellite): sdrr.calibrate()
                time.sleep(1)
            # Generate file name of path recording
            fileName = "{0}_{1:m%m_day%d_h%H_min%M_}".format(sdrr.config_name.replace(" ", "_"), datetime.utcnow())
            # Create path for all signals, if we don't have
            if req.path != '': 
                if not os.path.exists(req.path): os.makedirs(req.path) 
                # Create path for now signal
                os.mkdir("{0}/{1}".format(req.path,fileName))
                # Generate file name of file recording
                fileName = "{0}/{1}/{2}".format(req.path, fileName, fileName)
            else: 
                if not os.path.exists(path): os.makedirs(path) 
                # Create path for now signal
                os.mkdir("{0}/{1}".format(path,fileName))
                # Generate file name of file recording
                fileName = "{0}/{1}/{2}".format(path, fileName, fileName)
            # Start recording signal
            sdrr.start("{0}.iq".format(fileName),"{0}.log".format(fileName),"")
            # Wait untill we see satellite
            if req.time_recording != 0: time.sleep(req.time_recording)
            else: time.sleep(time_recording)
            # Stop recording, end of all process
            sdrr.stop()
            return srv.sdr_recorder_rosResponse(process = "start")
        elif req.action == "kill" or req.action == "stop" or req.action == "exit":
            sdrr = None
            exit()
        else: return srv.sdr_recorder_rosResponse(process = "error")
    except:   return srv.sdr_recorder_rosResponse(process = "error")

class Slaid_show:
    def __init__(self):
        self.is_background = True
        self.is_resize_pictures = True
        self.color_background = (255, 255, 255)
        self.color_text = (0, 0, 0)
        self.size_pictures_x = 1900
        self.size_pictures_y = 800
        self.time_delay = 2
        self.pub_in_topic = True
        self.pub_in_console = False
        self.name_window = "Images from satellite"
        self.bridge = CvBridge()
    def change_param(self, color_text = (0, 0, 0), color_background = (255, 255, 255), size_pictures_x = 1900, size_pictures_y = 800, time_delta = 2, name_window = "Images from satellite", pub_in_topic = True, pub_in_console = False, is_resize_pictures = True, is_background = True):
        self.is_background = is_background
        self.is_resize_pictures = is_resize_pictures
        self.color_background = color_background
        self.color_text = color_text
        self.size_pictures_x = size_pictures_x
        self.size_pictures_y = size_pictures_y
        self.time_delay = time_delay
        self.pub_in_topic = pub_in_topic
        self.pub_in_console = pub_in_console
        self.name_window = name_window
    def search_images(self, path= "/home/clover/Desktop/Metop"):
        images, flag = [], False
        if os.path.exists(path):
            for file in os.listdir(path):
                if os.path.isdir(path+"/"+file):
                    for pictures in os.listdir(path+"/"+file):
                        if ".png" in pictures or ".jpg" in pictures:
                            images.append(file+"/"+ str(pictures))
                            flag = True  
            if flag == False:
                print("yours path is empty")
            return images
    def search_max_memory_object(self, path):
        max_data_create = 0
        name = 'error'
        if os.path.exists(path):
            for file in os.listdir(path):
                if os.path.isdir(path+"/"+file):
                    data_create = os.stat(path+"/"+file).st_size
                    if max_data_create < data_create:
                        max_data_create = data_create
                        name = path+"/"+file
                else:
                    data_create = os.stat(path+"/"+file).st_size
                    if max_data_create < data_create and (".png" in file or ".jpg" in file):
                        max_data_create = data_create
                        name = path+"/"+file
        return name
    def search_last_created_object(self, path):
        max_data_create = 0
        name = 'error'
        if os.path.exists(path):
            for file in os.listdir(path):
                if os.path.isdir(path+"/"+file):
                    data_create = os.stat(path+"/"+file).st_ctime
                    if max_data_create < data_create:
                        max_data_create = data_create
                        name = path+"/"+file
                else:
                    data_create = os.stat(path+"/"+file).st_ctime
                    if max_data_create < data_create and (".png" in file or ".jpg" in file):
                        max_data_create = data_create
                        name = path+"/"+file
        return name
    def pictures_naming(self, img, name):
        x, y = 23*len(name), 35
        if self.is_resize_pictures: img = cv2.resize(img, (self.size_pictures_x, self.size_pictures_y))
        if self.is_background:  cv2.rectangle(img,(0,0),(x,y),self.color_background,-1)
        cv2.line(img, (0,y), (x,y), self.color_text, 2) # Line left to right
        cv2.line(img, (x,0), (x,y), self.color_text, 2) # Line up to down
        cv2.putText(img, name, (10, y-10), cv2.FONT_HERSHEY_TRIPLEX, 1, self.color_text)
        return img
    def publish(self, img):
        try:
            if self.pub_in_topic:
                self.pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))
                rospy.sleep(self.time_delay)
            if self.pub_in_console:
                cv2.imshow(self.name_window, img)
                cv2.waitKey(0)
        except:
            rospy.init_node('slaid_show_server')
            self.pub = rospy.Publisher("image", Image, queue_size=1000)
            img = cv2.imread(img)
            if self.pub_in_topic:
                for i in range(int(self.time_delay)+1):
                    self.pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))
                    rospy.sleep(1)
            if self.pub_in_console:
                cv2.imshow(self.name_window, img)
                cv2.waitKey(0)
    def start(self):
        images = self.search_images(self.in_path)
        if self.pub_in_topic:
            rospy.init_node('slaid_show_one_time')
            self.pub = rospy.Publisher("pictures_from_{}".format(self.in_path.split("/")[-1]), Image, queue_size=1000)
        for name in images:
            try:
                img = cv2.imread(self.in_path+"/"+name)
                print("Took picture from file {}".format(name))
                imgg = self.pictures_naming(img, name)
                self.publish(imgg)
            except Exception as e: print(e)
def slaid_work_server(req):
    try:
        if req.action == "calibrate":
            # Start work with airspy-sdr
            sdrr = OSMO_SDR(SDR_CONFIGS)
            # Configurate/calibrate sdr by name of satellite
            if req.satellite != '': 
                if sdrr.load_config(req.satellite): sdrr.calibrate()
            else: 
                if sdrr.load_config(satellite): sdrr.calibrate()
            return srv.sdr_recorder_rosResponse(process = "calibrate")
        elif req.action == "start":
            try:
                if sdrr == None:
                    # Start work with airspy-sdr
                    sdrr = OSMO_SDR(SDR_CONFIGS)
                    # Configurate/calibrate sdr by name of satellite
                    if req.satellite != '': 
                        if sdrr.load_config(req.satellite): sdrr.calibrate()
                    else: 
                        if sdrr.load_config(satellite): sdrr.calibrate()
                    time.sleep(1)
            except: 
                # Start work with airspy-sdr
                sdrr = OSMO_SDR(SDR_CONFIGS)
                # Configurate/calibrate sdr by name of satellite
                if req.satellite != '': 
                    if sdrr.load_config(req.satellite): sdrr.calibrate()
                else: 
                    if sdrr.load_config(satellite): sdrr.calibrate()
                time.sleep(1)
            # Generate file name of path recording
            fileName = "{0}_{1:m%m_day%d_h%H_min%M_}".format(sdrr.config_name.replace(" ", "_"), datetime.utcnow())
            # Create path for all signals, if we don't have
            if req.path != '': 
                if not os.path.exists(req.path): os.makedirs(req.path) 
                # Create path for now signal
                os.mkdir("{0}/{1}".format(req.path,fileName))
                # Generate file name of file recording
                fileName = "{0}/{1}/{2}".format(req.path, fileName, fileName)
            else: 
                if not os.path.exists(path): os.makedirs(path) 
                # Create path for now signal
                os.mkdir("{0}/{1}".format(path,fileName))
                # Generate file name of file recording
                fileName = "{0}/{1}/{2}".format(path, fileName, fileName)
            # Start recording signal
            sdrr.start("{0}.iq".format(fileName),"{0}.log".format(fileName),"")
            # Wait untill we see satellite
            if req.time_recording != 0: time.sleep(req.time_recording)
            else: time.sleep(time_recording)
            # Stop recording, end of all process
            sdrr.stop()
            return srv.sdr_recorder_rosResponse(process = "start")
        elif req.action == "kill" or req.action == "stop" or req.action == "exit":
            sdrr = None
            exit()
        else: return srv.sdr_recorder_rosResponse(process = "error")
    except:   return srv.sdr_recorder_rosResponse(process = "error")

if __name__ == '__main__':
    global input_path, time_delay, path
    opts = docopt(USAGE)
    input_path = opts['--in']
    time_delay = float(opts['--t'])
    path = opts['--p']
    #OUTPUT_TOPIC_FLAG = bool(opts['--out_topic'])
    #OUTPUT_CON_FLAG = bool(opts['--out_con'])

    if bool(opts['--auto']):
        rospy.init_node('slaid_show_server')
        s = rospy.Service('slaid_show_ros', srv.slaid_show_ros, slaid_work_server) #////////////////////////////////////////////
        print("Ready slaid show")
        rospy.spin()
    elif bool(opts['--last']):
        # Start work with airspy-sdr
        work = Slaid_show()
        work.in_path = input_path
        work.time_delay = time_delay
        work.start()
        #print("Bye bye")
    else:
        work = Slaid_show()
        work.in_path = input_path
        work.time_delay = time_delay
        print(work.search_last_created_object("/home/clover/Desktop"))
        d = work.search_max_memory_object("/home/clover/Desktop/Metop")
        work.pub_in_console = True
        work.pub_in_topic = False
        g = work.search_max_memory_object(d)
        work.publish(g)
        print(g)
        #work.pub_in_console = True
        #work.pub_in_topic = False
        #work.start()
        print("Bye bye")
