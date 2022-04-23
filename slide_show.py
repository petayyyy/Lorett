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
  rosrun lorett slaid_show.py --s 'Noaa 19' --t 100
  rosrun lorett slaid_show.py -a | --auto
  rosrun lorett slaid_show.py -h | --help
  ***Don't forget about " or ' then you have space in naming. Also you can see example, use --h or -help then you start program.

Usage:
  slaid_show.py [--in='<name>'] [--t=<sec>] [--out_topic=<bool>] [--out_con=<bool>]  
  slaid_show.py [--in='<name>'] [--t=<sec>] [--out_topic=<bool>] [--out_con=<bool>] -l | --last
  slaid_show.py [--in='<name>'] [--t=<sec>] [--out_topic=<bool>] [--out_con=<bool>] -m | --max
  slaid_show.py -h | --help
  slaid_show.py -a | --auto

Options:
  -h, --help             Show correct format parameters.
  -a, --auto             Flag working in autonov secions.
  -l, --last             Show latest create path or file in directory.
  -m, --max              Show max memory path or file in directory.
  --in='<name>'          Name of file or path, that you need to see [default: /home/clover/tracks].
  --t=<sec>              Time see one pictures, in seconds [default: 1].
  --out_topic=<bool>     Flag publish pictures in topic [default: True].
  --out_con=<bool>       Flag printing console useless data [default: False].
'''
class Slaid_show:
    def __init__(self, in_path = "/home/pi/tracks", size_pictures_x = 1900, size_pictures_y = 800, time_delay = 2, is_node = True, pub_in_topic = True, pub_in_console = False, is_resize_pictures = True, is_background = True):
        self.is_background = is_background
        self.is_resize_pictures = is_resize_pictures
        self.size_pictures_x = size_pictures_x
        self.size_pictures_y = size_pictures_y
        self.time_delay = time_delay
        self.in_path = in_path
        self.pub_in_topic = pub_in_topic
        self.pub_in_console = pub_in_console
        self.color_background = (255, 255, 255)
        self.color_text = (0, 0, 0)
        self.name_window = "Images from satellite"
        self.bridge = CvBridge()
        if is_node:
            rospy.init_node('slaid_show_one_time')
        if self.pub_in_topic:
            self.pub = rospy.Publisher("pictures_from_{}".format(self.in_path.split("/")[-1]), Image, queue_size=1000)
    def search_images(self, path):
        images, flag = [], False
        if os.path.exists(path):
            for file in os.listdir(path):
                if os.path.isdir(path+"/"+file):
                    for pictures in os.listdir(path+"/"+file):
                        if ".png" in pictures or ".jpg" in pictures:
                            images.append(file+"/"+ str(pictures))
                            flag = True  
            if flag == False:
                print("Yours path is empty")
            return images
    def search_max_memory_object(self, path, is_problem = True, is_dir = True, is_file = True):
        max_memory = 0
        name = 'error'
        if os.path.exists(path):
            for file in os.listdir(path):
                if os.path.isdir(path+"/"+file) and (is_problem or is_dir):
                    memory = os.stat(path+"/"+file).st_size
                    if max_memory < memory:
                        max_memory = memory
                        name = path+"/"+file
                elif is_problem or is_file:
                    memory = os.stat(path+"/"+file).st_size
                    if max_memory < memory and (".png" in file or ".jpg" in file):
                        max_memory = memory
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
        for name in images:
            try:
                img = cv2.imread(self.in_path+"/"+name)
                print("Took picture from file {}".format(name))
                imgg = self.pictures_naming(img, name)
                self.publish(imgg)
            except Exception as e: print(e)
def slaid_work_server(req):
    try:
        if req.action == "start":
            work = Slaid_show(is_node=False)
            work.in_path = req.input_path
            work.time_delay = req.time_delay
            work.start()
            work = None
            return srv.sdr_recorder_rosResponse(process = "start")
        elif req.action == "start_max_memory":
            # find max memory path or file in correct directory
            work = Slaid_show(is_node=False)
            work.time_delay = req.time_delay
            work.pub_in_console = True
            work.in_path = work.search_max_memory_object(path = req.input_path)
            if os.path.isdir(work.in_path):
                work.start()
            else:
                work.publish(work.in_path)
            work = None
            return srv.sdr_recorder_rosResponse(process = "start_max_memory")
        elif req.action == "start_last_creating":
            work = Slaid_show(is_node=False)
            work.time_delay = req.time_delay
            work.pub_in_console = True
            work.in_path = work.search_last_created_object(path = req.input_path)
            if os.path.isdir(work.in_path):
                work.start()
            else:
                work.publish(work.in_path)
            work = None
            return srv.sdr_recorder_rosResponse(process = "start_last_creating")
        elif req.action == "kill" or req.action == "stop" or req.action == "exit":
            exit()
        else: return srv.sdr_recorder_rosResponse(process = "error")
    except:   return srv.sdr_recorder_rosResponse(process = "error")
if __name__ == '__main__':
    global input_path, time_delay, path
    opts = docopt(USAGE)
    input_path = opts['--in']
    time_delay = float(opts['--t'])
    pub_in_topic = opts['--out_topic']
    pub_in_console = opts['--out_con']

    if bool(opts['--auto']):
        rospy.init_node('slaid_show_server')
        s = rospy.Service('Slaid_show_ros', srv.Slaid_show, slaid_work_server) #////////////////////////////////////////////
        print("Ready slaid show")
        rospy.spin()
    else:
        work = Slaid_show(pub_in_topic = pub_in_topic, pub_in_console = pub_in_console, is_node=not bool(opts['--auto']))
        if bool(opts['--last']):
            # find last creating path or file in correct directory
            work.in_path = work.search_last_created_object(path = input_path)
            if os.path.isdir(work.in_path):
                work.start()
            else:
                work.publish(work.in_path)
        elif bool(opts['--max']):
            # find max memory path or file in correct directory
            work.in_path = work.search_max_memory_object(path = input_path)
            if os.path.isdir(work.in_path):
                work.start()
            else:
                work.publish(work.in_path)
        else:
            #work = Slaid_show(in_path = input_path, time_delay = time_delay, pub_in_topic = pub_in_topic, pub_in_console = pub_in_console)
            work.start()
