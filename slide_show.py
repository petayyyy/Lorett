import cv2 
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()

def pictures_named(img, name, color_text = (0, 0, 0), color_background = (255, 255, 255), size_x = 1900, size_y = 800, time_delta = 2, put_in_topic = True, resize_pictures = True, background = True):
    x, y = 23*len(name), 35
    if resize_pictures: img = cv2.resize(img, (size_x, size_y))
    if background:  cv2.rectangle(img,(0,0),(x,y),color_background,-1)
    
    cv2.line(img, (0,y), (x,y), color_text, 2) # Line left to right
    cv2.line(img, (x,0), (x,y), color_text, 2) # Line up to down
    cv2.putText(img, name, (10, y-10), cv2.FONT_HERSHEY_TRIPLEX, 1, color_text)
    if put_in_topic:
        pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
        rospy.sleep(time_delta)
    else:
        cv2.imshow('img', img)
        cv2.waitKey(0)
def search_images(path= "/home/clover/Desktop/1111"):
    images, flag = [], False
    if os.path.exists(path):
        for file in os.listdir(path):
            print(file)
            if os.path.isdir(path+"/"+file):
                for pictures in os.listdir(path+"/"+file):
                    if ".png" in pictures or ".jpg" in pictures:
                        images.append(file+"/"+ str(pictures))
                        flag = True            
        if flag == False:
            print("yours path is empty")
        return images

def start_show(path = "/home/clover/Desktop/1111", pub_in_topic = True):
    global pub, rate
    images = search_images(path)
    if pub_in_topic:
        rospy.init_node('flight')
        pub = rospy.Publisher("pictures_from_{}".format(path.split("/")[-1]), Image, queue_size=1000)
    for name in images:
        try:
            img = cv2.imread(path+"/"+name)
            print("Took picture from file {}".format(name))
            pictures_named(img, name)
        except Exception as e: print(e)
start_show()
