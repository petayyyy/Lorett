import os
import cv2
from cv2 import aruco
import yaml
import numpy
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rospy.init_node('picture_cap')

class ChAruco_calibration():
    def __yaml_save(self):
        w, h, = self.image_size
        rm_data = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        mat_data = []
        for i in self.cameraMatrix:
            for x in i: mat_data.append(x)
        mat_data = list(map(float, mat_data))

        dst_data = list(map(float, self.distCoeffs[0]))

        pm_data = []
        for i in self.projection_matrix:
            for x in i: pm_data.append(x)
        pm_data = list(map(float, pm_data))

        data = {"image_width": w,
                "image_height": h,
                "distortion_model": "plumb_bob",
                "camera_name": "raspicam",
                "camera_matrix": {"rows": 3, "cols": 3, "data": mat_data},
                "distortion_coefficients": {"rows": 1, "cols": 8, "data": dst_data},
                "rectification_matrix": {"rows": 3, "cols": 3, "data": rm_data},
                "projection_matrix": {"rows": 3, "cols": 4, "data": pm_data}}
        file = open("2"+self.fname, "w")
        for key in data:
            #for key2 in key: file.write(yaml.dump({key2: key[key2]}))
            file.write(yaml.dump({key: data[key]}, default_flow_style=False))
    def save_to_file(self):
        with open(self.fname, "w") as f:
            f.write("image_width: 320\nimage_height: 240\ndistortion_model: plumb_bob\ncamera_name: main_camera_optical\ncamera_matrix:\n  rows: 3\n  cols: 3\n  data:\n")
            if self.print_yaml_on_screen: print("image_width: 320\nimage_height: 240\ndistortion_model: plumb_bob\ncamera_name: main_camera_optical\ncamera_matrix:\n  rows: 3\n  cols: 3\n  data:\n", end="")
            for i in self.cameraMatrix:
                for j in i:
                    f.write("  - {0}\n".format(j))
                    if self.print_yaml_on_screen: print("  - {0}\n".format(j), end="")
            f.write("distortion_coefficients:\n  rows: 1\n  cols: 8\n  data: ")
            if self.print_yaml_on_screen: print("distortion_coefficients:\n  rows: 1\n  cols: 8\n  data: ", end="")
            f.write("[ {0}".format(self.distCoeffs[0][0]))
            if self.print_yaml_on_screen: print("[ {0}".format(self.distCoeffs[0][0]), end="")
            for i in range(1,len(self.distCoeffs[0])):
                f.write(", {0}".format(self.distCoeffs[0][i]))
                if self.print_yaml_on_screen: print(", {0}".format(self.distCoeffs[0][i]), end="")
            f.write("]\n")
            if self.print_yaml_on_screen: print("]")
            f.write("rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n")
            if self.print_yaml_on_screen: print(f.write("rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"), end="")
            f.write("projection_matrix:\n  rows: 3\n  cols: 4\n  data:\n")
            if self.print_yaml_on_screen: print("projection_matrix:\n  rows: 3\n  cols: 4\n  data:\n", end="")
            for i in self.projection_matrix:
                for j in i:
                    f.write("  - {0}\n".format(j))
                    if self.print_yaml_on_screen: print("  - {0}\n".format(j))
        # Print to console our success
        print('Calibration successful. Calibration file used: {}'.format(self.fname))
    def Charuco_board_setting(self):
        if self.bit_aruco == 4: self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        elif self.bit_aruco == 5: self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        elif self.bit_aruco == 6: self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_1000)
        elif self.bit_aruco == 7: self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_7X7_1000)
        else: 
            print("Your Dictionary of aruco is incorrect")
            exit()
        self.CHARUCO_BOARD = aruco.CharucoBoard_create(squaresX=self.CHARUCOBOARD_COLCOUNT, squaresY=self.CHARUCOBOARD_ROWCOUNT, squareLength=self.squareLength, markerLength=self.markerLength, dictionary=self.ARUCO_DICT)
    def calculate_proj_mat(self):
        # Calculate projection matrix
        cam_mtx = numpy.zeros((3, 4), numpy.float64)
        cam_mtx[:, :-1] = self.cameraMatrix
        rmat = numpy.zeros((3, 3), numpy.float64)
        r_t_mat = numpy.zeros((3, 4), numpy.float64)
        cv2.Rodrigues(self.rvecs[0], rmat)
        r_t_mat = cv2.hconcat([rmat, self.tvecs[0]], r_t_mat)
        self.projection_matrix = (cam_mtx * r_t_mat)
    def search_images(self):
        self.images, flag = [], False
        directory = os.fsencode(self.path)        
        for file in os.listdir(directory):
            filename = os.fsdecode(file)
            if ".png" in filename or ".jpg" in filename:
                self.images.append(self.path +"/"+ str(filename))
                flag = True
        if flag == False:
            print("yours path is empty")
            exit()
    def calculate_parameters(self):
        if self.cv2show:
            # Destroy any open CV windows
            cv2.destroyAllWindows()
        # Make sure at least one image was found
        if self.work_with_files:
            if len(self.images) < 1:
                # Calibration failed because there were no images, warn the user
                print("Calibration was unsuccessful. No images of charucoboards were found. Add images of charucoboards and use or alter the naming conventions used in this file.")
                # Exit for failure
                exit()

        # Make sure we were able to calibrate on at least one charucoboard by checking
        # if we ever determined the image size
        if not self.image_size:
            # Calibration failed because we didn't see any charucoboards of the PatternSize used
            print("Calibration was unsuccessful. We couldn't detect charucoboards in any of the images supplied. Try changing the patternSize passed into Charucoboard_create(), or try different pictures of charucoboards.")
            # Exit for failure
            exit()

        # Now that we've seen all of our images, perform the camera calibration
        # based on the set of points we've discovered
        self.calibration, self.cameraMatrix, self.distCoeffs, self.rvecs, self.tvecs = aruco.calibrateCameraCharuco(
                charucoCorners=self.corners_all,
                charucoIds=self.ids_all,
                board=self.CHARUCO_BOARD,
                imageSize=self.image_size,
                cameraMatrix=None,
                distCoeffs=None)
        self.calculate_proj_mat()
    def find_charuco(self, img):
        # Grayscale the image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find aruco markers in the query image
        corners, ids, _ = aruco.detectMarkers(
                image=gray,
                dictionary=self.ARUCO_DICT)

        # Outline the aruco markers found in our query image
        img = aruco.drawDetectedMarkers(
                image=img, 
                corners=corners)

        # Get charuco corners and ids from detected aruco markers
        response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=self.CHARUCO_BOARD)
        # If a Charuco board was found, let's collect image/corner points
        # Requiring at least 20 squares
        if len(ids) >= (self.CHARUCOBOARD_COLCOUNT * self.CHARUCOBOARD_ROWCOUNT) // 2:
            # Add these corners and ids to our calibration arrays
            self.corners_all.append(charuco_corners)
            self.ids_all.append(charuco_ids)
            
            # Draw the Charuco board we've detected to show our calibrator the board was properly detected
            img = aruco.drawDetectedCornersCharuco(
                    image=img,
                    charucoCorners=charuco_corners)
        
            # If our image size is unknown, set it now
            if not self.image_size:
                self.image_size = gray.shape[::-1]
        
            # Reproportion the image, maxing width or height at 1000
            proportion = max(img.shape) / 1000.0
            img = cv2.resize(img, (int(img.shape[1]/proportion), int(img.shape[0]/proportion)))
            # Pause to display each image, waiting for key press
            if self.pub_in_topic != "":
                # Publish topic with detecting pictures
                self.pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))
                self.rate.sleep()
                a = input("To continue, press enter")
            if self.cv2show:
                cv2.imshow('Charuco board', img)
                cv2.waitKey(0)
        else:
            print("Not able to detect a charuco board in image")
    def __init__(self, work_with_files = False, path="/home/clover/Desktop/Charuco", fname="calibration.yaml", pub_in_topic = "", cv2show = False, print_yaml_on_screen = False):
        self.path = path
        self.work_with_files = work_with_files
        self.cv2show = cv2show
        self.fname = fname
        self.pub_in_topic = pub_in_topic
        self.cv2show = cv2show
        self.print_yaml_on_screen = print_yaml_on_screen

        if self.pub_in_topic != "" or self.work_with_files != True:
            self.bridge = CvBridge()
        if self.pub_in_topic != "":
            self.pub = rospy.Publisher(pub_in_topic, Image, queue_size=1000)
            self.rate = rospy.Rate(10)
        # Create the arrays and variables we'll use to store info like corners and IDs from images processed
        self.corners_all = [] # Corners discovered in all images processed
        self.ids_all = [] # Aruco ids corresponding to corners discovered
        self.image_size = None # Determined at runtime
        self.input_topic = "main_camera/image_raw"
        self.col_pictures = 25
        
        # Parameters ChAruco board
        self.CHARUCOBOARD_ROWCOUNT = 5 
        self.CHARUCOBOARD_COLCOUNT = 8
        self.bit_aruco = 4 
        self.squareLength=0.034 
        self.markerLength=0.027
    def start_calibration(self):
        self.Charuco_board_setting()
        if self.work_with_files:
            self.search_images()
            for iname in self.images:
                # Open the image
                try:
                    img = cv2.imread(iname)
                    print("Took picture from file {}".format(iname))
                    self.find_charuco(img)
                except Exception as e: print(e)
        else:
            for i in range(self.col_pictures):
                try:
                    img = self.bridge.imgmsg_to_cv2(rospy.wait_for_message(self.input_topic, Image), 'bgr8')
                    print("Took picture from topic {0} in {1}".format(i+1,self.col_pictures))
                    self.find_charuco(img)
                except Exception as e: print(e)
        print("Wait a little bit time")
        self.calculate_parameters()
        self.save_to_file()
        self.__yaml_save()

if __name__ == "__main__":
    detect = ChAruco_calibration(work_with_files=False, cv2show=False, pub_in_topic="Debug")
    detect.start_calibration()
