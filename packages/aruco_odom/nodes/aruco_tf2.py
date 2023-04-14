#!/usr/bin/env python3 
import rospy

import tf2_ros

from nav_msgs.msg import Odometry
import geometry_msgs.msg

import tf_conversions


import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np








def image_callback(data):
	cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
	
	gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	cv2.aruco_dict = arucoDict

	corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray_img, cv2.aruco_dict,parameters=arucoParams)
	pose_centry = [0,0,0]

	rot_centry = [0,0,0]
	
	
	if len(corners) > 0:
		counter = len(ids)
		for i in range(0, len(ids)):
			rvec, tvec, objPoint = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.25, intrinsic_camera, distortion)

			cv2.aruco.drawDetectedMarkers(cv_image, corners)
			#print(tvec)

			g = geto(ids[i], rospy.get_param('~map_file'))

			#print(rvec)
            
			if(g != None):
				pose_centry[0] += tvec[0][0][0] + g[3]
				pose_centry[1] += tvec[0][0][1] + g[2]
				pose_centry[2] += g[4]

				rot_centry[0] += rvec[0][0][0]
				rot_centry[1] += rvec[0][0][1]
				rot_centry[2] += rvec[0][0][2]
			else:
				counter -= 1
				continue
			#cv2.aruco.drawAxis(cv_image, intrinsic_camera, distortion, rvec, np.array((tvec[0][0][0] + g[3], tvec[0][0][1] + g[2], tvec[0][0][2])), 0.2)
		if(counter != 0):
			pose_centry[0] = pose_centry[0]/counter
			pose_centry[1] = pose_centry[1]/counter
			pose_centry[2] = pose_centry[2]/counter

			rot_centry[0] = rot_centry[0]/counter
			rot_centry[1] = rot_centry[1]/counter
			rot_centry[2] = rot_centry[2]/counter

			br = tf2_ros.TransformBroadcaster()
			t = geometry_msgs.msg.TransformStamped()

			t.header.stamp = rospy.Time.now()
			t.header.frame_id = "aruco_centry"
			t.child_frame_id = "camera" + numdrone
			t.transform.translation.x = pose_centry[0]
			t.transform.translation.y = pose_centry[1]
			t.transform.translation.z = 0.0


			q = tf_conversions.transformations.quaternion_from_euler(rot_centry[0], rot_centry[1], rot_centry[2])


			t.transform.rotation.x = q[0]
			t.transform.rotation.y = q[1]
			t.transform.rotation.z = q[2]
			t.transform.rotation.w = q[3]

			br.sendTransform(t)

			cv2.aruco.drawAxis(cv_image, intrinsic_camera, distortion, rvec, np.array((pose_centry[0], pose_centry[1], tvec[0][0][2])), 0.2)
	image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

def geto(id, file):
    f = open(file, 'r')
    for i in f.readlines():
        if(i[0] == '#'):
            continue
        g = list(map(float, i.split("\t")))
        if(id == int(g[0])):
            f.close()
            return g
    

if __name__ == '__main__':
	rospy.init_node('aruco_tf2')
	numdrone = rospy.get_param('~num')
	imagetopic = '/camera/fisheye%s/image_raw' % numdrone
	bridge = CvBridge()
	
	image_pub = rospy.Publisher('~debug_camera_' + numdrone, Image, queue_size=10)

	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
	arucoParams = cv2.aruco.DetectorParameters_create()

	intrinsic_camera = np.array(((float(rospy.get_param('~fx')), 0, float(rospy.get_param('~ppx'))),(0, float(rospy.get_param('~fy')), float(rospy.get_param('~ppy'))),(0,0,1)))
	distortion = np.array((float(rospy.get_param('~c1')),float(rospy.get_param('~c2')),float(rospy.get_param('~c3')),float(rospy.get_param('~c4')), 0))

	image_sub = rospy.Subscriber(imagetopic, Image, image_callback)


	rospy.spin()
