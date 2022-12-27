#!/usr/bin/env python3 
import rospy

import tf_conversions

import tf2_web_republisher
import tf2_ros

from nav_msgs.msg import Odometry
import geometry_msgs.msg

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np

import open_map






def image_callback(data):
	cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
	
	gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	cv2.aruco_dict = arucoDict

	corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray_img, cv2.aruco_dict,parameters=arucoParams)
	pose_centry = [0,0,0]
	
	if len(corners) > 0:
		for i in range(0, len(ids)):
			rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.25, intrinsic_camera, distortion)

			cv2.aruco.drawDetectedMarkers(cv_image, corners)
			#print(tvec)
			g = open_map.geto(ids[i], rospy.get_param('~map_file'))

			pose_centry[0] += tvec[0][0][0] + g[3]
			pose_centry[1] += tvec[0][0][1] + g[2]
			pose_centry[2] += g[4]
			#cv2.aruco.drawAxis(cv_image, intrinsic_camera, distortion, rvec, np.array((tvec[0][0][0] + g[3], tvec[0][0][1] + g[2], tvec[0][0][2])), 0.2)
		pose_centry[0] = pose_centry[0]/len(ids)
		pose_centry[1] = pose_centry[1]/len(ids)
		pose_centry[2] = pose_centry[2]/len(ids)

		br = tf2_ros.TransformBroadcaster()
		t = geometry_msgs.msg.TransformStamped()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "aruco_centry"
		t.child_frame_id = "camera" + numdrone
		t.transform.translation.x = pose_centry[0]
		t.transform.translation.y = pose_centry[1]
		t.transform.translation.z = 0.0

		t.transform.rotation.x = 0
		t.transform.rotation.y = 0
		t.transform.rotation.z = 0
		t.transform.rotation.w = 1

		br.sendTransform(t)

		cv2.aruco.drawAxis(cv_image, intrinsic_camera, distortion, rvec, np.array((pose_centry[0], pose_centry[1], tvec[0][0][2])), 0.2)
	image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))



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
