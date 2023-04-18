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








def image_callback(data, intrinsic_camera, distortion, bridge, image_pub):
	cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
	
	gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

	corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray_img, cv2.aruco_dict,parameters=arucoParams)
	pose_centry = [0,0,0]

	rot_centry = [0,0,0]
	
	
	if len(corners) > 0:
		counter = len(ids)
		for i in range(0, len(ids)):
			rvec, tvec, objPoint = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.25, intrinsic_camera, distortion)

			cv2.aruco.drawDetectedMarkers(cv_image, corners)
			
			g = all_aruco[str(ids[i])]

            
			if(g != None):
				pose_centry[0] += tvec[0][0][0] + g[2]
				pose_centry[1] += tvec[0][0][1] + g[1]
				pose_centry[2] += g[3]

				rot_centry[0] += rvec[0][0][0] + g[4]
				rot_centry[1] += rvec[0][0][1] + g[5]
				rot_centry[2] += rvec[0][0][2] + g[6]
			else:
				counter -= 1
				continue
		if(counter != 0):
			pose_centry[0] = pose_centry[0]/counter
			pose_centry[1] = pose_centry[1]/counter
			pose_centry[2] = pose_centry[2]/counter

			rot_centry[0] = rot_centry[0]/counter
			rot_centry[1] = rot_centry[1]/counter
			rot_centry[2] = rot_centry[2]/counter
	return pose_centry, rot_centry


    

if __name__ == '__main__':
	rospy.init_node('aruco_tf2')

	arucoParams = cv2.aruco.DetectorParameters_create()
	cv2.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
	all_aruco = {}
	f = open(rospy.get_param('~map_file'), 'r')
	for i in f.readlines():
		if(i[0] == '#'):
			continue
		u = list(map(float, i.split("\t")))
		
		all_aruco[str(u[0])] = all_aruco[1:]
	f.close()

	bridge_1 = CvBridge()
	image_pub_1 = rospy.Publisher('~debug_camera_1', Image, queue_size=10)
	
	image_camera_1 = rospy.wait_for_message("/camera/fisheye1/image_raw", Image)
	pos_camera_1, rot_camera_1 = image_callback(image_camera_1, np.array(((float(rospy.get_param('~fx_1')), 0, float(rospy.get_param('~ppx_1'))),(0, float(rospy.get_param('~fy_1')), float(rospy.get_param('~ppy_1'))),(0,0,1))), np.array((float(rospy.get_param('~c1_1')),float(rospy.get_param('~c2_1')),float(rospy.get_param('~c3_1')),float(rospy.get_param('~c4_1')), 0)), bridge_1, image_pub_1)



	bridge_2 = CvBridge()
	image_pub_2 = rospy.Publisher('~debug_camera_2', Image, queue_size=10)
	
	image_camera_2 = rospy.wait_for_message("/camera/fisheye2/image_raw", Image)
	pos_camera_2, rot_camera_2 = image_callback(image_camera_1, np.array(((float(rospy.get_param('~fx_2')), 0, float(rospy.get_param('~ppx_2'))),(0, float(rospy.get_param('~fy_2')), float(rospy.get_param('~ppy_2'))),(0,0,1))), np.array((float(rospy.get_param('~c1_2')),float(rospy.get_param('~c2_2')),float(rospy.get_param('~c3_2')),float(rospy.get_param('~c4_2')), 0)), bridge_2, image_pub_2)

	pos = [0,0,0]
	pos[0] = (pos_camera_1[0] + pos_camera_2[0] )/2
	pos[1] = (pos_camera_1[1] + pos_camera_2[1] )/2
	pos[2] = (pos_camera_1[2] + pos_camera_2[2] )/2
	
	rot = [0,0,0]
	rot[0] = (rot_camera_1[0] + rot_camera_2[0] )/2
	rot[1] = (rot_camera_1[1] + rot_camera_2[1] )/2
	rot[2] = (rot_camera_1[2] + rot_camera_2[2] )/2

	msg = rospy.wait_for_message('/mavros/local_position/pose', geometry_msgs.msg.PoseStamped)
	
	
	
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()

	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "map"
	t.child_frame_id = "aruco_map"

	t.transform.translation.x = pos[0] + msg.pose.position.x 
	t.transform.translation.y = pos[1] + msg.pose.position.y
	t.transform.translation.z = pos[2] + msg.pose.position.z

	rot_mav = tf_conversions.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

	q = tf_conversions.transformations.quaternion_from_euler(rot[0] + rot_mav[0], rot[1] + rot_mav[1], rot[2] + rot_mav[2])        
	
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]
	br.sendTransform(t)
	rospy.spin()
