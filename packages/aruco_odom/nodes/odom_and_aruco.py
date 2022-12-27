#!/usr/bin/env python3 
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg

from tf2_msgs.msg import TFMessage


camera1_offset_x = None
camera2_offset_x = None
camera1_offset_y = None
camera2_offset_y = None
camera1_offset_z = None
camera2_offset_z = None

def handle_body_pose(msg):

    if(camera1_offset_x != None and camera1_offset_y  != None and camera1_offset_z != None and camera2_offset_x != None and camera2_offset_y != None and camera2_offset_z != None):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom_map"
        t.child_frame_id = "aruco_odom_map"

        t.transform.translation.x = msg.pose.position.x - (camera1_offset_y + camera2_offset_y)/2
        t.transform.translation.y = msg.pose.position.y - (camera1_offset_x + camera2_offset_x)/2
        t.transform.translation.z = (camera1_offset_z + camera2_offset_z)/2

        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        br.sendTransform(t)

def handle_odom_pose(msg):
        
    if(msg.transforms[0].child_frame_id == "camera1"):
        global camera1_offset_x, camera1_offset_y, camera1_offset_z
        camera1_offset_x = msg.transforms[0].transform.translation.x
        camera1_offset_y = msg.transforms[0].transform.translation.y
        camera1_offset_z = msg.transforms[0].transform.translation.z
    elif(msg.transforms[0].child_frame_id == "camera2"):
        global camera2_offset_x, camera2_offset_y, camera2_offset_z
        camera2_offset_x = msg.transforms[0].transform.translation.x
        camera2_offset_y = msg.transforms[0].transform.translation.y
        camera2_offset_z = msg.transforms[0].transform.translation.z


if __name__ == '__main__':
    rospy.init_node('odom_ar')
    odom_sub = rospy.Subscriber('/tf', TFMessage, handle_odom_pose)
    rospy.Subscriber('/mavros/local_position/pose', geometry_msgs.msg.PoseStamped, handle_body_pose)
    rospy.spin()
