#!/usr/bin/env python3 
import rospy

# Because of transformations

import tf2_ros
import geometry_msgs.msg

from tf2_msgs.msg import TFMessage


import tf_conversions


camera1_offset_x = None
camera2_offset_x = None
camera1_offset_y = None
camera2_offset_y = None
camera1_offset_z = None
camera2_offset_z = None

camera1_rotation_x = None
camera2_rotation_x = None
camera1_rotation_y = None
camera2_rotation_y = None
camera1_rotation_z = None
camera2_rotation_z = None


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

        rot_x = tf_conversions.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[0]
        rot_y = tf_conversions.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[1]
        rot_z = tf_conversions.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]


        q = tf_conversions.transformations.quaternion_from_euler(rot_x - (camera1_rotation_x + camera2_rotation_x)/2, rot_y - (camera1_rotation_y + camera2_rotation_y)/2, rot_z - (camera1_rotation_z + camera2_rotation_z)/2)        

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)


        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "aruco_centry"
        t.child_frame_id = "aruco_map"

        t.transform.translation.x = (camera1_offset_y + camera2_offset_y)/2
        t.transform.translation.y = (camera1_offset_x + camera2_offset_x)/2
        t.transform.translation.z = (camera1_offset_z + camera2_offset_z)/2

        rot_x = tf_conversions.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[0]
        rot_y = tf_conversions.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[1]
        rot_z = tf_conversions.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]


        q = tf_conversions.transformations.quaternion_from_euler((camera1_rotation_x + camera2_rotation_x)/2, (camera1_rotation_y + camera2_rotation_y)/2, (camera1_rotation_z + camera2_rotation_z)/2)        

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)



def handle_odom_pose(msg):
        
    if(msg.transforms[0].child_frame_id == "camera1"):
        global camera1_offset_x, camera1_offset_y, camera1_offset_z, camera1_rotation_x, camera1_rotation_y, camera1_rotation_z
        camera1_offset_x = msg.transforms[0].transform.translation.x
        camera1_offset_y = msg.transforms[0].transform.translation.y
        camera1_offset_z = msg.transforms[0].transform.translation.z

        rt = tf_conversions.transformations.euler_from_quaternion([msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w])

        camera1_rotation_x = rt[0]
        camera1_rotation_y = rt[1]
        camera1_rotation_z = rt[2]
        
    elif(msg.transforms[0].child_frame_id == "camera2"):
        global camera2_offset_x, camera2_offset_y, camera2_offset_z, camera2_rotation_x, camera2_rotation_y, camera2_rotation_z
        camera2_offset_x = msg.transforms[0].transform.translation.x
        camera2_offset_y = msg.transforms[0].transform.translation.y
        camera2_offset_z = msg.transforms[0].transform.translation.z

        rt = tf_conversions.transformations.euler_from_quaternion([msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w])

        camera2_rotation_x = rt[0]
        camera2_rotation_y = rt[1]
        camera2_rotation_z = rt[2]


if __name__ == '__main__':
    rospy.init_node('odom_ar')
    odom_sub = rospy.Subscriber('/tf', TFMessage, handle_odom_pose)
    rospy.Subscriber('/mavros/local_position/pose', geometry_msgs.msg.PoseStamped, handle_body_pose)
    rospy.spin()
