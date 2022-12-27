#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "lorett_c4s_telemetry");
    ros::NodeHandle nh, nh_priv("~");

    std::string frame_id = "map";
    std::string child_frame_id = "base_link";

    ros::Publisher telemetry_pub = nh.advertise<geometry_msgs::PoseStamped>("/lorett/telemetry", 30);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped pose_msg;
    ros::Rate rate_telem(30);

    while (nh.ok()) {
        try {
            listener.lookupTransform("/" + frame_id, "/" + child_frame_id, ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
        pose_msg.header.frame_id = frame_id;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = transform.getOrigin().x();
        pose_msg.pose.position.y = transform.getOrigin().y();
        pose_msg.pose.position.z = transform.getOrigin().z();
        pose_msg.pose.orientation.x = transform.getRotation().getX();
        pose_msg.pose.orientation.y = transform.getRotation().getY();
        pose_msg.pose.orientation.z = transform.getRotation().getZ();
        pose_msg.pose.orientation.w = transform.getRotation().getW();
        telemetry_pub.publish(pose_msg);
        rate_telem.sleep();
        ros::spinOnce();
    }
}