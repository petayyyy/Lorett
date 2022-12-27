#include <iostream>
#include <string>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>


#include <lorett_c4s/takeoff.h>
#include <lorett_c4s/publishPose.h>
#include <lorett_c4s/yawPose.h>

const double PI = 3.141592653589793238463;

ros::ServiceClient arming, set_mode, cmd_command;
mavros_msgs::State state;

ros::Duration offboard_timeout;
ros::Duration arming_timeout;
mavros_msgs::StatusText statustext;

ros::Publisher position_pub;

mavros_msgs::PositionTarget position_raw_msg;
geometry_msgs::PoseStamped position_msg;

geometry_msgs::PoseStamped pose_msg;
geometry_msgs::PoseStamped telemetry_pose_msg;

ros::Publisher status_drone_pub, flag_startDetection_pub, flag_startRecognition_pub;
ros::Timer status_timer, telemetry_timer;
bool mavros_position_sub = false;
bool detect_human_sub = false;

bool arm = true, already = true, offbord = true;
bool telebetry_dont_call = false;

double x_tf = 0, y_tf = 0, z_tf = 0;

bool kill = false;

geometry_msgs::PoseStamped pose_yaw_msg;

bool offarm = true;

enum status_drone{
    READY_TO_FLY,
    WORKING,
    CALIBRATION,
    WAITING_FOR_TASK,
    NO_TASK = 10,
    NO_CAMERA,
    UNKNOWN_ERROR = 19
};

void offboardAndArm() {
    try{
        ros::Rate r(10);
        if (state.mode != "OFFBOARD") {
            auto start = ros::Time::now();
            ROS_INFO("switch to OFFBOARD");
            static mavros_msgs::SetMode sm;
            sm.request.custom_mode = "OFFBOARD";

            if (!set_mode.call(sm))
                throw std::runtime_error("Error calling set_mode service");

            // wait for OFFBOARD mode
            while (ros::ok()) {
                ros::spinOnce();
                if (state.mode == "OFFBOARD") {
                    break;
                } else if (ros::Time::now() - start > offboard_timeout) {
                    std::string report = "OFFBOARD timed out";
                    if (statustext.header.stamp > start)
                        report += ": " + statustext.text;
                    throw std::runtime_error(report);
                }
                ros::spinOnce();
                r.sleep();
            }
        }

        if (!state.armed) {
            ros::Time start = ros::Time::now();
            ROS_INFO("arming");
            mavros_msgs::CommandBool srv;
            srv.request.value = true;
            if (!arming.call(srv)) {
                throw std::runtime_error("Error calling arming service");
            }

            // wait until armed
            while (ros::ok()) {
                ros::spinOnce();
                if (state.armed) {
                    break;
                } else if (ros::Time::now() - start > arming_timeout) {
                    std::string report = "Arming timed out";
                    if (statustext.header.stamp > start)
                        report += ": " + statustext.text;
                    throw std::runtime_error(report);
                }
                ros::spinOnce();
                r.sleep();
            }
        }
    }
    catch (...){
        /* */
    }
}

bool navi(double x, double y, double z, std::string farme_id) {
    if (kill)    return false;
    ros::Time time_now = ros::Time::now();
    position_msg.header.stamp = time_now;
    position_msg.header.frame_id = farme_id;

    position_msg.pose.position.x = x;
    position_msg.pose.position.y = y;
    position_msg.pose.position.z = z;

    position_msg.pose.orientation.x = pose_msg.pose.orientation.x;
    position_msg.pose.orientation.y = pose_msg.pose.orientation.y;
    position_msg.pose.orientation.z = pose_msg.pose.orientation.z;
    position_msg.pose.orientation.w = pose_msg.pose.orientation.w;

    position_pub.publish(position_msg);
    if (offarm) {
        offboardAndArm();
        offarm = false;
    }
    return true;
}

bool yaw_pose(double yaw, std::string farme_id) {
    if (kill)    return false;
    ros::Time time_now = ros::Time::now();
    position_msg.header.stamp = time_now;
    position_msg.header.frame_id = farme_id;

    position_msg.pose.position.x = pose_yaw_msg.pose.position.x;
    position_msg.pose.position.y = pose_yaw_msg.pose.position.y;
    position_msg.pose.position.z = pose_yaw_msg.pose.position.z;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw);
    myQuaternion=myQuaternion.normalize();

    position_msg.pose.orientation.x = myQuaternion[0];
    position_msg.pose.orientation.y = myQuaternion[1];
    position_msg.pose.orientation.z = myQuaternion[2];
    position_msg.pose.orientation.w = myQuaternion[3];

    position_pub.publish(position_msg);
    if (offarm) {
        offboardAndArm();
        offarm = false;
    }
    return true;
}

bool land (std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    static mavros_msgs::SetMode sm;
    sm.request.custom_mode = "AUTO.LAND";
    set_mode.call(sm);
    while (!state.armed)    ros::spinOnce();
    offarm = true;
    res.success=true;
    ROS_INFO("land PERFORMED");

    std_msgs::Bool s_bool;
    s_bool.data = true;
    flag_startRecognition_pub.publish(s_bool);
    ros::Duration(1.0).sleep();
    s_bool.data = false;
    flag_startRecognition_pub.publish(s_bool);
    return true;
}

bool killSwitch (std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    kill = true;
    ros::Rate r(10);
    for(int i = 20; ros::ok() && i > 0; --i){ 
        mavros_msgs::CommandBool srv;
        srv.request.value = false;
        arming.call(srv);
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("killSwitch PERFORMED");
    return true;
}

bool takeOff (lorett_c4s::takeoff::Request& req, lorett_c4s::takeoff::Response& res) {
    ros::Rate rate(20.0);
    pose_msg = telemetry_pose_msg;
    bool telemetry_get = false;
    for(int i = 50; ros::ok() && i > 0; --i){
        if (i < 40 && !telemetry_get) { pose_msg = telemetry_pose_msg; telemetry_get = true;}

        if (telemetry_get)  navi(pose_msg.pose.position.x, pose_msg.pose.position.y, req.z, "body");
        else                navi(0, 0, req.z, "body");

        ros::spinOnce();
        rate.sleep();
        ROS_INFO("map z=%f  step=%d", req.z, i);
        // std::cout << "body z=" << req.z << "; step = " << i << std::endl;
    }
    for(int i = 100; ros::ok() && i > 0; --i){
        // navi(0, 0, req.z, "map");
        navi(pose_msg.pose.position.x, pose_msg.pose.position.y, req.z, "body");
        ros::spinOnce();
        rate.sleep();
        telebetry_dont_call = true;
        ROS_INFO("map z=%f  step=%d", req.z, i);
        // std::cout << "map z=" << req.z << "; step = " << i << std::endl;
    }
    
    res.s=true;
    ROS_INFO("takeOff PERFORMED");
    return true;
}


bool point_aruco (lorett_c4s::publishPose::Request& req, lorett_c4s::publishPose::Response& res) {
    try{
        ros::Rate rate(20.0);
        for(int i = 200; ros::ok() && i > 0; --i){
            navi(req.x, req.y, req.z, "aruco_odom_map");
            ros::spinOnce();
            rate.sleep();
        }
        res.s=true;
        ROS_INFO("aruco point PERFORMED");
    }
    catch (...){
        ROS_INFO("aruco have problem");
    }
    return true;
}


bool point (lorett_c4s::publishPose::Request& req, lorett_c4s::publishPose::Response& res) {
    ros::Rate rate(20.0);
    for(int i = 200; ros::ok() && i > 0; --i){
        navi(req.x, req.y, req.z, "map");
        ros::spinOnce();
        rate.sleep();
    }
    res.s=true;
    ROS_INFO("point PERFORMED");
    return true;
}


bool point_yaw (std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    pose_yaw_msg = telemetry_pose_msg;
    double yaw = tf::getYaw(pose_yaw_msg.pose.orientation);
    ros::Rate rate(4.0);

    std_msgs::Bool s;
    s.data = true;
    flag_startDetection_pub.publish(s);

    // for(int i = 200; ros::ok() && i > 0; --i){
    for (double i = 0; ros::ok() && i <= 2 * PI; i+= (PI / 40)) { 
        // if (detect_human_sub)   break;
        yaw_pose (yaw + i, "map");
        ROS_INFO("yaw=%f", yaw+i);
        // std::cout << "yaw = " << yaw + i << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    s.data = false;
    flag_startDetection_pub.publish(s);
    ROS_INFO("point_yaw PERFORMED");

    res.success = true;
    return true;
}

bool point_yaw_target (lorett_c4s::yawPose::Request& req, lorett_c4s::yawPose::Response& res) {
    pose_yaw_msg = telemetry_pose_msg;
    double yaw = tf::getYaw(pose_yaw_msg.pose.orientation);
    ros::Rate rate(20.0);    
    for(int i = 50; ros::ok() && i > 0; --i){
        yaw_pose (req.yaw, "map");
        ROS_INFO("yaw=%f", req.yaw);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("point_yaw_target PERFORMED");
    res.s = true;
    return true;
}

void handleTelemetry (const geometry_msgs::PoseStamped& Reached) {
    telemetry_pose_msg = Reached;
}

void handleState (const mavros_msgs::State& s) {
	state = s;
}

void handleStatusText (const mavros_msgs::StatusText& st) {
    statustext = st;
}

void handleFlagJob (const std_msgs::Bool& Reached) {
    mavros_position_sub = Reached.data; // true or fals. true when the drone has flown else false
}

bool calibr (std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    auto seq_last = statustext.header.seq;
    ros::Rate r(10);
    auto start = ros::Time::now();
    ROS_INFO("gyro calibration");
    static mavros_msgs::CommandLong cmd_long_gyro;
    std::string report_calibr_gyro = "";
    cmd_long_gyro.request.command = 241;
    cmd_long_gyro.request.param1 = 1;
    if (!cmd_command.call(cmd_long_gyro))
        throw std::runtime_error("Error calling cmd_command service");
    while (ros::ok()) {
        ros::spinOnce();
        // std::cout << statustext.header.seq << " - " << seq_last << std::endl;
        if (statustext.header.seq > seq_last){
            report_calibr_gyro = statustext.text;
            auto found = report_calibr_gyro.find("done") != std::string::npos;
            std::cout << "[INFO] " << report_calibr_gyro << std::endl;
            if (found)  break;
        }
        ros::spinOnce();
        r.sleep();
    }

    ros::Duration(5.0).sleep();

    seq_last = statustext.header.seq;
    ROS_INFO("level calibration");
    static mavros_msgs::CommandLong cmd_long_level;
    std::string report_calibr_level = "";
    cmd_long_level.request.command = 241;
    cmd_long_level.request.param5 = 2;
    if (!cmd_command.call(cmd_long_level))
        throw std::runtime_error("Error calling cmd_command service");

    while (ros::ok()) {
        ros::spinOnce();
        // std::cout << statustext.header.seq << " - " << seq_last << std::endl;
        if (statustext.header.seq > seq_last){
            report_calibr_level = statustext.text;
            auto found = report_calibr_level.find("done") != std::string::npos;
            std::cout << "[INFO] " << report_calibr_level << std::endl;
            if (found)  break;
        }
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("calibr PERFORMED");

    res.success = true;
    return true;
}

bool reboot_fcu (std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

    auto seq_last = state.header.seq;
    ros::Rate r(10);
    auto start = ros::Time::now();
    ROS_INFO("reboot fcu");
    static mavros_msgs::CommandLong cmd_long_fcu;
    std::string report_fcu = "";
    cmd_long_fcu.request.command = 246;
    cmd_long_fcu.request.param1 = 1;
    if (!cmd_command.call(cmd_long_fcu))
        throw std::runtime_error("Error calling cmd_command service");
        
    while (ros::ok()) {
        ros::spinOnce();
        if (state.header.seq > seq_last)    break;
        
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("reboot_fcu PERFORMED");

    res.success = true;
    return true;
}

void publishStatus (const ros::TimerEvent& e) {
    status_drone s;
    if (mavros_position_sub)   s = WORKING;
    else {
        s = READY_TO_FLY;
    }

    std_msgs::Int8 res;
    res.data = s;
    status_drone_pub.publish(res);
    
    // std_msgs::Bool s_bool;
    // s_bool.data = true;
    // if (state.mode == "AUTO.LAND" && !state.armed)  flag_startRecognition_pub.publish(s_bool);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "lorett_c4s_node");
    ros::NodeHandle nh, nh_priv("~");

    offboard_timeout = ros::Duration(3.0);
    arming_timeout = ros::Duration(4.0);

    arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    cmd_command = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    status_drone_pub = nh.advertise<std_msgs::Int8>("/lorett/status", 1);
    
    auto ld_serv = nh.advertiseService("lorett/land", &land);
    auto takeOff_serv = nh.advertiseService("lorett/takeOff", &takeOff);
    auto point_serv = nh.advertiseService("lorett/point", &point);
    auto point_aruco_serv = nh.advertiseService("lorett/point_aruco", &point_aruco);
    auto killSwitch_serv = nh.advertiseService("lorett/killSwitch", &killSwitch);
    auto calibr_serv = nh.advertiseService("lorett/calibr", &calibr);
    auto reboot_fcu_serv = nh.advertiseService("lorett/reboot/fcu", &reboot_fcu);
    auto point_yaw_target_serv = nh.advertiseService("lorett/point_yaw_target", &point_yaw_target);

    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 1, handleState);
    ros::Subscriber statustext_sub = nh.subscribe("/mavros/statustext/recv", 1, handleStatusText);
    ros::Subscriber telemetry_sub = nh.subscribe("/lorett/telemetry", 30, handleTelemetry);
    
    status_timer = nh.createTimer(ros::Duration(0.01), &publishStatus);

    ROS_INFO("ready to fly");
    ros::spin();
    return 0;
}
