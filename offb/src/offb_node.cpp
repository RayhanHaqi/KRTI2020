/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Header.h>
#include <offb/kordinat.h>


std::map<char, std::vector<float>> gerak_posisi {
    {'w', {0, -0.5, 0}},
    {'a', {0.5, 0, 0}},
    {'s', {0, 0.5, 0}},
    {'d', {-0.5, 0, 0}},
    {'[', {0, 0, 0.5}},
    {']', {0, 0, -0.5}}
};

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

std_msgs::Header teleop_key;
void callback2(const std_msgs::Header::ConstPtr& msg){
    teleop_key = *msg;
}

offb::kordinat kordinat_objek;
void callback3(const offb::kordinat::ConstPtr& msg){
    kordinat_objek = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    float ketinggian_now;
    char c;
    double time_teleop, time_vision;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> 
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> //Publish posisi target
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist> //Publish kecepatan drone
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Subscriber current_pos = nh.subscribe<geometry_msgs::PoseStamped> //Subscribe posisi drone
            ("mavros/local_position/pose", 10, callback);
    ros::Subscriber teleop_sub = nh.subscribe<std_msgs::Header>
            ("teleop", 10, callback2);
    ros::Subscriber sub_kor = nh.subscribe<offb::kordinat>
            ("vision", 10, callback3);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(32.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 0;
    
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = 0;
    cmd_msg.linear.y = 0;
    cmd_msg.linear.z = 0;
    cmd_msg.angular.x = 0;
    cmd_msg.angular.y = 0;
    cmd_msg.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        cmd_pub.publish(cmd_msg);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    int qop;
    std::cout << "Masukin angka!";
    std::cin >> qop;

    while(ros::ok() && (current_state.mode != "OFFBOARD" || !current_state.armed)){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        ketinggian_now = current_position.pose.position.z;
        std::string s = std::to_string(ketinggian_now);
        std::string ss = teleop_key.frame_id;
        c = ss[0];
        time_teleop = teleop_key.stamp.toSec();
        time_vision = kordinat_objek.stamp.toSec();
        ROS_INFO("Ketinggian = %s Teleop Key = %s", s.c_str(), ss.c_str());
        double time_now = ros::Time::now().toSec();

        if (time_now - time_teleop < 0.2){
            if (gerak_posisi.count(c)){
                cmd_msg.linear.x = gerak_posisi[c][0];
                cmd_msg.linear.y = gerak_posisi[c][1];
                cmd_msg.linear.z = gerak_posisi[c][2];
                cmd_pub.publish(cmd_msg);

                pose.pose.position.x = current_position.pose.position.x;
                pose.pose.position.y = current_position.pose.position.y;
                pose.pose.position.z = current_position.pose.position.z;
            }
            else{
                cmd_msg.linear.x = 0;
                cmd_msg.linear.y = 0;
                cmd_msg.linear.z = 0;
                cmd_pub.publish(cmd_msg);
            }
        }
        else{
            cmd_msg.linear.x = 0;
            cmd_msg.linear.y = 0;
            cmd_msg.linear.z = 0;
            cmd_pub.publish(cmd_msg);
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("PILOOOOOOOOOOTTTTTTTTTTT");

    return 0;
}
