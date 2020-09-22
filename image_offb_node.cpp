/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Header.h>

class PID{
    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
    public:
        PID(double dt, double max, double min, double Kp, double Kd, double Ki);
        double calculate(double setpoint, double pv);
};

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki){
    _dt = dt;
    _max = max;
    _min = min;
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    _pre_error = 0;
    _integral = 0;

}

double PID::calculate(double setpoint, double pv){
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_offb_node");
    ros::NodeHandle nh;
    double inc;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist> //Publish kecepatan drone
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Subscriber pose_now = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, callback);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 3;

    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = 0;
    cmd_msg.linear.y = 0;
    cmd_msg.linear.z = 0;
    cmd_msg.angular.x = 0;
    cmd_msg.angular.y = 0;
    cmd_msg.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    PID mypid(0.1, 2, -0.5, 0.1, 0.01, 0.5);
    double val = 20;

    while(ros::ok()){
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
        

        float ketinggian_now = current_position.pose.position.z;
        pose.pose.position.x = current_position.pose.position.x;
        pose.pose.position.y = current_position.pose.position.y;

        cmd_msg.linear.x = 0;
        cmd_msg.linear.y = 0;
        cmd_msg.linear.z = 0;
        float takeoff = cmd_msg.linear.z;

        if (val < 0.1 && val > -0.1){    
            ROS_INFO("SIPPPPPPP");
        }
        else{
            inc = mypid.calculate(0, val);
            val += inc;
            ROS_INFO("val = %f inc = %f", val, inc);
        }
        if(ketinggian_now < 1){
            cmd_msg.linear.x = 0;
            cmd_msg.linear.z = 1;
            cmd_pub.publish(cmd_msg);

            // local_pos_pub.publish(pose);
            // ROS_INFO("Ketinggian berkurang : %.2f",current_position.pose.position.z);
        }
        else if(ketinggian_now < 1.9){
            cmd_msg.linear.x = 0;
            cmd_msg.linear.z = 0.2;
            cmd_pub.publish(cmd_msg);

            // local_pos_pub.publish(pose);
            // ROS_INFO("Ketinggian berkurang : %.2f",current_position.pose.position.z);
        }
        else{
            cmd_pub.publish(cmd_msg);
            // ROS_INFO("Gerak...");
        }       
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}