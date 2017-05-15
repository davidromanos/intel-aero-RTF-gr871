/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include "utils.h"

using namespace std;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped position;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

tf::Quaternion q1;

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    position = *msg;
}



int main(int argc, char **argv)
{    

    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/mocap/pose", 10, pos_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);


    ros::ServiceClient setHome_client = nh.serviceClient<mavros_msgs::CommandHome>
            ("mavros/cmd/arming");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    ros::Rate rate(20.0);
    int k = 0;
    int i = 0;
    double yawSetpoint = 0;
    vector<double> setpoints;
    /*setpoints.push_back(M_PI*0.1);
    setpoints.push_back(M_PI*0.3);
    setpoints.push_back(M_PI*0.5);
    setpoints.push_back(M_PI*0.7);
    setpoints.push_back(M_PI*0.9);*/
    setpoints.push_back(0.6*0.2);
    setpoints.push_back(0.6*0.4);
    setpoints.push_back(0.6*0.6);
    setpoints.push_back(0.6*0.8);
    setpoints.push_back(0.6);




    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = 2;
    pose.pose.position.y = 2;
    pose.pose.position.z = 1;
    pose.pose.orientation.w = 0;
    pose.pose.orientation.z = 1;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.x = 0;


    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandHome home_cmd;
    home_cmd.request.current_gps = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_request1 = ros::Time::now();




    while(ros::ok()){
        /*if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enableddd");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }*/

        if(current_state.mode == "OFFBOARD")
        {
            if(k%600 < 150)
            {
                yawSetpoint = setpoints[i];
            }
            else if(k%600 < 300)
            {
                yawSetpoint = 0.0;
            }
            else if(k%600 < 450)
            {
                yawSetpoint = -1*setpoints[i];
            }
            else
            {
                yawSetpoint = 0.0;
            }
            if(k%600 == 0)
            {
                i++;
                std::cout << "Progress: " <<100*i/setpoints.size() << "%\n";
            }
            if(i>setpoints.size()-1)
            {
                i=0;
            }
            k++;
        }
        else
        {
            k = 0;
            i = 0;
            pose.pose.position.x = position.pose.position.x;
            pose.pose.position.y = position.pose.position.y;
            pose.pose.position.z = position.pose.position.z;
        }

        q1.setW(position.pose.orientation.w);
        q1.setX(position.pose.orientation.x);
        q1.setY(position.pose.orientation.y);
        q1.setZ(position.pose.orientation.z);

        tf::Matrix3x3 m(q1);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        q1.setRPY(0,0,yawSetpoint);



        pose.pose.orientation.x = q1.getX();
        pose.pose.orientation.y = q1.getY();
        pose.pose.orientation.z = q1.getZ();
        pose.pose.orientation.w = q1.getW();


        local_pos_pub.publish(pose);
        //logToFile("/home/chris/Dropbox/P8 (CA2)/Controller/logs/yawlog.txt","%f,%f",yawSetpoint,yaw);
        last_request1 = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

