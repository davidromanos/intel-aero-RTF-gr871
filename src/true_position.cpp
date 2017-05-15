#include "ros/ros.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <random>

std::default_random_engine generator;
std::normal_distribution<double> distribution(0.0,0.01);


gazebo_msgs::ModelStates gazebo_model_state;
void callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    gazebo_model_state = *msg;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "true_position");
    ros::NodeHandle nh;

    ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::ModelStates>
            ("gazebo/model_states", 30, callback);

    ros::Publisher true_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/mocap/pose", 30);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>
            ("twist", 30);

  ros::Rate loop_rate(30);

  
int count = 1;
  while (ros::ok())
  {

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist twist;

    if (gazebo_model_state.pose.size() >= 3) 
    {
       pose.pose = gazebo_model_state.pose[2];
       twist = gazebo_model_state.twist[2];
    }
    
    pose.header.stamp=ros::Time::now();
    pose.header.seq=count;
    pose.header.frame_id=1;

    pose.pose.position.x = pose.pose.position.x + distribution(generator);
    pose.pose.position.y = pose.pose.position.y + distribution(generator);
    pose.pose.position.z = pose.pose.position.z + distribution(generator);

    true_pos_pub.publish(pose);
    twist_pub.publish(twist);
    ros::spinOnce();
    count++;
    loop_rate.sleep();
  
  }

  //ros::spin();
  return 0;
}
