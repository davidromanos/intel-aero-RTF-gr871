#include "ros/ros.h"
#include "std_msgs/String.h"
#include "udp.h"
#include "vicon.h"
#include <intel_aero_rtf_gr871/vicon.h>		// package name / message name
#include "intel_aero_rtf_gr871/position.h"               // package name / service name
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <assert.h>
#include <iostream>

//Desired position
float dx=0; float dy=0; float dz=1; 
//Desired angle || DO NOT USE YET
float dtx=0; float dty=0; float dtz=0;
//Actual position
float x=5; float y=5; float z=5;
//Angles in Eular radian 
tf2Scalar pitch = 5; tf2Scalar roll = 5; tf2Scalar yaw = 5;

bool updatePosition(intel_aero_rtf_gr871::position::Request &req,
            intel_aero_rtf_gr871::position::Response &res){

	if(dz<0)dz=0; if(dz>2)dz=2; if(dx<-3)dx=-3; if(dx>2)dx=2; if(dy<-2)dy=-2; if(dy>2)dy=2;
	dx = req.x;
	dy = req.y;
 	dz = req.z;
	dtx = req.thetax;
	dty = req.thetay;
	dtz = req.thetaz;
	if(x>=dx-0.2 && x<=dx+0.2)
		if(y>=dy-0.2 && y<=dy+0.2)
			res.inPosition = true;
		else
			res.inPosition = false;
return true;
}


int main(int argc, char **argv){

  ros::init(argc,argv,"vicon_listen");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",10);


  ros::Rate loop_rate(200);

  udp_struct udpNavLog;
  udp_struct udpState;
  FRAME_DATA VICON_frame;
  PVA_DATA   pva;
  udpServer_Init(&udpState,7901,0);
  init_frame(&VICON_frame, 31); //Set 27 to packet length if you change it
  int count = 1;
  geometry_msgs::PoseStamped msg;
  geometry_msgs::PoseStamped position;  
  tf2::Quaternion quat;
  tf::Quaternion q1;

while(ros::ok()){
        get_vicon_packet(&VICON_frame, &udpState, &pva);
   
	msg.header.stamp=ros::Time::now();
	position.header.stamp = ros::Time::now();
	msg.header.seq=count;
	position.header.seq=count;
	msg.header.frame_id=1;
	position.header.frame_id=1;

        x = pva.position[0];
        y = pva.position[1];
        z = pva.position[2] + 0.4;
        //pitch = pva.orientation[0];
        //roll = pva.orientation[1];
        //yaw = pva.orientation[2];

        //quat.setEuler(roll,pitch,yaw);

        //std::cout << "  x:  "  << pva.position[0] << "  y:  "  << pva.position[1] << "  z:  "  << pva.position[2] - 0.56 << "  p:  "  << pva.orientation[0] << "  r:  "  << pva.orientation[1]  << "  y:  "  << pva.orientation[2]    << "\n";

        msg.pose.position.x = x;
	msg.pose.position.y = y;
	msg.pose.position.z = z;
        //msg.pose.orientation.x = quat.x();
        //msg.pose.orientation.y = quat.y();
        //msg.pose.orientation.z = quat.z();
        //msg.pose.orientation.w = quat.w();
        msg.pose.orientation.x = pva.orientation[0];
        msg.pose.orientation.y = pva.orientation[1];
        msg.pose.orientation.z = pva.orientation[2];
        msg.pose.orientation.w = pva.orientation[3];
        //tf::Matrix3x3 m(quat);
        //m.getRPY(roll,pitch,yaw);
        //std::cout << "  p:  "  << pitch << "  r:  "  << roll  << "  y:  "  << yaw    << "\n";

	position.pose.position.x = dx;
	position.pose.position.y = dy;
	position.pose.position.z = dz;
	position.pose.orientation.z = 0;

    if(x > 0.01 && y > 0.01)
    {
    	pub.publish(msg);
    }
    ros::spinOnce();
    count++;
    loop_rate.sleep();
  }

  return 0;
}
