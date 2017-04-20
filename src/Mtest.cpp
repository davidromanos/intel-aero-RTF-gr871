#include <iostream>
//#include <Eigen/Core>
//#include <Eigen/LU>
#include <ros/ros.h>
//#include <ecl/linear_algebra/eigen.hpp>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//typedef Matrix<float, 6, 1> Vector6f;
//typedef Matrix<float, 6, 6> Matrix6f;
//typedef Matrix<float, 6, Dynamic> Matrix6kf;

/* ############################## Defines measurement class ##############################  */






int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mtest");
    ros::NodeHandle nh;

    Eigen::Matrix2f H;
    H << 1,3,
         2,3;

    /*LandmarkMeasurement z1(1);
	LandmarkMeasurement z2(2);
	LandmarkMeasurement z3(3);
	LandmarkMeasurement z4(4);

	
  	std::cout << "Hello World!" << std::endl;
	std::cout << z1.c << std::endl;
    std::cout << z3.c << std::endl;*/
    std::cout << H;
  	return 0;
}

