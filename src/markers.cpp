#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

// To be able to use cout
#include <iostream>
#include <iomanip>
using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "markers");
    ros::NodeHandle n;
    printf("READY to get image\n");

    image_transport::ImageTransport it(n);

    cv::namedWindow("view", CV_WINDOW_KEEPRATIO);    
    cv::startWindowThread();

  	cout << "OpenCV version: "
			<< CV_MAJOR_VERSION << "." 
			<< CV_MINOR_VERSION << "."
			<< CV_SUBMINOR_VERSION
			<< endl;

    ros::spin();

    cv::destroyWindow("view");
    return 0;
}
