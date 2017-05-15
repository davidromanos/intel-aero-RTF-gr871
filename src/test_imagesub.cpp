/* http://answers.ros.org/question/143916/problem-in-displaying-an-image/ */ 

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        ROS_INFO("Hi");
        cv::imshow("view", cv_ptr->image);
        //cv::namedWindow("view",CV_WINDOW_AUTOSIZE);
	cv::waitKey(1); // this is necessary to show the image in the view from OpenCV 3
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;  

  cv::namedWindow("view", CV_WINDOW_KEEPRATIO);
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/test_image", 1, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
}
