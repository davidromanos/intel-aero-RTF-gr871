/* http://answers.ros.org/question/143916/problem-in-displaying-an-image/ */ 

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

void ConfigureCamera(bool autoExposure);
void imageCallback(const sensor_msgs::ImageConstPtr& image);

image_transport::Publisher pub_;
cv::Mat element_erode;
cv::Mat element_dilate;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_image_publisher");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  pub_ = it.advertise("/test_image", 3);


  //cv::Mat element = cv::Mat::ones( 10, 10, CV_32F );

  cvNamedWindow ("trackbar", 2 );
  cvStartWindowThread();

  bool staticImage = false;

  std::string param;
  ros::param::get("~image", param); // use rosrun with the parameter "_image:='static'" or "_image:='dynamic'"
  if (param.find("dynamic") == std::string::npos) {
      param.assign("static");
      staticImage = true;
  } else { // Use dynamic image
      staticImage = false;
      ConfigureCamera(true); // use auto exposure
  }

  if (staticImage) {
      cv::Mat img = cv::imread("depth.png",CV_LOAD_IMAGE_COLOR);    // OBS. If relative path is used, it will be the ros workspace, see http://answers.ros.org/question/57075/how-to-save-file-to-the-current-directory-with-roslaunch/

      if (!img.data)        // Check for invalid input
      {
        ROS_ERROR("Could not open or find the image");
        return -1;
      }

      //cv::Mat element = cv::Mat::ones( 10, 10, CV_32F );
      int dilation_size = 3;
      element_dilate = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                             cv::Point( dilation_size, dilation_size ) );
      cv::Mat dilated;
      cv::dilate( img, dilated, element_dilate );

      cv_bridge::CvImage cv_image;
      cv_image.image = dilated;
      cv_image.encoding = "bgr8";

      sensor_msgs::Image ros_image;
      cv_image.toImageMsg(ros_image);


      ros::Rate loop_rate(5);

      while (nh.ok())
      {
        ROS_INFO("Publishing static image");
        pub_.publish(ros_image);
        ros::spinOnce();
        loop_rate.sleep();
      }
  }
  else {
      printf("Publishing dynamic images\n");
      image_transport::ImageTransport it(nh);
      image_transport::Subscriber sub = it.subscribe("/camera/depth_registered/sw_registered/image_rect_raw", 1, imageCallback);

      ros::spin();
  }

  cvDestroyWindow("trackbar");
  return 0;
}

void ConfigureCamera(bool autoExposure)
{
    // See http://wiki.ros.org/dynamic_reconfigure
    // rosrun dynamic_reconfigure dynparam dump /camera/driver dump.yaml

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf;

    if (autoExposure) {
        int_param.name = "r200_dc_preset";
        int_param.value = 2;
        conf.ints.push_back(int_param);

        int_param.name = "r200_lr_auto_exposure_enabled";
        int_param.value = 1;
        conf.ints.push_back(int_param);
    }
    else {
        int_param.name = "r200_dc_preset";
        int_param.value = 2;
        conf.ints.push_back(int_param);

        int_param.name = "r200_lr_auto_exposure_enabled";
        int_param.value = 0;
        conf.ints.push_back(int_param);

        int_param.name = "r200_lr_gain";
        int_param.value = 3700;
        conf.ints.push_back(int_param);

        int_param.name = "r200_lr_exposure";
        int_param.value = 60;
        conf.ints.push_back(int_param);
    }


    srv_req.config = conf;

    ros::service::call("/camera/driver/set_parameters", srv_req, srv_resp);
}


// Image Callback
void imageCallback(const sensor_msgs::ImageConstPtr& image) {

    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(image);

        // imshow expects a float value to lie in [0,1], so we need to normalize
        // for visualization purposes.
        double max = 0.0;
        cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);

        ROS_INFO("Max distance: %f", max);

        int maxInt;
        cvCreateTrackbar("Visible distance:", "trackbar", &maxInt, 8000);
        if (maxInt > 8000 || maxInt <= 0) maxInt = 2000;
        max = maxInt;

        int clipDistance;
        cvCreateTrackbar("Clip distance:", "trackbar", &clipDistance, 8000);
        if (clipDistance > 8000 || clipDistance <= 0) clipDistance = 4000;
        float _clipDistance = clipDistance;

        cv::Mat temp;
        cv_ptr->image.convertTo(temp, CV_32FC1);

        cv::Mat thresholded;
        cv::threshold(temp, thresholded, _clipDistance, _clipDistance, cv::THRESH_TOZERO_INV);

        int erosion_size;
        cvCreateTrackbar("Erosion size:", "trackbar", &erosion_size, 10);
        if (erosion_size > 10 || erosion_size < 0) erosion_size = 3;

        int dilation_size;
        cvCreateTrackbar("Dilation size:", "trackbar", &dilation_size, 10);
        if (dilation_size > 10 || dilation_size < 0) dilation_size = 3;



        if (erosion_size > 0) {
            element_erode = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                   cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                   cv::Point( erosion_size, erosion_size ) );

            cv::erode( thresholded, thresholded, element_erode );
        }

        if (dilation_size > 0) {
            element_dilate = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                   cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                   cv::Point( dilation_size, dilation_size ) );

            cv::dilate( thresholded, thresholded, element_dilate );
        }

        cv::Mat grayBGR;
        cv::cvtColor(thresholded, grayBGR, cv::COLOR_GRAY2BGR);

        cv::Mat normalized;
        grayBGR.convertTo(normalized, CV_8UC3, 255.0/max, 0);  // see http://docs.ros.org/diamondback/api/cv_bridge/html/c++/classsensor__msgs_1_1CvBridge.html

        cv_bridge::CvImage cv_image;
        cv_image.image = normalized;
        cv_image.encoding = "bgr8";

        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);

        pub_.publish(ros_image);

        //cv::imshow("foo", normalized);
        //cv::waitKey(1);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
