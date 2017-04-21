/* http://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/ */

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

#define USE_IMAGE_SYNCHRONIZER 1


typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

typedef struct rs_intrinsics           // see https://github.com/IntelRealSense/librealsense/blob/master/include/librealsense/rs.h#L300-L310
{
    bool initialized = false;
    int           width;     /**< Width of the image in pixels */
    int           height;    /**< Height of the image in pixels */
    float         ppx;       /**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
    float         ppy;       /**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
    float         fx;        /**< Focal length of the image plane, as a multiple of pixel width */
    float         fy;        /**< Focal length of the image plane, as a multiple of pixel height */    
    float         coeffs[5]; /**< Distortion coefficients */
} rs_intrinsics;

typedef struct rs_extrinsics    // see https://github.com/IntelRealSense/librealsense/blob/master/include/librealsense/rs.h#L332-L336
{
    bool initialized = false;
    float rotation[9];    /**< Column-major 3x3 rotation matrix */
    float translation[3]; /**< Three-element translation vector, in meters */
} rs_extrinsics;  // need to program automatic /tf_static topic reader - see http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29

static void rs_deproject_pixel_to_point(float point[3], const struct rs_intrinsics * intrin, const float pixel[2], float depth);
static void rs_project_point_to_pixel(float pixel[2], const struct rs_intrinsics * intrin, const float point[3], bool undistort);
static void rs_transform_point_to_point(float to_point[3], const struct rs_extrinsics * extrin, const float from_point[3]);

static struct rs_intrinsics rgb_intrin;
static struct rs_intrinsics depth_intrin;
static struct rs_extrinsics depth_to_color;

ros::Subscriber camerainfo1_sub;
ros::Subscriber camerainfo2_sub;

cv::Mat RGB_Image;


int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image)
{
    // If position is invalid
    if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width))
        return -1;
    int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depth_image->step/depth_image->width) == 4) {
        U_FloatConvert depth_data;
        int i, endian_check = 1;
        // If big endian
        if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) ||  // Both big endian
           ((!depth_image->is_bigendian) && (*(char*)&endian_check == 1))) { // Both lil endian
            for (i = 0; i < 4; i++)
                depth_data.byte_data[i] = depth_image->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depth_data.float_data == depth_data.float_data)
                return int(depth_data.float_data*1000);
            return -1;  // If depth data invalid
        }
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++) 
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return int(depth_data.float_data*1000);
        return -1;  // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
   int temp_val;
   // If big endian
   if (depth_image->is_bigendian)
       temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
   // If little endian
   else
       temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);
   // Make sure data is valid (check if NaN)
   if (temp_val == temp_val)
       return temp_val;
   return -1;  // If depth data invalid
}


int displayX = -1;
int displayY = -1;
bool needToInit = false;
bool addRemovePt = false;
vector<cv::Point2f> points[2];
cv::Mat prevGray;

cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
cv::Size subPixWinSize(10,10);
cv::Size winSize(31,31);

const int MAX_COUNT = 500;

void WindowMouseCallback(int event, int x, int y, int flags, void* userdata) // see http://opencv-srf.blogspot.dk/2011/11/mouse-events.html
{
     /*if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == cv::EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == cv::EVENT_MOUSEMOVE )
     {
          cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }*/
    if  ( event == cv::EVENT_LBUTTONDOWN ) {
        displayX = x;
        displayY = y;
        addRemovePt = true;
    }
    else if  ( event == cv::EVENT_RBUTTONDOWN ) {
        displayX = -1;
        displayY = -1;
        needToInit = true;
    }
}



// Image Callback
void imageCallback(const sensor_msgs::ImageConstPtr& image) {
    int x, y;    
    int depth;
    char str[200];
    
    /*x = 130;
    y = 190;
    depth = ReadDepthData(y, x, image);
    ROS_INFO("X: %d, Y: %d, Depth: %d", x, y, depth);

    x = 250;
    y = 190;
    depth = ReadDepthData(y, x, image);
    ROS_INFO("X: %d, Y: %d, Depth: %d", x, y, depth);

    x = 400;
    y = 190;
    depth = ReadDepthData(y, x, image);
    ROS_INFO("X: %d, Y: %d, Depth: %d", x, y, depth);*/

    ROS_INFO("==================================");


    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(image);

        cv::Mat temp;
        cv_ptr->image.convertTo(temp, CV_32FC1);

        if (depth_to_color.initialized && rgb_intrin.initialized && depth_intrin.initialized) {

            /*cv::Mat registered_depth(rgb_intrin.height, rgb_intrin.width, CV_32FC1, cv::Scalar::all(0));
            cv::Mat registered_color(rgb_intrin.height, rgb_intrin.width, CV_8UC3, cv::Scalar::all(0));

            float depth_in_meters;
            float depth_pixel[2];
            float depth_point[3], color_point[3], color_pixel[2], registered_pixel[2];
            cv::Vec3b color_pixel_bgr;

            for (y = 0; y < temp.rows; y++) {
                float* pixel = temp.ptr<float>(y);  // point to first color in row
                for (x = 0; x < temp.cols; x++) {
                    //depth_in_meters = temp.at<float>(y,x) / 1000.0;  // see http://stackoverflow.com/questions/8932893/accessing-certain-pixel-rgb-value-in-opencv
                    depth_in_meters = *pixel++ / 1000.0;
                    depth_pixel[0] = x;
                    depth_pixel[1] = y;

                    rs_deproject_pixel_to_point(depth_point, &depth_intrin, depth_pixel, depth_in_meters);
                    rs_transform_point_to_point(color_point, &depth_to_color, depth_point);
                    rs_project_point_to_pixel(color_pixel, &rgb_intrin, color_point, true);
                    rs_project_point_to_pixel(registered_pixel, &rgb_intrin, color_point, false);

                    if (color_pixel[0] >= 0 && color_pixel[0] < RGB_Image.cols && color_pixel[1] >= 0 && color_pixel[1] < RGB_Image.rows) {
                        color_pixel_bgr = RGB_Image.at<cv::Vec3b>(color_pixel[1], color_pixel[0]);
                    } else {
                        color_pixel_bgr = cv::Vec3f(0, 0, 0);
                    }


                    if (registered_pixel[0] >= 0 && registered_pixel[0] < registered_depth.cols && registered_pixel[1] >= 0 && registered_pixel[1] < registered_depth.rows) {
                        registered_depth.at<float>(registered_pixel[1],registered_pixel[0]) = depth_in_meters * 1000;
                        registered_color.at<cv::Vec3b>(registered_pixel[1],registered_pixel[0]) = color_pixel_bgr;
                    }
                }
            }*/
            cv::Mat registered_depth(rgb_intrin.height, rgb_intrin.width, CV_32FC1, cv::Scalar::all(0));
            cv::Mat registered_depth2(rgb_intrin.height, rgb_intrin.width, CV_32FC3, cv::Scalar::all(0));

            float depth_in_meters;
            float depth_pixel[2];
            float depth_point[3], color_point[3], color_pixel[2], registered_pixel[2];
            cv::Vec3f depthPx;

            for (y = 0; y < temp.rows; y++) {
                float* pixel = temp.ptr<float>(y);  // point to first color in row
                for (x = 0; x < temp.cols; x++) {
                    //depth_in_meters = temp.at<float>(y,x) / 1000.0;  // see http://stackoverflow.com/questions/8932893/accessing-certain-pixel-rgb-value-in-opencv
                    depth_in_meters = *pixel++ / 1000.0;
                    depth_pixel[0] = x;
                    depth_pixel[1] = y;

                    rs_deproject_pixel_to_point(depth_point, &depth_intrin, depth_pixel, depth_in_meters);
                    rs_transform_point_to_point(color_point, &depth_to_color, depth_point);
                    rs_project_point_to_pixel(color_pixel, &rgb_intrin, color_point, true);
                    rs_project_point_to_pixel(registered_pixel, &rgb_intrin, color_point, false);

                    if (color_pixel[0] >= 0 && color_pixel[0] < registered_depth.cols && color_pixel[1] >= 0 && color_pixel[1] < registered_depth.rows) {
                        registered_depth.at<float>(color_pixel[1],color_pixel[0]) = depth_in_meters * 1000;
                        depthPx = cv::Vec3f(registered_pixel[0], registered_pixel[1], depth_in_meters); // store undistorted X/Y pixel coordinate + depth (in meters)
                        registered_depth2.at<cv::Vec3f>(color_pixel[1],color_pixel[0]) = depthPx;
                    }
                }
            }

            //cout << "M = " << endl << " " << temp << endl << endl;

            // Do feature tracking
            cv::Mat gray;
            cv::cvtColor(RGB_Image, gray, cv::COLOR_BGR2GRAY);

            cv::Point2f point;

            if (addRemovePt) {
                point = cv::Point2f((float)displayX, (float)displayY);
            }

            if( needToInit )
            {
                // automatic initialization
                cv::goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
                cv::cornerSubPix(gray, points[1], subPixWinSize, cv::Size(-1,-1), termcrit);
                addRemovePt = false;
            }
            else if( !points[0].empty() )
            {
                vector<uchar> status;
                vector<float> err;
                if(prevGray.empty())
                    gray.copyTo(prevGray);
                cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                     3, termcrit, 0, 0.001);
                size_t i, k;
                for( i = k = 0; i < points[1].size(); i++ )
                {
                    if( addRemovePt )
                    {
                        if( norm(point - points[1][i]) <= 5 )
                        {
                            addRemovePt = false;
                            continue;
                        }
                    }

                    if( !status[i] )
                        continue;

                    points[1][k++] = points[1][i];
                    cv::circle( RGB_Image, points[1][i], 3, cv::Scalar(0,255,0), -1, 8);
                }
                points[1].resize(k);
            }

            if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
            {
                vector<cv::Point2f> tmp;
                tmp.push_back(point);
                cv::cornerSubPix( gray, tmp, winSize, cv::Size(-1,-1), termcrit);
                points[1].push_back(tmp[0]);
                addRemovePt = false;
            }


            needToInit = false;
            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);


            // Prepare for display
            cv::Mat grayBGR;
            cv::cvtColor(registered_depth, grayBGR, cv::COLOR_GRAY2BGR);

            cv::Mat normalized;
            grayBGR.convertTo(normalized, CV_8UC3, 255.0/5000, 0);  // see http://docs.ros.org/diamondback/api/cv_bridge/html/c++/classsensor__msgs_1_1CvBridge.html

            if (RGB_Image.cols == normalized.cols) {
                cv::Mat blended;
                cv::addWeighted( normalized, 0.5, RGB_Image, 0.5, 0.0, blended);

                if (displayX != -1 && displayY != -1) {
                    cv::circle(blended, cv::Point(displayX, displayY), 2, cv::Scalar(0,255,0,255), -1); // see http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html#circle
                    cv::circle(blended, cv::Point(displayX, displayY), 1, cv::Scalar(0,0,255,255), -1);

                    depthPx = registered_depth2.at<cv::Vec3f>(displayY, displayX);
                    /*sprintf(str, "X=%d", (int)depthPx[0]);
                    cv::putText(blended, str, cv::Point(displayX+4, displayY-12+4), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255,255)); // see http://answers.opencv.org/question/6544/how-can-i-display-timer-results-with-a-c-puttext-command/
                    sprintf(str, "Y=%d", (int)depthPx[1]);
                    cv::putText(blended, str, cv::Point(displayX+4, displayY+4), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255,255));
                    sprintf(str, "Z=%d", (int)depthPx[2]);
                    cv::putText(blended, str, cv::Point(displayX+4, displayY+12+4), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255,255));*/

                    // Calcuate correct X, Y, Z world coordinate - see http://docs.ros.org/jade/api/sensor_msgs/html/msg/CameraInfo.html
                     /* # Given a 3D point [X Y Z]', the projection (x, y) of the point onto
                        #  the rectified image is given by:
                        #  [u v w]' = P * [X Y Z 1]'
                        #         x = u / w
                        #         y = v / w
                     */
                    // u = fx*X + Z*cx
                    // v = fy*Y + Z*cy
                    //
                    // x = (fx*X + Z*cx) / Z
                    // y = (fy*Y + Z*cy) / Z
                    //
                    // x = fx*X/Z + cx
                    // y = fy*Y/Z + cy
                    //
                    // X = Z * (x - cx) / fx
                    // Y = Z * (y - cy) / fy

                    int cameraX, cameraY;
                    float worldX, worldY, worldZ;

                    cameraX = depthPx[0];
                    cameraY = depthPx[1];
                    worldZ = depthPx[2];

                    worldX = worldZ * (cameraX - rgb_intrin.ppx) / rgb_intrin.fx;
                    worldY = worldZ * (cameraY - rgb_intrin.ppy) / rgb_intrin.fy;

                    sprintf(str, "X=%1.3f", worldX);
                    cv::putText(blended, str, cv::Point(displayX+4, displayY-12+4), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255,255)); // see http://answers.opencv.org/question/6544/how-can-i-display-timer-results-with-a-c-puttext-command/
                    sprintf(str, "Y=%1.3f", worldY);
                    cv::putText(blended, str, cv::Point(displayX+4, displayY+4), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255,255));
                    sprintf(str, "Z=%1.3f", worldZ);
                    cv::putText(blended, str, cv::Point(displayX+4, displayY+12+4), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255,255));
                }

                cv::imshow("view", blended);
            }

            cv_bridge::CvImage cv_image;
            cv_image.image = normalized;
            cv_image.encoding = "bgr8";
        }

        /*
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);
        pub_.publish(ros_image);*/



        ///

    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void imageCallback2(const sensor_msgs::ImageConstPtr& image) { // rgb image
    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(image);
        cv_ptr->image.copyTo(RGB_Image);
        ROS_INFO("RGB Image width: %d", RGB_Image.cols);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void camerainfo_cb(const sensor_msgs::CameraInfoConstPtr& cameraInfo) { // rgb image
    if (!strcmp(cameraInfo->distortion_model.c_str(), "plumb_bob")) { // string matches
            /*ROS_INFO("Camera info for RGB image");
            ROS_INFO("Width: %d, Height: %d", cameraInfo->width, cameraInfo->height);
            ROS_INFO("D[0] = %f, D[1] = %f, D[2] = %f, D[3] = %f, D[4] = %f", (double)cameraInfo->D[0], (double)cameraInfo->D[1], (double)cameraInfo->D[2], (double)cameraInfo->D[3], (double)cameraInfo->D[4]);
            ROS_INFO("P = [");
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[0], (double)cameraInfo->P[1], (double)cameraInfo->P[2], (double)cameraInfo->P[3]);
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[4], (double)cameraInfo->P[5], (double)cameraInfo->P[6], (double)cameraInfo->P[7]);
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[8], (double)cameraInfo->P[9], (double)cameraInfo->P[10], (double)cameraInfo->P[11]);
            ROS_INFO("]");

            ROS_INFO("==================================");*/            
            ROS_INFO("RGB");

            // Convert into intrinsics object - see https://github.com/intel-ros/realsense/blob/17b7279fb0a3bfe11bb162c0413f949d639b7a76/realsense_camera/src/base_nodelet.cpp#L682-L705
            rgb_intrin.width = cameraInfo->width;
            rgb_intrin.height = cameraInfo->height;
            rgb_intrin.fx = cameraInfo->K[0];
            rgb_intrin.ppx = cameraInfo->K[2];
            rgb_intrin.fy = cameraInfo->K[4];
            rgb_intrin.ppy = cameraInfo->K[5];
            rgb_intrin.coeffs[0] = cameraInfo->D[0];
            rgb_intrin.coeffs[1] = cameraInfo->D[1];
            rgb_intrin.coeffs[2] = cameraInfo->D[2];
            rgb_intrin.coeffs[3] = cameraInfo->D[3];
            rgb_intrin.coeffs[4] = cameraInfo->D[4];
            rgb_intrin.initialized = true;

            camerainfo1_sub.shutdown();
    }
}

void camerainfo2_cb(const sensor_msgs::CameraInfoConstPtr& cameraInfo) { // depth image
    if (!strcmp(cameraInfo->distortion_model.c_str(), "plumb_bob")) { // string matches
            /*ROS_INFO("Camera info for Depth image");
            ROS_INFO("Width: %d, Height: %d", cameraInfo->width, cameraInfo->height);
            ROS_INFO("D[0] = %f, D[1] = %f, D[2] = %f, D[3] = %f, D[4] = %f", (double)cameraInfo->D[0], (double)cameraInfo->D[1], (double)cameraInfo->D[2], (double)cameraInfo->D[3], (double)cameraInfo->D[4]);
            ROS_INFO("P = [");
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[0], (double)cameraInfo->P[1], (double)cameraInfo->P[2], (double)cameraInfo->P[3]);
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[4], (double)cameraInfo->P[5], (double)cameraInfo->P[6], (double)cameraInfo->P[7]);
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[8], (double)cameraInfo->P[9], (double)cameraInfo->P[10], (double)cameraInfo->P[11]);
            ROS_INFO("]");

            ROS_INFO("==================================");*/            
            ROS_INFO("Depth");

            // Convert into intrinsics object - see https://github.com/intel-ros/realsense/blob/17b7279fb0a3bfe11bb162c0413f949d639b7a76/realsense_camera/src/base_nodelet.cpp#L682-L705
            depth_intrin.width = cameraInfo->width;
            depth_intrin.height = cameraInfo->height;
            depth_intrin.fx = cameraInfo->K[0];
            depth_intrin.ppx = cameraInfo->K[2];
            depth_intrin.fy = cameraInfo->K[4];
            depth_intrin.ppy = cameraInfo->K[5];
            depth_intrin.coeffs[0] = cameraInfo->D[0];
            depth_intrin.coeffs[1] = cameraInfo->D[1];
            depth_intrin.coeffs[2] = cameraInfo->D[2];
            depth_intrin.coeffs[3] = cameraInfo->D[3];
            depth_intrin.coeffs[4] = cameraInfo->D[4];
            depth_intrin.initialized = true;

            camerainfo2_sub.shutdown();
    }
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
        int_param.value = 3;
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

void callback(
  const sensor_msgs::ImageConstPtr& depth_image,
  const sensor_msgs::ImageConstPtr& rgb_image
){
    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(rgb_image);
        cv_ptr->image.copyTo(RGB_Image);
        imageCallback(depth_image);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

//*** Main ***//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imagegrab");
    ros::NodeHandle n;
    printf("READY to get image\n");

    image_transport::ImageTransport it(n);

#if USE_IMAGE_SYNCHRONIZER
    typedef image_transport::SubscriberFilter ImageSubscriber;

    ImageSubscriber depth_image_sub_( it, "/camera/depth/image_raw", 1 );
    ImageSubscriber rgb_image_sub_( it, "/camera/rgb/image_raw", 1 );

    typedef message_filters::sync_policies::ApproximateTime< // ExactTime
      sensor_msgs::Image, sensor_msgs::Image
    > MySyncPolicy;

    message_filters::Synchronizer< MySyncPolicy > sync( MySyncPolicy( 50 ), depth_image_sub_, rgb_image_sub_ );

    sync.registerCallback( boost::bind( &callback, _1, _2 ) );
#else
    image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, imageCallback);
    image_transport::Subscriber sub2 = it.subscribe("/camera/rgb/image_raw", 1, imageCallback2);
#endif

    camerainfo1_sub = n.subscribe<sensor_msgs::CameraInfo>
            ("/camera/rgb/camera_info", 10, camerainfo_cb);

    camerainfo2_sub = n.subscribe<sensor_msgs::CameraInfo>
            ("/camera/depth/camera_info", 10, camerainfo2_cb);

    /*ros::Subscriber camerainfo3_sub = n.subscribe<sensor_msgs::CameraInfo>
            ("/camera/depth_registered/sw_registered/camera_info", 10, camerainfo2_cb);*/

    ConfigureCamera(true); // use auto exposure


    float rotation[] = {0.999970,0.007027,0.003347,-0.007092,0.999780,0.019726,-0.003208,-0.019749,0.999800};
    float translation[] = {-0.058334,-0.000392,-0.000706};
    int i;
    for (i = 0; i < 9; i++) {
        depth_to_color.rotation[i] = rotation[i];
    }
    for (i = 0; i < 3; i++) {
        depth_to_color.translation[i] = translation[i];
    }
    depth_to_color.initialized = true;


    cv::namedWindow("view", CV_WINDOW_KEEPRATIO);
    cv::setMouseCallback("view", WindowMouseCallback, NULL);
    cv::startWindowThread();

    ros::spin();

    cv::destroyWindow("view");
    return 0;
}


/* Test script by using rosbag
    Terminal 1: roscore
    Terminal 2: rosbag play test.bag -l --clock -u 0.2 -s 9
*/




// Taken from: https://github.com/IntelRealSense/librealsense/blob/master/include/librealsense/rsutil.h
// Demo with conversion is found here: https://github.com/IntelRealSense/librealsense/blob/master/examples/c-tutorial-3-pointcloud.c#L148-L150

/* Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera */
static void rs_deproject_pixel_to_point(float point[3], const struct rs_intrinsics * intrin, const float pixel[2], float depth)
{
    //assert(intrin->model != RS_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
    //assert(intrin->model != RS_DISTORTION_FTHETA); // Cannot deproject to an ftheta image

    float x = (pixel[0] - intrin->ppx) / intrin->fx;
    float y = (pixel[1] - intrin->ppy) / intrin->fy;
    /* Only non R200 cameras can do this
    if(intrin->model == RS_DISTORTION_INVERSE_BROWN_CONRADY)
    {
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    */
    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
}

/* Given a point in 3D space, compute the corresponding pixel coordinates in an image with no distortion or forward distortion coefficients produced by the same camera */
static void rs_project_point_to_pixel(float pixel[2], const struct rs_intrinsics * intrin, const float point[3], bool undistort) // only undistort if R200 color image
{
    //assert(intrin->model != RS_DISTORTION_INVERSE_BROWN_CONRADY); // Cannot project to an inverse-distorted image
    //assert(intrin->model != RS_DISTORTION_FTHETA); // Cannot project to an ftheta image

    float x = point[0] / point[2], y = point[1] / point[2];
    //if(intrin->model == RS_DISTORTION_MODIFIED_BROWN_CONRADY)
    //{
    if (undistort) {
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        x *= f;
        y *= f;
        float dx = x + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float dy = y + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = dx;
        y = dy;
    }
    pixel[0] = x * intrin->fx + intrin->ppx;
    pixel[1] = y * intrin->fy + intrin->ppy;
}

/* Transform 3D coordinates relative to one sensor to 3D coordinates relative to another viewpoint */
static void rs_transform_point_to_point(float to_point[3], const struct rs_extrinsics * extrin, const float from_point[3])
{
    to_point[0] = extrin->rotation[0] * from_point[0] + extrin->rotation[3] * from_point[1] + extrin->rotation[6] * from_point[2] + extrin->translation[0];
    to_point[1] = extrin->rotation[1] * from_point[0] + extrin->rotation[4] * from_point[1] + extrin->rotation[7] * from_point[2] + extrin->translation[1];
    to_point[2] = extrin->rotation[2] * from_point[0] + extrin->rotation[5] * from_point[1] + extrin->rotation[8] * from_point[2] + extrin->translation[2];
}
