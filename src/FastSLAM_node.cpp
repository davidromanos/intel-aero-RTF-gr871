/* http://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/ */

#include <string.h>
#include <inttypes.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/aruco.hpp>

#include "FastSLAM.h"
#include "utils.h"

#include <tf/transform_datatypes.h> // for Quaternion transformation

// To be able to use cout
#include <iostream>
#include <iomanip>

using namespace std;
using namespace Eigen;

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

static struct rs_intrinsics rgb_intrin;
static struct rs_intrinsics depth_intrin;
static struct rs_extrinsics depth_to_color;

ros::Subscriber camerainfo1_sub;
ros::Subscriber camerainfo2_sub;

cv::Mat RGB_Image;
cv::Mat Depth_Image;
bool RGB_Image_New = false;
bool Depth_Image_New = false;
bool RGBD_Image_Ready = false;
ros::Time PoseTimestamp(0);
ros::Duration RGBD_Timestamp;
ros::Time Time0(0);
ros::Duration DepthToPose_TimeOffset(0);

cv::aruco::Dictionary markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
Eigen::IOFormat OctaveFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]"); // see https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html#a840cac6401adc4de421260d63dc3d861
Eigen::IOFormat TSVFmt(Eigen::FullPrecision, Eigen::DontAlignCols, "", "\t", "", "", "", ""); // tab seperated
Eigen::IOFormat CSVFmt(Eigen::FullPrecision, Eigen::DontAlignCols, "", ", ", "", "", "", ""); // comma seperated

Vector6f MocapPose;

ofstream MocapLog;
ofstream CameraLog;
ofstream IntrinsicsLog;
ofstream MocapVelocityLog;
ofstream MotionModelLog;

// ==== FastSLAM variables ====
int Nparticles;
unsigned int GOT_MeasurementID;
VectorChiFastSLAMf s0 = VectorChiFastSLAMf::Constant(0);
MatrixChiFastSLAMf s_0_Cov;
ParticleSet* Pset;
VectorUFastSLAMf u;



// ==== Function definitions ====
static void rs_deproject_pixel_to_point(float point[3], const struct rs_intrinsics * intrin, const float pixel[2], float depth);
static void rs_project_point_to_pixel(float pixel[2], const struct rs_intrinsics * intrin, const float point[3], bool undistort);
static void rs_transform_point_to_point(float to_point[3], const struct rs_extrinsics * extrin, const float from_point[3]);

void MocapPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
void Depth_Image_Callback(const sensor_msgs::ImageConstPtr& image);
void RGB_Image_Callback(const sensor_msgs::ImageConstPtr& image);
void RGBD_Image_Callback(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::ImageConstPtr& rgb_image);
void CameraInfo_RGB_Callback(const sensor_msgs::CameraInfoConstPtr& cameraInfo);
void CameraInfo_Depth_Callback(const sensor_msgs::CameraInfoConstPtr& cameraInfo);
void ConfigureCamera(bool autoExposure);
void InitHardcodedExtrinsics(void);
void ProcessRGBDimage(MeasurementSet * MeasSet);



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

/* 2nd order IIR filter
          D = designfilt('lowpassiir', 'FilterOrder', 2, ...
             'PassbandFrequency', 8, 'PassbandRipple', 0.5,...
             'SampleRate', round(1/mean(diff(td)), 0)); */
const float LPF_COEFF_A[] = {1.000000000000000,         -1.22790041314432,         0.500627356053467}; // 2nd order IIR filter
const float LPF_COEFF_B[] = {0.0643677091773966,         0.128735418354793,        0.0643677091773966};

IIR Xdot_Filter(LPF_COEFF_A, 3, LPF_COEFF_B, 3);
IIR Ydot_Filter(LPF_COEFF_A, 3, LPF_COEFF_B, 3);
IIR Zdot_Filter(LPF_COEFF_A, 3, LPF_COEFF_B, 3);
IIR Roll_Filter(LPF_COEFF_A, 3, LPF_COEFF_B, 3);
IIR Pitch_Filter(LPF_COEFF_A, 3, LPF_COEFF_B, 3);
IIR Yaw_Filter(LPF_COEFF_A, 3, LPF_COEFF_B, 3);

Vector6f PreviousPose = Vector6f::Zero();
ros::Time PreviousTimestamp(0);
bool SkipMeasurement = false; // used to skip every second measurement

Vector3f MocapVelocity = Vector3f::Zero();
Vector3f DroneVelocity = Vector3f::Zero();

void MocapVelocityFilter()
{
    ros::Duration dt;
    float dx, dy, dz;

    if (!SkipMeasurement) {
        SkipMeasurement = true;
        dt = PoseTimestamp - PreviousTimestamp;
        dx = MocapPose(0) - PreviousPose(0);
        dy = MocapPose(1) - PreviousPose(1);
        dz = MocapPose(2) - PreviousPose(2);

        //cout << "dt: " << dt.toSec() << endl;
        //cout << "xdot: " << (dx / dt.toSec()) << endl;

        if (dt.toSec() < 0.005) {
            SkipMeasurement = false;
            return;
        }

        if (dx != 0)
            MocapVelocity(0) = Xdot_Filter.Filter(dx / dt.toSec());
        else {
            //MocapVelocity(0) = Xdot_Filter.Filter(MocapVelocity(0)); // velocity can not be zero, so we assume that it is because of no measurement, hence take the previous velocity
            MocapVelocity(0) = MocapVelocity(0); // velocity can not be zero, so we assume that it is because of no measurement, hence take the previous velocity
            SkipMeasurement = false;
        }

        if (dy != 0)
            MocapVelocity(1) = Ydot_Filter.Filter(dy / dt.toSec());
        else {
            //MocapVelocity(1) = Ydot_Filter.Filter(MocapVelocity(1)); // velocity can not be zero, so we assume that it is because of no measurement, hence take the previous velocity
            MocapVelocity(1) = MocapVelocity(1); // velocity can not be zero, so we assume that it is because of no measurement, hence take the previous velocity
            SkipMeasurement = false;
        }

        if (dz != 0)
            MocapVelocity(2) = Zdot_Filter.Filter(dz / dt.toSec());
        else {
            //MocapVelocity(2) = Zdot_Filter.Filter(MocapVelocity(2)); // velocity can not be zero, so we assume that it is because of no measurement, hence take the previous velocity
            MocapVelocity(2) = MocapVelocity(2); // velocity can not be zero, so we assume that it is because of no measurement, hence take the previous velocity
            SkipMeasurement = false;
        }

        //cout << "xdot filt: " << MocapVelocity(0) << endl;

        // Convert MocapVelocity into Drone velocity by applying Yaw rotation from previous pose
        // R = [cos(psi)  sin(psi);        % For rotation from world velocity into drone velocity
        //      -sin(psi) cos(psi)]
        // DroneVel = R * MocapVel

        DroneVelocity(0) = cos(PreviousPose(5))*MocapVelocity(0) + sin(PreviousPose(5))*MocapVelocity(1);
        DroneVelocity(1) = -sin(PreviousPose(5))*MocapVelocity(0) + cos(PreviousPose(5))*MocapVelocity(1);
        DroneVelocity(2) = MocapVelocity(2);

        //cout << "DroneVelocity: " << endl << DroneVelocity << endl;

        PreviousPose = MocapPose;
        PreviousTimestamp = PoseTimestamp;

        if (!Time0.isZero()) { // only log if time is synchronized
            logAppendTimestamp(MocapVelocityLog, (PoseTimestamp - Time0));
            MocapVelocityLog << MocapVelocity.format(CSVFmt) << ", " << DroneVelocity.format(CSVFmt) << endl;
            MocapVelocityLog.flush();
        }
    } else {
        SkipMeasurement = false;
    }
}

void MocapPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
    double roll, pitch, yaw;
    float rollFilt, pitchFilt, yawFilt;

    PoseTimestamp = pose->header.stamp;

    tf::Quaternion q1;

    q1.setW(pose->pose.orientation.w);
    q1.setX(pose->pose.orientation.x);
    q1.setY(pose->pose.orientation.y);
    q1.setZ(pose->pose.orientation.z);

    tf::Matrix3x3 m(q1);

    m.getRPY(roll, pitch, yaw);

    rollFilt = Roll_Filter.Filter(roll);
    pitchFilt = Pitch_Filter.Filter(pitch);
    yawFilt = Yaw_Filter.Filter(yaw);

    MocapPose << pose->pose.position.x,
                 pose->pose.position.y,
                 pose->pose.position.z,
                 roll,
                 pitch,
                 yaw;

    MocapVelocityFilter();


    if (!Time0.isZero()) { // only log if time is synchronized
        logAppendTimestamp(MocapLog, (pose->header.stamp - Time0));
        MocapLog << MocapPose.format(CSVFmt) << endl;
        MocapLog.flush();
    }
}

// Image Callback
void Depth_Image_Callback(const sensor_msgs::ImageConstPtr& image) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(image);
        cv_ptr->image.convertTo(Depth_Image, CV_32FC1);
        Depth_Image_New = true;

        if (!PoseTimestamp.isZero() && Time0.isZero()) { // timestamp synchronization hack
            ros::Time depthTime = image->header.stamp;
            if (depthTime > PoseTimestamp) {
                Time0 = PoseTimestamp;
            } else {
                Time0 = depthTime;
            }
            PreviousTimestamp = PoseTimestamp;

            DepthToPose_TimeOffset = depthTime - PoseTimestamp;
            cout << "depthTime: " << depthTime << " - poseTime: " << PoseTimestamp << endl;
            cout << "delta time (offset): " << DepthToPose_TimeOffset << endl;
        }

        if (depth_to_color.initialized && rgb_intrin.initialized && depth_intrin.initialized && RGB_Image_New && Depth_Image_New) {
            RGBD_Timestamp = image->header.stamp - DepthToPose_TimeOffset - Time0; // subtract to get RGBD timestamp into Pose timestamp
            RGBD_Image_Ready = true;
            RGB_Image_New = false;
            Depth_Image_New = false;
        }
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void RGB_Image_Callback(const sensor_msgs::ImageConstPtr& image) { // rgb image
    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(image);
        cv_ptr->image.copyTo(RGB_Image);
        RGB_Image_New = true;

        if (depth_to_color.initialized && rgb_intrin.initialized && depth_intrin.initialized && RGB_Image_New && Depth_Image_New) {
            RGBD_Timestamp = image->header.stamp - DepthToPose_TimeOffset - Time0; // subtract to get RGBD timestamp into Pose timestamp
            RGBD_Image_Ready = true;
            RGB_Image_New = false;
            Depth_Image_New = false;
        }
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void RGBD_Image_Callback(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::ImageConstPtr& rgb_image) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(rgb_image);
        cv_ptr->image.copyTo(RGB_Image);

        cv_bridge::CvImageConstPtr cv_ptr2;
        cv_ptr2 = cv_bridge::toCvShare(depth_image);
        cv_ptr2->image.convertTo(Depth_Image, CV_32FC1);

        if (!PoseTimestamp.isZero() && Time0.isZero()) { // timestamp synchronization hack
            ros::Time depthTime = depth_image->header.stamp;
            //if (depthTime > PoseTimestamp) {
                Time0 = PoseTimestamp;
            /*} else {
                Time0 = depthTime;
            }*/
            PreviousTimestamp = PoseTimestamp;

            DepthToPose_TimeOffset = depthTime - PoseTimestamp;
            cout << "depthTime: " << depthTime << " - poseTime: " << PoseTimestamp << endl;
            cout << "delta time (offset): " << DepthToPose_TimeOffset << endl;
        }

        else if (depth_to_color.initialized && rgb_intrin.initialized && depth_intrin.initialized) {
            //if (depth_image->header.stamp > rgb_image->header.stamp) {
                RGBD_Timestamp = depth_image->header.stamp - DepthToPose_TimeOffset - Time0; // subtract to get RGBD timestamp into Pose timestamp
            /*} else {
                RGBD_Timestamp = rgb_image->header.stamp - DepthToPose_TimeOffset - Time0; // subtract to get RGBD timestamp into Pose timestamp
            }*/
            RGBD_Image_Ready = true;
            RGB_Image_New = false;
            Depth_Image_New = false;

//            ROS_INFO("New RGBD image ready");
        }

    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void CameraInfo_RGB_Callback(const sensor_msgs::CameraInfoConstPtr& cameraInfo) { // rgb image
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
//            ROS_INFO("RGB intrinsics received");

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

            IntrinsicsLog << "rgb,";
            IntrinsicsLog << rgb_intrin.width << ",";
            IntrinsicsLog << rgb_intrin.height << ",";
            IntrinsicsLog << setprecision(14) << rgb_intrin.fx << ",";
            IntrinsicsLog << setprecision(14) << rgb_intrin.ppx << ",";
            IntrinsicsLog << setprecision(14) << rgb_intrin.fy << ",";
            IntrinsicsLog << setprecision(14) << rgb_intrin.ppy << ",";
            IntrinsicsLog << setprecision(14) << rgb_intrin.coeffs[0] << ",";
            IntrinsicsLog << setprecision(14) << rgb_intrin.coeffs[1] << ",";
            IntrinsicsLog << setprecision(14) << rgb_intrin.coeffs[2] << ",";
            IntrinsicsLog << setprecision(14) << rgb_intrin.coeffs[3] << ",";
            IntrinsicsLog << setprecision(14) << rgb_intrin.coeffs[4] << endl;

            camerainfo1_sub.shutdown(); // now that we have received the intrinsics we don't need to subscribe to the topic anymore
    }
}

void CameraInfo_Depth_Callback(const sensor_msgs::CameraInfoConstPtr& cameraInfo) { // depth image
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
            ROS_INFO("Depth intrinsics received");

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

            IntrinsicsLog << "depth,";
            IntrinsicsLog << depth_intrin.width << ",";
            IntrinsicsLog << depth_intrin.height << ",";
            IntrinsicsLog << setprecision(14) << depth_intrin.fx << ",";
            IntrinsicsLog << setprecision(14) << depth_intrin.ppx << ",";
            IntrinsicsLog << setprecision(14) << depth_intrin.fy << ",";
            IntrinsicsLog << setprecision(14) << depth_intrin.ppy << ",";
            IntrinsicsLog << setprecision(14) << depth_intrin.coeffs[0] << ",";
            IntrinsicsLog << setprecision(14) << depth_intrin.coeffs[1] << ",";
            IntrinsicsLog << setprecision(14) << depth_intrin.coeffs[2] << ",";
            IntrinsicsLog << setprecision(14) << depth_intrin.coeffs[3] << ",";
            IntrinsicsLog << setprecision(14) << depth_intrin.coeffs[4] << endl;

            camerainfo2_sub.shutdown(); // now that we have received the intrinsics we don't need to subscribe to the topic anymore
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

void InitHardcodedExtrinsics(void)
{
    // Hardcoded initialization of Extrinsics, taken from the R200 camera on our Intel Aero drone
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
}


cv::Vec3f GetWorldCoordinateFromMeasurement(cv::Vec3f meas)
{
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

    cv::Vec3f WorldPos;
    int cameraX, cameraY;

    cameraX = meas[0];
    cameraY = meas[1];
    WorldPos[2] = meas[2];

    WorldPos[0] = WorldPos[2] * (cameraX - rgb_intrin.ppx) / rgb_intrin.fx;
    WorldPos[1] = WorldPos[2] * (cameraY - rgb_intrin.ppy) / rgb_intrin.fy;

    return WorldPos;
}

VectorChiFastSLAMf motionModel(VectorChiFastSLAMf sold, VectorUFastSLAMf* u, float Ts) // Ts == sample time
{
    VectorChiFastSLAMf s_k = sold; // s(k) = f(s(k-1),u(k))

    if (Ts > 3) {
        return s_k; // error with the sampling time, just return old pose estimate
    }

    // Kinematic motion model where u=[x_dot, y_dot, z_dot, yaw_difference]  with x_dot and y_dot being in heading frame
    //cout << "dt: " << Ts << endl;
    //cout << "Motion input: " << endl << (*u) << endl;

    // Convert Drone velocity into World velocity by applying Yaw rotation from old pose
    // R = [cos(psi)  -sin(psi);        % For rotation from drone velocity into world velocity
    //      sin(psi) cos(psi)]
    // WorldVel = R * DroneVel
    s_k(0) = s_k(0) + Ts * (cos(sold(3))* (*u)(0) - sin(sold(3))* (*u)(1));
    s_k(1) = s_k(1) + Ts * (sin(sold(3))* (*u)(0) + cos(sold(3))* (*u)(1));

    s_k(2) = s_k(2) + Ts * (*u)(2); // add integrated zdot contribution

    s_k(3) = s_k(3) + (*u)(3); // add yaw difference

    return s_k;
}


void ProcessRGBDimage(MeasurementSet * MeasSet)
{
    int x, y;
    char str[200];

    if (RGBD_Image_Ready) {
//        cout << "Processing RGB data" << endl;
        cv::Mat RGB;
        cv::Mat Depth;
        RGB_Image.copyTo(RGB);
        Depth_Image.copyTo(Depth);

        RGBD_Image_Ready = false;
        cv::Mat registered_depth(rgb_intrin.height, rgb_intrin.width, CV_32FC1, cv::Scalar::all(0));
        cv::Mat registered_depth2(rgb_intrin.height, rgb_intrin.width, CV_32FC3, cv::Scalar::all(0));

        float depth_in_meters;
        float depth_pixel[2];
        float depth_point[3], color_point[3], color_pixel[2], registered_pixel[2];
        cv::Vec3f depthPx;

        for (y = 0; y < Depth.rows; y++) {
            float* pixel = Depth.ptr<float>(y);  // point to first color in row
            for (x = 0; x < Depth.cols; x++) {
                //depth_in_meters = Depth.at<float>(y,x) / 1000.0;  // see http://stackoverflow.com/questions/8932893/accessing-certain-pixel-rgb-value-in-opencv
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

        // Prepare for display
        cv::Mat grayBGR;
        cv::cvtColor(registered_depth, grayBGR, cv::COLOR_GRAY2BGR);

        cv::Mat normalized;
        grayBGR.convertTo(normalized, CV_8UC3, 255.0/5000, 0);  // see http://docs.ros.org/diamondback/api/cv_bridge/html/c++/classsensor__msgs_1_1CvBridge.html

        if (RGB.cols == normalized.cols) {
            cv::Mat blended;
            cv::addWeighted( normalized, 0.5, RGB, 0.5, 0.0, blended);

            // Perform Aruco detection
            vector<int> markerIds;
            vector<vector<cv::Point2f> > markerCorners, rejectedCandidates;
            cv::aruco::detectMarkers(RGB, markerDictionary, markerCorners, markerIds);
            cv::aruco::drawDetectedMarkers(blended, markerCorners, markerIds);

//            cout << "Detected markers: " << markerCorners.size() << endl;

            unsigned int ID;
            int dispX,dispY;
            vector<cv::Point2f> MarkerPoints;
            cv::Vec3f MarkerMeas; // cameraX, cameraY, worldZ (depth)
            float Xmean, Ymean, DepthMean;
            int ValuesAddedToMeanCount;
            signed int meanX, meanY;
            Eigen::Vector3f MarkerMeas_;
            ImgMeasurement* z_img;

            for (int i = 0; i < markerCorners.size(); i++) {
                ValuesAddedToMeanCount = 0;
                Xmean = 0.f;
                Ymean = 0.f;
                DepthMean = 0.f;
                MarkerPoints = markerCorners[i];
                cv::Point2f point = MarkerPoints[0];                
                // Get mean depth
                for (meanY = -1; meanY <= 1; meanY++) {
                    for (meanX = -1; meanX <= 1; meanX++) {
                        if ((point.x+meanX >= 0 && point.x+meanX < rgb_intrin.width) && (point.y+meanY >= 0 && point.y+meanY < rgb_intrin.height)) {
                            MarkerMeas = registered_depth2.at<cv::Vec3f>(point.y+meanY, point.x+meanX);
                            //cout << MarkerMeas[2] << " ";
                            if (MarkerMeas[2] > 0) {
                                Xmean += MarkerMeas[0];
                                Ymean += MarkerMeas[1];
                                DepthMean += MarkerMeas[2];
                                ValuesAddedToMeanCount++;
                            }
                        }
                    }
                }                

                if (ValuesAddedToMeanCount > 0) {
                    Xmean = Xmean / (float)ValuesAddedToMeanCount;
                    Ymean = Ymean / (float)ValuesAddedToMeanCount;
                    DepthMean = DepthMean / (float)ValuesAddedToMeanCount;
                }

                if (DepthMean > 0) {
                    MarkerMeas[0] = Xmean;
                    MarkerMeas[1] = Ymean;
                    MarkerMeas[2] = DepthMean;
                    /*MarkerMeas = registered_depth2.at<cv::Vec3f>(point.y, point.x);
                    if (MarkerMeas[2] > 0) {*/

                    MarkerMeas_(0) = MarkerMeas[0];
                    MarkerMeas_(1) = MarkerMeas[1];
                    MarkerMeas_(2) = MarkerMeas[2];

                    ID = (unsigned int)markerIds[i] + 1; // make sure ID go from 1 and up

//                    ROS_INFO("Marker ID %u at (%f, %f, %f)", ID, MarkerMeas_(0), MarkerMeas_(1), MarkerMeas_(2));

                    z_img = new ImgMeasurement(ID, MarkerMeas_, MocapPose(3), MocapPose(4)); // ID, Marker measurements and include current/latest raw Roll and Pitch measurement (in this case directly from Mocap instead of from the estimator)
                    MeasSet->addMeasurement(z_img);

                    dispX = point.x;
                    dispY = point.y;
                    cv::Vec3f World = GetWorldCoordinateFromMeasurement(MarkerMeas);

                    sprintf(str, "X=%1.3f", World[0]);
                    cv::putText(blended, str, cv::Point(dispX+4, dispY-12+4), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255,255)); // see http://answers.opencv.org/question/6544/how-can-i-display-timer-results-with-a-c-puttext-command/
                    sprintf(str, "Y=%1.3f", World[1]);
                    cv::putText(blended, str, cv::Point(dispX+4, dispY+4), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255,255));
                    sprintf(str, "Z=%1.3f", World[2]);
                    cv::putText(blended, str, cv::Point(dispX+4, dispY+12+4), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255,255));

                    logAppendTimestamp(CameraLog, RGBD_Timestamp);
                    CameraLog << ID << ", " << MarkerMeas_.format(CSVFmt) << endl;
                }
            }

            CameraLog.flush();

            cv::imshow("view", blended);

            cv::waitKey(1); // this is necessary to show the image in the view from OpenCV 3
        }
    }
}


//*** Main ***//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "FastSLAM_node");
    ros::NodeHandle n;
    printf("READY to get image\n");

    prepareLogFile(&MocapLog, "Mocap");
    if (!MocapLog.is_open()) {
        ROS_ERROR("Error opening Mocap log file");
        return -1;
    }

    prepareLogFile(&MocapVelocityLog, "MocapVelocity");
    if (!MocapVelocityLog.is_open()) {
        ROS_ERROR("Error opening Mocap Velocity log file");
        return -1;
    }

    prepareLogFile(&MotionModelLog, "MotionModel");
    if (!MotionModelLog.is_open()) {
        ROS_ERROR("Error opening Motion Model log file");
        return -1;
    }


    prepareLogFile(&CameraLog, "Camera");
    if (!CameraLog.is_open()) {
        ROS_ERROR("Error opening Camera log file");
        return -1;
    }

    prepareLogFile(&IntrinsicsLog, "Intrinsics");
    if (!IntrinsicsLog.is_open()) {
        ROS_ERROR("Error opening Intrinsics log file");
        return -1;
    }

    vector<vector<double> > Config = load_csv("src/intel_aero_rtf_gr871/src/FastSLAM/config.csv");
    if (Config.size() == 0) {
        ROS_ERROR("Error loading config.csv");
        ROS_ERROR("Config should be placed within catkin_ws/src/intel_aero_rtf_gr871/src/FastSLAM/");
        ROS_ERROR("Remember to run the node from the catkin_ws folder");
        return -1;
    }

    // ===== Configure FastSLAM =====
    Nparticles = Config[0][0];
    cout << "Config.Nparticles" << endl << Nparticles << endl;

    // motion model covariance
    Particle::sCov(0,0) = pow(Config[1][0]/3,2); //0.05;
    Particle::sCov(1,1) = pow(Config[1][1]/3,2); //0.05;
    Particle::sCov(2,2) = pow(Config[1][2]/3,2); //0.05;
    Particle::sCov(3,3) = pow(Config[1][3]*(pi/180)/3,2); //0.15*0.349066; // 20 degrees
    cout << "Config.sCov = " << endl << Particle::sCov << endl;

    GOTMeasurement::zCov(0,0) = pow(Config[2][0]/3,2); //0.001;
    GOTMeasurement::zCov(1,1) = pow(Config[2][1]/3,2); //0.001;
    GOTMeasurement::zCov(2,2) = pow(Config[2][2]/3,2); //0.001;
    cout << "Config.GOT.zCov = " << endl << GOTMeasurement::zCov << endl;

    ImgMeasurement::zCov(0,0) = pow(Config[3][0]/3,2); //5;
    ImgMeasurement::zCov(1,1) = pow(Config[3][1]/3,2); //5;
    ImgMeasurement::zCov(2,2) = pow(Config[3][2]/3,2); //0.2;
    cout << "Config.Img.zCov = " << endl << ImgMeasurement::zCov << endl;

    ImgMeasurement::CameraOffset(0) = Config[4][0];
    ImgMeasurement::CameraOffset(1) = Config[4][1];
    ImgMeasurement::CameraOffset(2) = Config[4][2];
    cout << "Config.Img.CameraOffset = " << endl << ImgMeasurement::CameraOffset << endl;

    float GOT_loss_xyz[5];
    GOT_loss_xyz[0] = Config[5][0]; // xmin
    GOT_loss_xyz[1] = Config[5][1]; // xmax
    GOT_loss_xyz[2] = Config[5][2]; // ymin
    GOT_loss_xyz[3] = Config[5][3]; // ymax
    GOT_loss_xyz[4] = Config[5][4]; // zmin
    GOT_loss_xyz[5] = Config[5][5]; // zmax

    float GOT_loss_time[2];
    GOT_loss_time[0] = Config[6][0]; // start
    GOT_loss_time[1] = Config[6][1]; // end
    cout << "Config.GOT.LossStart = " << endl << GOT_loss_time[0] << endl;
    cout << "Config.GOT.LossEnd = " << endl << GOT_loss_time[1] << endl;

    // ==== End configuration of FastSLAM ====


    image_transport::ImageTransport it(n);

#if USE_IMAGE_SYNCHRONIZER
    typedef image_transport::SubscriberFilter ImageSubscriber;

    ImageSubscriber depth_image_sub_( it, "/camera/depth/image_raw", 1 );
    ImageSubscriber rgb_image_sub_( it, "/camera/rgb/image_raw", 1 );

    typedef message_filters::sync_policies::ApproximateTime< // ExactTime
      sensor_msgs::Image, sensor_msgs::Image
    > MySyncPolicy;

    message_filters::Synchronizer< MySyncPolicy > sync( MySyncPolicy( 50 ), depth_image_sub_, rgb_image_sub_ );

    sync.registerCallback( boost::bind( &RGBD_Image_Callback, _1, _2 ) );
#else
    image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, Depth_Image_Callback);
    image_transport::Subscriber sub2 = it.subscribe("/camera/rgb/image_raw", 1, RGB_Image_Callback);
#endif

    camerainfo1_sub = n.subscribe<sensor_msgs::CameraInfo>
            ("/camera/rgb/camera_info", 20, CameraInfo_RGB_Callback);

    camerainfo2_sub = n.subscribe<sensor_msgs::CameraInfo>
            ("/camera/depth/camera_info", 20, CameraInfo_Depth_Callback);

    ros::Subscriber position_sub = n.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/mocap/pose", 1000, MocapPose_Callback);

    ConfigureCamera(true); // use auto exposure
    InitHardcodedExtrinsics(); // Hardcoded initialization of Extrinsics, taken from the R200 camera on our Intel Aero drone

    // Wait for intrinsics to arrive
    while(ros::ok() && (!depth_to_color.initialized || !rgb_intrin.initialized || !depth_intrin.initialized || Time0.isZero() || ((PoseTimestamp - Time0).toSec() <= 0))) {
        ros::spinOnce();
    }

    ImgMeasurement::ax = rgb_intrin.fx;
    ImgMeasurement::ay = rgb_intrin.fy;
    ImgMeasurement::x0 = rgb_intrin.ppx;
    ImgMeasurement::y0 = rgb_intrin.ppy;

    // ===== Configure FastSLAM =====
    GOT_MeasurementID = 49;
    s0 << MocapPose(0), MocapPose(1), MocapPose(2), MocapPose(5); // take starting Mocap Pose as initial particle location
    s_0_Cov = MatrixChiFastSLAMf::Zero(); // motion model covariance is initialized below
    ParticleSet Pset(Nparticles,GOT_MeasurementID,s0,s_0_Cov);
    VectorUFastSLAMf u = VectorUFastSLAMf::Zero();
    cout << "Initial particle location: " << endl << s0 << endl;
    // ==== End configuration of FastSLAM ====

    RGB_Image_New = false;
    Depth_Image_New = false;
    RGBD_Image_Ready = false;

    cv::namedWindow("view", CV_WINDOW_KEEPRATIO);
    cv::startWindowThread();

    MeasurementSet MeasSet;
    ros::Duration dt;
    ros::Time PreviousMeasurementTimestamp = PoseTimestamp;
    GOTMeasurement* z_GOT;
    Eigen::Vector3f GOT_meas;
    float PreviousYaw = MocapPose(5);
    float YawDifference = 0.f;

    VectorChiFastSLAMf s_k = VectorChiFastSLAMf::Zero();

    while(ros::ok()){
        ros::spinOnce(); // process the latest measurements in the queue (subscribers) and move these into the RGB_Image and Depth_Image objects

        ProcessRGBDimage(&MeasSet);

        YawDifference = MocapPose(5) - PreviousYaw;
        dt = PoseTimestamp - PreviousMeasurementTimestamp;

        //if (MeasSet.getNumberOfMeasurements() > 0 && dt.toSec() > 0) {
        if (dt.toSec() > 0) {
            cout << "Time: " << (PoseTimestamp-Time0).toSec() << endl;
           if ( ((PoseTimestamp-Time0).toSec() < GOT_loss_time[0]) || ((PoseTimestamp-Time0).toSec() > (GOT_loss_time[1])) ) { // simulate time loss of GOT
                if(!(MocapPose(0)>GOT_loss_xyz[0] && MocapPose(0)<GOT_loss_xyz[1])){  // simulate position loss of GOT
                    if(!(MocapPose(1)>GOT_loss_xyz[2] && MocapPose(1)<GOT_loss_xyz[3])){
                        if(!(MocapPose(2)>GOT_loss_xyz[4] && MocapPose(2)<GOT_loss_xyz[5])){
                            cout << "GOT enabled" << endl;
                            GOT_meas << MocapPose(0), MocapPose(1), MocapPose(2);
                            z_GOT = new GOTMeasurement(GOT_MeasurementID, GOT_meas); // ID, Marker measurements and include current/latest raw Roll and Pitch measurement (in this case directly from Mocap instead of from the estimator)
                            MeasSet.addMeasurement(z_GOT);
                        }
                    }
                }
            } else {
               cout << "GOT disabled" << endl;
            }
            u << DroneVelocity, YawDifference;
            //cout << "dt: " << dt.toSec() << endl;
            //cout << "Motion input: " << endl << u << endl;

            s_k = motionModel(s_k, &u, dt.toSec());

            logAppendTimestamp(MotionModelLog, (PoseTimestamp - Time0));
            MotionModelLog << dt.toSec() << ", " << u.format(CSVFmt) << ", " << s_k.format(CSVFmt) << endl;
            MotionModelLog.flush();

            Pset.updateParticleSet(&MeasSet, u, dt.toSec());

            MeasSet.emptyMeasurementSet();

            PreviousYaw = MocapPose(5);
            PreviousMeasurementTimestamp = PoseTimestamp;
        }
    }

    Pset.saveData();

    MocapLog.close();
    CameraLog.close();
    IntrinsicsLog.close();
    MocapVelocityLog.close();
    MotionModelLog.close();

    cv::destroyWindow("view");
    return 0;
}
