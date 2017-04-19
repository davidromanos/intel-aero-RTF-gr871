/* http://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/ */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

typedef struct rs_intrinsics
{
    int           width;     /**< Width of the image in pixels */
    int           height;    /**< Height of the image in pixels */
    float         ppx;       /**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
    float         ppy;       /**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
    float         fx;        /**< Focal length of the image plane, as a multiple of pixel width */
    float         fy;        /**< Focal length of the image plane, as a multiple of pixel height */    
    float         coeffs[5]; /**< Distortion coefficients */
} rs_intrinsics;

static void rs_deproject_pixel_to_point(float point[3], const struct rs_intrinsics * intrin, const float pixel[2], float depth);
static void rs_project_point_to_pixel(float pixel[2], const struct rs_intrinsics * intrin, const float point[3], bool undistort);

static struct rs_intrinsics rgb_intrin;
static struct rs_intrinsics depth_intrin;


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

// Image Callback
void imageCallback(const sensor_msgs::ImageConstPtr& image) {
    int x, y;
    int xi, yi, depthi, i;
    int depth;
    
    x = 130;
    y = 190;
    depth = 0;
    i = 0;
    for (xi = -1; xi <= 1; xi++) {
        for (yi = -1; yi <= 1; yi++) {
            depthi = ReadDepthData(y+yi, x+xi, image);
            if (depthi != 0) {
                depth += depthi;
                i++;
            }
        }
    }
    depth = depthi / i;
    ROS_INFO("X: %d, Y: %d, Depth: %d", x, y, depth);

    x = 250;
    y = 190;
    depth = ReadDepthData(y, x, image);
    ROS_INFO("X: %d, Y: %d, Depth: %d", x, y, depth);

    x = 400;
    y = 190;
    depth = ReadDepthData(y, x, image);
    ROS_INFO("X: %d, Y: %d, Depth: %d", x, y, depth);

    ROS_INFO("==================================");
}

void camerainfo_cb(const sensor_msgs::CameraInfoConstPtr& cameraInfo) {
    if (!strcmp(cameraInfo->distortion_model.c_str(), "plumb_bob")) { // string matches
        if (cameraInfo->width > 480) { // rgb image
            ROS_INFO("Camera info for RGB image");
            ROS_INFO("Width: %d, Height: %d", cameraInfo->width, cameraInfo->height);
            ROS_INFO("D[0] = %f, D[1] = %f, D[2] = %f, D[3] = %f, D[4] = %f", (double)cameraInfo->D[0], (double)cameraInfo->D[1], (double)cameraInfo->D[2], (double)cameraInfo->D[3], (double)cameraInfo->D[4]);
            ROS_INFO("P = [");
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[0], (double)cameraInfo->P[1], (double)cameraInfo->P[2], (double)cameraInfo->P[3]);
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[4], (double)cameraInfo->P[5], (double)cameraInfo->P[6], (double)cameraInfo->P[7]);
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[8], (double)cameraInfo->P[9], (double)cameraInfo->P[10], (double)cameraInfo->P[11]);
            ROS_INFO("]");

            ROS_INFO("==================================");

            // Convert into intrinsics object - see https://github.com/intel-ros/realsense/blob/17b7279fb0a3bfe11bb162c0413f949d639b7a76/realsense_camera/src/base_nodelet.cpp#L682-L705
            //rgb_intrin
        }
        else { // depth image
            ROS_INFO("Camera info for Depth image");
            ROS_INFO("Width: %d, Height: %d", cameraInfo->width, cameraInfo->height);
            ROS_INFO("D[0] = %f, D[1] = %f, D[2] = %f, D[3] = %f, D[4] = %f", (double)cameraInfo->D[0], (double)cameraInfo->D[1], (double)cameraInfo->D[2], (double)cameraInfo->D[3], (double)cameraInfo->D[4]);
            ROS_INFO("P = [");
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[0], (double)cameraInfo->P[1], (double)cameraInfo->P[2], (double)cameraInfo->P[3]);
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[4], (double)cameraInfo->P[5], (double)cameraInfo->P[6], (double)cameraInfo->P[7]);
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[8], (double)cameraInfo->P[9], (double)cameraInfo->P[10], (double)cameraInfo->P[11]);
            ROS_INFO("]");

            ROS_INFO("==================================");

            // Convert into intrinsics object - see https://github.com/intel-ros/realsense/blob/17b7279fb0a3bfe11bb162c0413f949d639b7a76/realsense_camera/src/base_nodelet.cpp#L682-L705
            //depth_intrin
        }
    }
}

void camerainfo2_cb(const sensor_msgs::CameraInfoConstPtr& cameraInfo) {
    if (!strcmp(cameraInfo->distortion_model.c_str(), "plumb_bob")) { // string matches
            ROS_INFO("Camera info for Depth image");
            ROS_INFO("Width: %d, Height: %d", cameraInfo->width, cameraInfo->height);
            ROS_INFO("D[0] = %f, D[1] = %f, D[2] = %f, D[3] = %f, D[4] = %f", (double)cameraInfo->D[0], (double)cameraInfo->D[1], (double)cameraInfo->D[2], (double)cameraInfo->D[3], (double)cameraInfo->D[4]);
            ROS_INFO("P = [");
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[0], (double)cameraInfo->P[1], (double)cameraInfo->P[2], (double)cameraInfo->P[3]);
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[4], (double)cameraInfo->P[5], (double)cameraInfo->P[6], (double)cameraInfo->P[7]);
            ROS_INFO("    %f\t%f\t%f\t%f", (double)cameraInfo->P[8], (double)cameraInfo->P[9], (double)cameraInfo->P[10], (double)cameraInfo->P[11]);
            ROS_INFO("]");

            ROS_INFO("==================================");

            // Convert into intrinsics object - see https://github.com/intel-ros/realsense/blob/17b7279fb0a3bfe11bb162c0413f949d639b7a76/realsense_camera/src/base_nodelet.cpp#L682-L705
            //depth_intrin
    }
}

//*** Main ***//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imagegrab");
    ros::NodeHandle n;
    printf("READY to get image\n");
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/depth_registered/sw_registered/image_rect_raw", 1, imageCallback);

    ros::Subscriber camerainfo1_sub = n.subscribe<sensor_msgs::CameraInfo>
            ("/camera/rgb/camera_info", 10, camerainfo_cb);

    ros::Subscriber camerainfo2_sub = n.subscribe<sensor_msgs::CameraInfo>
            ("/camera/depth_registered/sw_registered/camera_info", 10, camerainfo2_cb);


    ros::spin();
    return 0;
}


/* Test script by using rosbag
    Terminal 1: roscore
    Terminal 2: rosbag play test.bag -l --clock -u 0.2 -s 9
*/




// Taken from https://github.com/IntelRealSense/librealsense/blob/master/include/librealsense/rsutil.h

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


