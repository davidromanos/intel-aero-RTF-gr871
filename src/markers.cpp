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

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


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
using namespace cv;

const float arucoSquareDimension = 0.0190f; //meters


void createArucoMarkers()
{
	Mat outputMarker;

	//Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	aruco::Dictionary markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
											

	for(int i = 0; i < 50; i++)
	{
		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
 		ostringstream convert;
		string imageName = "4x4Marker_";
		convert << imageName << i << ".jpg";
		imwrite(convert.str(), outputMarker);
	}

}


//int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension)
int startWebcamMonitoring()
{
	ros::Rate rate(20.0);	
	Mat frame;
	
	vector<int> markerIds;
	vector<vector<Point2f> > markerCorners, rejectedCandidates;
	aruco::DetectorParameters parameters;

	aruco::Dictionary markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

	VideoCapture vid(0);

	
	if(!vid.isOpened())
	{
		cout << "Error opening webcam stream" << endl;
		return -1;
	}

	vid.set(CAP_PROP_FRAME_WIDTH, 80);
	vid.set(CAP_PROP_FRAME_HEIGHT, 60);
	//namedWindow("Webcam", CV_WINDOW_NORMAL);
	//namedWindow("Webcam", 1);

	//vector<Vec3d> rotationVectors, translationVectors;

	while(ros::ok())
	{		
		if(!vid.read(frame)) {
			break;
		}

		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        	//aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimensions, calibrateCamera, rotationVectors, translationVectors);

        	aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        	//cout << markerCorners.size() << endl;
		for(int i = 0; i < markerIds.size(); i++)
		{
                	cout << "x coordinate =" << markerCorners[i][0].x << ";" << endl; //top left corner x coordinate
                	cout << "y coordinate =" << markerCorners[i][0].y << ";" << endl; //top left corner y coordinate
		}		
		
		imshow("Webcam", frame);
		cv::waitKey(1); // this is necessary to show the image in the view from OpenCV 3

		ros::spinOnce();
		rate.sleep();
	}

    return 1;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "markers");
    ros::NodeHandle n;
    printf("READY to get image\n");

    image_transport::ImageTransport it(n);

    namedWindow("Webcam", CV_WINDOW_KEEPRATIO);    
    startWindowThread();

  	cout << "OpenCV version: "
			<< CV_MAJOR_VERSION << "." 
			<< CV_MINOR_VERSION << "."
			<< CV_SUBMINOR_VERSION
			<< endl;

    startWebcamMonitoring();

    //ros::spin();

    destroyWindow("Webcam");
    return 0;
}
