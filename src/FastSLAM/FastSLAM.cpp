#include "FastSLAM.h"

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// Boost is needed for Gaussian random number generation
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <random>
#include <boost/filesystem.hpp>

#define USE_NUMERICAL_STABILIZED_KALMAN_FILTERS 0
#define ADD_LANDMARKS_AFTER_RESAMPLING 0

using namespace std;

float pi_to_pi(float ang)
{
    int n;

    if ( (ang < -2*M_PI) || (ang > 2*M_PI) ) {
        n   = floor(ang/(2*M_PI));
        ang = ang - n*(2*M_PI);
    }

    if (ang > M_PI)
        ang = ang - (2*M_PI);

    if (ang < -M_PI)
        ang = ang + (2*M_PI);

    return ang;
}

float pi_to_pi2(float ang)
{
    if (ang > M_PI)
        ang = ang - (2*M_PI);

    if (ang < -M_PI)
        ang = ang + (2*M_PI);

    return ang;
}

void KF_cholesky_update(Eigen::VectorXf &x, Eigen::MatrixXf &P, Eigen::VectorXf &v, Eigen::MatrixXf &R, Eigen::MatrixXf &H)
{
    Eigen::MatrixXf PHt = P*H.transpose();
    Eigen::MatrixXf S = H*PHt + R;

    // FIXME: why use conjugate()?
    S = (S+S.transpose()) * 0.5; //make symmetric
    Eigen::MatrixXf SChol = S.llt().matrixU();
    //SChol.transpose();
    //SChol.conjugate();


    Eigen::MatrixXf SCholInv = SChol.inverse(); //tri matrix
    Eigen::MatrixXf W1 = PHt * SCholInv;
    Eigen::MatrixXf W = W1 * SCholInv.transpose();

    x = x + W*v;
    P = P - W1*W1.transpose();
}


/* ############################## Defines measurement class ##############################  */

/* ############################## Defines GOTMeasurement class ##############################  */
GOTMeasurement::GOTMeasurement(unsigned int i, Eigen::Vector3f GOT_meas)
{
    c = i;
    z = GOT_meas;
    timestamp = ros::Time::now();
}

Eigen::VectorXf GOTMeasurement::MeasurementModel(VectorChiFastSLAMf pose, Eigen::Vector3f l)
{
    Eigen::Vector3f z;
    z << pose(0), pose(1), pose(2);
    z = z - l;
    return z;
}

Eigen::VectorXf GOTMeasurement::inverseMeasurementModel(VectorChiFastSLAMf pose)
{
    VectorChiFastSLAMf s = pose; // temp variable to make it look like equations    
    Eigen::Vector3f l;
    l << s(0), s(1), s(2);
    l = l - z;
    return l;
}

Eigen::MatrixXf GOTMeasurement::calculateHs(VectorChiFastSLAMf pose, Eigen::Vector3f l)
{
    Eigen::MatrixXf Hs(3, 4);
    Hs << Eigen::Matrix3f::Identity(3,3), Eigen::MatrixXf::Zero(3,1);
    //cout << " Hs" << Hs << endl;
    return Hs;
}

Eigen::MatrixXf GOTMeasurement::calculateHl(VectorChiFastSLAMf pose, Eigen::Vector3f l)
{
    //s = pose; // temp variable to make it look like equations
    Eigen::Matrix3f Hl = -1.0*Eigen::Matrix3f::Identity();
    return Hl;
};

Eigen::MatrixXf GOTMeasurement::getzCov(){
    return zCov;
}

Eigen::MatrixXf GOTMeasurement::zCov = 0.05*Eigen::Matrix3f::Identity(); // static variable - has to be declared outside class!


/* ############################## Defines ImgMeasurement class ##############################  */
float ImgMeasurement::ax = 0; // also known as fx
float ImgMeasurement::ay = 0; // also known as fy
float ImgMeasurement::x0 = 0; // also known as ppx
float ImgMeasurement::y0 = 0; // also known as ppy
/*ImgMeasurement::ImgMeasurement(unsigned int i, Eigen::Vector3f img_meas){
    pitch = 0;
    roll = 0;
    c = i;
    z = img_meas;
    timestamp = ros::Time::now();
}*/

ImgMeasurement::ImgMeasurement(unsigned int i, Eigen::Vector3f img_meas,float roll_, float pitch_){
    pitch = pitch_;
    roll = roll_;
    c = i;
    z = img_meas;
    timestamp = ros::Time::now();
}

Eigen::VectorXf ImgMeasurement::MeasurementModel(VectorChiFastSLAMf pose, Eigen::Vector3f l)
{
    Eigen::Vector3f z;

    float c_psi = cos(pose(3));
    float s_psi = sin(pose(3));
    float c_theta = cos(pitch);
    float s_theta = sin(pitch);
    float c_phi = cos(roll);
    float s_phi = sin(roll);

    Eigen::Matrix3f EB_R; // Rotation matrix corresponding to: EB_R    (transforming from drone to earth frame)
    EB_R  <<   (c_theta*c_psi), (c_psi*s_theta*s_phi - c_phi*s_psi), (s_phi*s_psi + c_phi*c_psi*s_theta),
               (c_theta*s_psi), (c_phi*c_psi + s_theta*s_phi*s_psi), (c_phi*s_theta*s_psi - c_psi*s_phi),
               (-s_theta),      (c_theta*s_phi),                     (c_theta*c_phi);

    Eigen::Vector3f CamCenter; // Camera center location in world coordinate
    CamCenter << pose(0), pose(1), pose(2);
    CamCenter = CamCenter + EB_R * CameraOffset;

    //cout << "landmark: " << l << endl;
    //cout << "pose: " << pose << endl;

    // Calculate world coordinate of landmark in the camera frame - Notice we use Roll-Pitch-Yaw angle convention
    float c_xl = (-c_psi*s_theta*s_phi + s_psi*c_phi)*(l(0) - CamCenter(0)) + (-s_psi*s_theta*s_phi-c_psi*c_phi)*(l(1) - CamCenter(1)) - (c_theta*s_phi)*(l(2) - CamCenter(2));
    float c_yl = (-c_psi*s_theta*c_phi-s_psi*s_phi)*(l(0) - CamCenter(0)) + (-s_psi*s_theta*c_phi+c_psi*s_phi)*(l(1) - CamCenter(1)) - (c_theta*c_phi)*(l(2) - CamCenter(2));
    float c_zl = (c_psi*c_theta)*(l(0) - CamCenter(0)) + (s_psi*c_theta)*(l(1) - CamCenter(1)) - s_theta*(l(2) - CamCenter(2));

    //cout << "World coordinate: " << c_xl << ", " << c_yl << ", " << c_zl << endl;

    // Project world coordinate onto image plane
    float xi = (ax*c_xl + x0*c_zl) / c_zl;
    float yi = (ay*c_yl + y0*c_zl) / c_zl;
    float zc = c_zl;

    // Form the measurement vector
    z << xi,
         yi,
         zc;

    return z;
}

Eigen::VectorXf ImgMeasurement::inverseMeasurementModel(VectorChiFastSLAMf pose)
{
    float c_psi = cos(pose(3));
    float s_psi = sin(pose(3));
    float c_theta = cos(pitch);
    float s_theta = sin(pitch);
    float c_phi = cos(roll);
    float s_phi = sin(roll);

    Eigen::Matrix3f R; // Rotation matrix corresponding to: BC_R' * EB_R'    (transforming from earth to drone/camera frame)
    R  <<   (-c_psi*s_theta*s_phi + s_psi*c_phi),(-s_psi*s_theta*s_phi-c_psi*c_phi), -(c_theta*s_phi),
            (-c_psi*s_theta*c_phi-s_psi*s_phi),  (-s_psi*s_theta*c_phi+c_psi*s_phi), -(c_theta*c_phi),
            (c_psi*c_theta),                     (s_psi*c_theta),                    -(s_theta);

    Eigen::Matrix3f EB_R; // Rotation matrix corresponding to: EB_R    (transforming from drone to earth frame)
    EB_R  <<   (c_theta*c_psi), (c_psi*s_theta*s_phi - c_phi*s_psi), (s_phi*s_psi + c_phi*c_psi*s_theta),
               (c_theta*s_psi), (c_phi*c_psi + s_theta*s_phi*s_psi), (c_phi*s_theta*s_psi - c_psi*s_phi),
               (-s_theta),      (c_theta*s_phi),                     (c_theta*c_phi);

    float c_zl = z(2);
    float c_xl = (z(0)*c_zl - x0*c_zl) / ax;
    float c_yl = (z(1)*c_zl - y0*c_zl) / ay;

    Eigen::Vector3f CamLandmark;
    CamLandmark << c_xl, c_yl, c_zl;

    Eigen::Vector3f pose_xyz;
    pose_xyz << pose(0), pose(1), pose(2);

    Eigen::Vector3f TempLandmark = CamLandmark + R*pose_xyz;

    Eigen::Vector3f WorldLandmark = R.transpose() * TempLandmark + EB_R * CameraOffset; // rot.transpose() corresponds to EB_R * BC_R     (transforming from drone/camera to earth frame)

    return WorldLandmark;
}

Eigen::MatrixXf ImgMeasurement::calculateHs(VectorChiFastSLAMf pose, Eigen::Vector3f l)
{
    Eigen::MatrixXf Hs(3, 4);

    float c_psi = cos(pose(3));
    float s_psi = sin(pose(3));
    float c_theta = cos(pitch);
    float s_theta = sin(pitch);
    float c_phi = cos(roll);
    float s_phi = sin(roll);

    /*float den = powf((c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1))),2);

    Hs(0,0) = (ax*(c_phi*s_psi - c_psi*s_theta*s_phi))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1))) + (ax*c_theta*c_psi*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2))))/den;
    Hs(0,1) = (ax*c_theta*s_psi*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2))))/den - (ax*(c_phi*c_psi + s_theta*s_phi*s_psi))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(0,2) = - (ax*s_theta*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2))))/den - (ax*c_theta*s_phi)/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(0,3) = -(ax*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(0,4) = - (ax*(c_theta*c_psi*s_phi*(pose(0) - l(0)) - s_theta*s_phi*(pose(2) - l(2)) + c_theta*s_phi*s_psi*(pose(1) - l(1))))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1))) - (ax*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2)))*(c_theta*(pose(2) - l(2)) + c_psi*s_theta*(pose(0) - l(0)) + s_theta*s_psi*(pose(1) - l(1))))/den;
    Hs(0,5) = (ax*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(0) - l(0)) + (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(1) - l(1))))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1))) + (ax*(c_theta*c_psi*(pose(1) - l(1)) - c_theta*s_psi*(pose(0) - l(0)))*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2))))/den;
    Hs(1,0) = (ay*c_theta*c_psi*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/den - (ay*(s_phi*s_psi + c_phi*c_psi*s_theta))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(1,1) = (ay*(c_psi*s_phi - c_phi*s_theta*s_psi))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1))) + (ay*c_theta*s_psi*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/den;
    Hs(1,2) = - (ay*s_theta*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/den - (ay*c_theta*c_phi)/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(1,3) = (ay*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2))))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(1,4) = - (ay*(c_theta*c_phi*c_psi*(pose(0) - l(0)) - c_phi*s_theta*(pose(2) - l(2)) + c_theta*c_phi*s_psi*(pose(1) - l(1))))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1))) - (ay*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2)))*(c_theta*(pose(2) - l(2)) + c_psi*s_theta*(pose(0) - l(0)) + s_theta*s_psi*(pose(1) - l(1))))/den;
    Hs(1,5) = (ay*(c_theta*c_psi*(pose(1) - l(1)) - c_theta*s_psi*(pose(0) - l(0)))*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/den - (ay*((c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(0) - l(0)) + (s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(1) - l(1))))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(2,0) = -c_theta*c_psi;
    Hs(2,1) = -c_theta*s_psi;
    Hs(2,2) = s_theta;
    Hs(2,3) = 0;
    Hs(2,4) = c_theta*(pose(2) - l(2)) + c_psi*s_theta*(pose(0) - l(0)) + s_theta*s_psi*(pose(1) - l(1));
    Hs(2,5) = c_theta*s_psi*(pose(0) - l(0)) - c_theta*c_psi*(pose(1) - l(1));
    */

    /*Hs(0,0) = (ax*(c_phi*s_psi - c_psi*s_theta*s_phi))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1))) + (ax*c_theta*c_psi*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2))))/den;
    Hs(0,1) = (ax*c_theta*s_psi*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2))))/den - (ax*(c_phi*c_psi + s_theta*s_phi*s_psi))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(0,2) = - (ax*s_theta*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2))))/den - (ax*c_theta*s_phi)/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(0,3) = (ax*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(0) - l(0)) + (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(1) - l(1))))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1))) + (ax*(c_theta*c_psi*(pose(1) - l(1)) - c_theta*s_psi*(pose(0) - l(0)))*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2))))/den;
    Hs(1,0) = (ay*c_theta*c_psi*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/den - (ay*(s_phi*s_psi + c_phi*c_psi*s_theta))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(1,1) = (ay*(c_psi*s_phi - c_phi*s_theta*s_psi))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1))) + (ay*c_theta*s_psi*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/den;
    Hs(1,2) = - (ay*s_theta*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/den - (ay*c_theta*c_phi)/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(1,3) = (ay*(c_theta*c_psi*(pose(1) - l(1)) - c_theta*s_psi*(pose(0) - l(0)))*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/den - (ay*((c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(0) - l(0)) + (s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(1) - l(1))))/(c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1)));
    Hs(2,0) = -c_theta*c_psi;
    Hs(2,1) = -c_theta*s_psi;
    Hs(2,2) = s_theta;
    Hs(2,3) = c_theta*s_psi*(pose(0) - l(0)) - c_theta*c_psi*(pose(1) - l(1));*/

    float den = (CameraOffset(0) - s_theta*(pose(2) - l(2)) + c_theta*c_psi*(pose(0) - l(0)) + c_theta*s_psi*(pose(1) - l(1)));
    float denPow2 = pow(den,2);

    Hs(0,0) = (ax*(c_phi*s_psi - c_psi*s_theta*s_phi))/den + (ax*c_theta*c_psi*(CameraOffset(1) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + (c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) + c_theta*s_phi*(pose(2) - l(2))))/denPow2;
    Hs(0,1) = (ax*c_theta*s_psi*(CameraOffset(1) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + (c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) + c_theta*s_phi*(pose(2) - l(2))))/denPow2 - (ax*(c_phi*c_psi + s_theta*s_phi*s_psi))/den;
    Hs(0,2) = - (ax*s_theta*(CameraOffset(1) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + (c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) + c_theta*s_phi*(pose(2) - l(2))))/denPow2 - (ax*c_theta*s_phi)/den;
    //Hs(0,3) = -(ax*((s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/den;
    //Hs(0,4) = - (ax*(c_theta*c_psi*s_phi*(pose(0) - l(0)) - s_theta*s_phi*(pose(2) - l(2)) + c_theta*s_phi*s_psi*(pose(1) - l(1))))/den - (ax*(c_theta*(pose(2) - l(2)) + c_psi*s_theta*(pose(0) - l(0)) + s_theta*s_psi*(pose(1) - l(1)))*(CameraOffset(1) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + (c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) + c_theta*s_phi*(pose(2) - l(2))))/denPow2;
    Hs(0,3) = (ax*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(0) - l(0)) + (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(1) - l(1))))/den + (ax*(c_theta*c_psi*(pose(1) - l(1)) - c_theta*s_psi*(pose(0) - l(0)))*(CameraOffset(1) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + (c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) + c_theta*s_phi*(pose(2) - l(2))))/denPow2;

    Hs(1,0) = (ay*c_theta*c_psi*(CameraOffset(2) + (s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/denPow2 - (ay*(s_phi*s_psi + c_phi*c_psi*s_theta))/den;
    Hs(1,1) = (ay*(c_psi*s_phi - c_phi*s_theta*s_psi))/den + (ay*c_theta*s_psi*(CameraOffset(2) + (s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/denPow2;
    Hs(1,2) = - (ay*s_theta*(CameraOffset(2) + (s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/denPow2 - (ay*c_theta*c_phi)/den;
    //Hs(1,3) = (ay*((c_phi*c_psi + s_theta*s_phi*s_psi)*(pose(1) - l(1)) - (c_phi*s_psi - c_psi*s_theta*s_phi)*(pose(0) - l(0)) + c_theta*s_phi*(pose(2) - l(2))))/den;
    //Hs(1,4) = - (ay*(c_theta*c_phi*c_psi*(pose(0) - l(0)) - c_phi*s_theta*(pose(2) - l(2)) + c_theta*c_phi*s_psi*(pose(1) - l(1))))/den - (ay*(c_theta*(pose(2) - l(2)) + c_psi*s_theta*(pose(0) - l(0)) + s_theta*s_psi*(pose(1) - l(1)))*(CameraOffset(2) + (s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/denPow2;
    Hs(1,3) = (ay*(c_theta*c_psi*(pose(1) - l(1)) - c_theta*s_psi*(pose(0) - l(0)))*(CameraOffset(2) + (s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(0) - l(0)) - (c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(1) - l(1)) + c_theta*c_phi*(pose(2) - l(2))))/denPow2 - (ay*((c_psi*s_phi - c_phi*s_theta*s_psi)*(pose(0) - l(0)) + (s_phi*s_psi + c_phi*c_psi*s_theta)*(pose(1) - l(1))))/den;

    Hs(2,0) = -c_theta*c_psi;
    Hs(2,1) = -c_theta*s_psi;
    Hs(2,2) = s_theta;
    //Hs(2,3) = 0;
    //Hs(2,4) = c_theta*(pose(2) - l(2)) + c_psi*s_theta*(pose(0) - l(0)) + s_theta*s_psi*(pose(1) - l(1));
    Hs(2,3) = c_theta*s_psi*(pose(0) - l(0)) - c_theta*c_psi*(pose(1) - l(1));

    return Hs;
}

Eigen::MatrixXf ImgMeasurement::calculateHl(VectorChiFastSLAMf pose, Eigen::Vector3f l)
{
    float c_psi = cos(pose(3));
    float s_psi = sin(pose(3));
    float c_theta = cos(pitch);
    float s_theta = sin(pitch);
    float c_phi = cos(roll);
    float s_phi = sin(roll);

    Eigen::Matrix3f R; // Rotation matrix corresponding to: BC_R' * EB_R'
    R  <<   (-c_psi*s_theta*s_phi + s_psi*c_phi),(-s_psi*s_theta*s_phi-c_psi*c_phi), -(c_theta*s_phi),
            (-c_psi*s_theta*c_phi-s_psi*s_phi),  (-s_psi*s_theta*c_phi+c_psi*s_phi), -(c_theta*c_phi),
            (c_psi*c_theta),                     (s_psi*c_theta),                    -(s_theta);

    Eigen::Matrix3f Hl;

    /*float den1 = powf((R(2,0)*(pose(0) - l(0)) + R(2,1)*(pose(1) - l(1)) + R(2,2)*(pose(2) - l(2))),2);
    float den2 = (R(2,0)*(pose(0) - l(0)) + R(2,1)*(pose(1) - l(1)) + R(2,2)*(pose(2) - l(2)));*/

    //cout << "den1: " << den1 << " - den2: " << den2 << endl;

    /*Hl(0,0) = (R(2,0)*ax*(R(0,0)*(pose(0) - l(0)) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/den1 - (R(0,0)*ax)/den2;
    Hl(1,0) = (R(2,0)*ay*(R(1,0)*(pose(0) - l(0)) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/den1 - (R(1,0)*ay)/den2;
    Hl(2,0) = R(2,0);
    Hl(0,1) = (R(2,1)*ax*(R(0,0)*(pose(0) - l(0)) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/den1 - (R(0,1)*ax)/den2;
    Hl(1,1) = (R(2,1)*ay*(R(1,0)*(pose(0) - l(0)) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/den1 - (R(1,1)*ay)/den2;
    Hl(2,1) = R(2,1);
    Hl(0,2) = (R(2,2)*ax*(R(0,0)*(pose(0) - l(0)) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/den1 - (R(0,2)*ax)/den2;
    Hl(1,2) = (R(2,2)*ay*(R(1,0)*(pose(0) - l(0)) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/den1 - (R(1,2)*ay)/den2;
    Hl(2,2) = R(2,2);*/

    /*Hl(0,0) = (R(2,0)*ax*(R(0,0)*(pose(0) - l(0)) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/den1 - (R(0,0)*ax)/den2;
    Hl(1,0) = (R(2,0)*ay*(R(1,0)*(pose(0) - l(0)) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/den1 - (R(1,0)*ay)/den2;
    Hl(2,0) = R(2,0);
    Hl(0,1) = (R(2,1)*ax*(R(0,0)*(pose(0) - l(0)) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/den1 - (R(0,1)*ax)/den2;
    Hl(1,1) = (R(2,1)*ay*(R(1,0)*(pose(0) - l(0)) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/den1 - (R(1,1)*ay)/den2;
    Hl(2,1) = R(2,1);
    Hl(0,2) = (R(2,2)*ax*(R(0,0)*(pose(0) - l(0)) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/den1 - (R(0,2)*ax)/den2;
    Hl(1,2) = (R(2,2)*ay*(R(1,0)*(pose(0) - l(0)) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/den1 - (R(1,2)*ay)/den2;
    Hl(2,2) = R(2,2);*/

    float den = (CameraOffset(0) + R(2,0)*(pose(0) - l(0)) + R(2,1)*(pose(1) - l(1)) + R(2,2)*(pose(2) - l(2)));
    float denPow2 = pow(den,2);

    Hl(0,0) = (R(2,0)*ax*(R(0,0)*(pose(0) - l(0)) - CameraOffset(1) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/denPow2 - (R(0,0)*ax)/den;
    Hl(1,0) = (R(2,0)*ay*(R(1,0)*(pose(0) - l(0)) - CameraOffset(2) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/denPow2 - (R(1,0)*ay)/den;
    Hl(2,0) = R(2,0);
    Hl(0,1) = (R(2,1)*ax*(R(0,0)*(pose(0) - l(0)) - CameraOffset(1) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/denPow2 - (R(0,1)*ax)/den;
    Hl(1,1) = (R(2,1)*ay*(R(1,0)*(pose(0) - l(0)) - CameraOffset(2) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/denPow2 - (R(1,1)*ay)/den;
    Hl(2,1) = R(2,1);
    Hl(0,2) = (R(2,2)*ax*(R(0,0)*(pose(0) - l(0)) - CameraOffset(1) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/denPow2 - (R(0,2)*ax)/den;
    Hl(1,2) = (R(2,2)*ay*(R(1,0)*(pose(0) - l(0)) - CameraOffset(2) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/denPow2 - (R(1,2)*ay)/den;
    Hl(2,2) = R(2,2);

    return Hl;
}

Eigen::MatrixXf ImgMeasurement::getzCov(){
    /*Eigen::Matrix3f cov;
    cov << 5, 0, 0,
            0, 5, 0,
            0, 0, 0.5;*/
    return zCov;
}

Eigen::MatrixXf ImgMeasurement::zCov = 0.1*Eigen::Matrix3f::Identity(); // static variable - has to be declared outside class!
Eigen::Vector3f ImgMeasurement::CameraOffset = Eigen::Vector3f::Zero(); // static variable - has to be declared outside class!




/* ############################## Defines measurement set class ##############################  */
MeasurementSet::MeasurementSet(){
    firstMeasNode = NULL;
    nMeas = 0;
    //cout << "n0: " << nMeas << endl;
}

MeasurementSet::MeasurementSet(Measurement *meas){
    firstMeasNode = new Node_MeasurementSet;
    firstMeasNode->meas = meas;
    firstMeasNode->nextNode = NULL;
    nMeas = 1;
    firstMeasNode->measIdentifier = nMeas;
    //cout << "n1: " << nMeas << endl;
}

MeasurementSet::~MeasurementSet(){
    //deleteMeasurementSet();
}

void MeasurementSet::emptyMeasurementSet(){
    if(firstMeasNode != NULL){
        if(firstMeasNode->nextNode != NULL){
            emptyMeasurementSet(firstMeasNode->nextNode);
        }
        if (firstMeasNode->meas != NULL)
            delete firstMeasNode->meas;
        delete firstMeasNode;
        nMeas = 0;
        firstMeasNode = NULL;
    }
}

void MeasurementSet::emptyMeasurementSet(Node_MeasurementSet *MeasNode){
    if(MeasNode->nextNode != NULL){
        emptyMeasurementSet(MeasNode->nextNode);
    }
    delete MeasNode->meas;
    delete MeasNode;
    nMeas--;
}

void MeasurementSet::addMeasurement(Measurement *meas){
    if (firstMeasNode == NULL){
        //cout << "Adding measurement to empty set (firstMeasNode)" << endl;
        firstMeasNode = new Node_MeasurementSet;
        firstMeasNode->meas = meas;
        firstMeasNode->nextNode = NULL;
        nMeas = 1;
        firstMeasNode->measIdentifier = nMeas;
    }
    else{
        //cout << "Adding measurement to set" << endl;
        Node_MeasurementSet* tmp_pointer = firstMeasNode;
        while(tmp_pointer->nextNode != NULL){
            tmp_pointer = tmp_pointer->nextNode;
        }
        tmp_pointer->nextNode = new Node_MeasurementSet;
        tmp_pointer->nextNode->meas = meas;
        tmp_pointer->nextNode->nextNode = NULL;
        nMeas++;
        tmp_pointer->nextNode->measIdentifier = nMeas;
    }
}

int MeasurementSet::countNumberOfMeasurements(){

    if (firstMeasNode==NULL){
        nMeas = 0;
        return 0;
    }
    else{
        int i = 1;
        Node_MeasurementSet* tmp_pointer = firstMeasNode;
        while(tmp_pointer->nextNode != NULL){
            tmp_pointer = tmp_pointer->nextNode;
            i++;
        }
        nMeas = i;
        return i;
    }

}

int MeasurementSet::getNumberOfMeasurements(){
    return nMeas;
}

Measurement* MeasurementSet::getMeasurement(int i){
    Node_MeasurementSet* tmp_measNodePointer = firstMeasNode;

    while (tmp_measNodePointer->measIdentifier != i){
        tmp_measNodePointer = tmp_measNodePointer->nextNode;
    }

    return tmp_measNodePointer->meas;
}





/* ############################## Defines Maps class ##############################  */

unsigned int landmark::globalLandmarkCounter; // can be used to check if number of landmarks does not grow without bound
unsigned int mapNode::globalMapNodeCounter; // can be used to check if number of MapNodes does not grow without bound
int MapTree::mapTreeIdentifierCounter = 1;

MapTree::MapTree(const MapTree &MapToCopy)
{
    mapTreeIdentifier = mapTreeIdentifierCounter;

    mapTreeIdentifierCounter++;
    root = MapToCopy.root;
    // we add new reference for a mapNode and have to increment its reference counter
    if (MapToCopy.root != NULL){
        MapToCopy.root->referenced++;
    }
    N_Landmarks = MapToCopy.N_Landmarks;
    N_layers = MapToCopy.N_layers;
    N_nodes = MapToCopy.N_nodes;
}

MapTree::MapTree()
{
    mapTreeIdentifier = mapTreeIdentifierCounter;
    mapTreeIdentifierCounter++;
  root=NULL;
  N_Landmarks = 0;
  N_layers = 0;
  N_nodes = 0;
}

MapTree::~MapTree()
{
    if (root != NULL){
        //cout << "deleting MapTree: " << mapTreeIdentifier << " References to root: " << root->referenced << " Debugging: ";
        removeReferenceToSubTree(root);
        //cout << endl;
    }
}

void MapTree::removeReferenceToSubTree(mapNode* nodeToStartFrom){
    if ((nodeToStartFrom != NULL) && nodeToStartFrom->referenced != 0){
        nodeToStartFrom->referenced--;
    }
    if(nodeToStartFrom->referenced < 1){ // we have to delete the node! since the nodeToStartFrom
        //cout << "D50 ";
        //if( (nodeToStartFrom->left != NULL) && (nodeToStartFrom->left->referenced <= 1)){
        if(nodeToStartFrom->left != NULL){
            //cout << "D51 ";
            removeReferenceToSubTree(nodeToStartFrom->left);
        }
        //if( (nodeToStartFrom->right != NULL) && (nodeToStartFrom->right->referenced <= 1)){
        if(nodeToStartFrom->right != NULL){
            //cout << "D52 ";
            removeReferenceToSubTree(nodeToStartFrom->right);
        }
        if (nodeToStartFrom->key_value == 0){ // we are at a leaf node and have to delete the landmark
            //cout << "D53 ";
            delete nodeToStartFrom->l;
        }
        //cout << "D55 ";
        delete nodeToStartFrom;
        nodeToStartFrom = NULL;
    }
    else{ // we should not delete the node!
        //cout << "D54 ";
    }
}

void MapTree::insertLandmark(landmark* newLandmark){
 /*   if(N_Landmarks==0){
        root = new mapNode;
        root->key_value = 0;
        root->left=NULL;
        root->right=NULL;
        root->l = newLandmark;
        root->referenced = 1;
        N_layers = 0;
        N_nodes = 0;
        //cout << "D1: N" << N_nodes << " keyvalue: " << root->key_value << endl;
    }
    else{*/

        if (newLandmark->c == 1){ // handle special case
            int Needed_N_layers = 1;
            if (Needed_N_layers>N_layers){
                creatNewLayers(Needed_N_layers);
            }
        }
        else{
            float c_tmp = (float)newLandmark->c;
            int Needed_N_layers = (int)ceil(log2(c_tmp));

            if (Needed_N_layers>N_layers){
                creatNewLayers(Needed_N_layers);
            }
        }

        mapNode* tmpMapNodePointer = root;
        int i = N_layers;
        unsigned int i2 = (tmpMapNodePointer->key_value)/2;

        while (i>1){ //when i = 1 we have reached the bottom of the tree
            if(newLandmark->c > tmpMapNodePointer->key_value){ // we go to the right
                if(tmpMapNodePointer->right != NULL){
                    tmpMapNodePointer=tmpMapNodePointer->right;
                    //cout << "D:R" << endl;
                }
                else{ //tmpMapNodePointer->right != NULL does not point to anything we have to creat a new node!
                    tmpMapNodePointer->right = new mapNode;
                    tmpMapNodePointer->right->key_value = tmpMapNodePointer->key_value + i2;
                    tmpMapNodePointer->right->left = NULL;
                    tmpMapNodePointer->right->right = NULL;
                    tmpMapNodePointer->right->l = NULL;
                    tmpMapNodePointer->right->referenced = 1;
                    N_nodes++;
                    //cout << "D2: N" << N_nodes << " keyvalue: " << tmpMapNodePointer->right->key_value <<endl;

                    tmpMapNodePointer=tmpMapNodePointer->right;
                }
            }
            else if(newLandmark->c <= tmpMapNodePointer->key_value){
                if(tmpMapNodePointer->left != NULL){
                    tmpMapNodePointer=tmpMapNodePointer->left;
                    //cout << "D:L" << endl;
                }
                else{ //tmpMapNodePointer->right != NULL does not point to anything we have to creat a new node!
                    tmpMapNodePointer->left = new mapNode;
                    tmpMapNodePointer->left->key_value = tmpMapNodePointer->key_value - i2;
                    tmpMapNodePointer->left->left = NULL;
                    tmpMapNodePointer->left->right = NULL;
                    tmpMapNodePointer->left->l = NULL;
                    tmpMapNodePointer->left->referenced = 1;
                    N_nodes++;
                    //cout << "D3: N" << N_nodes << " keyvalue: " << tmpMapNodePointer->left->key_value << endl;

                    tmpMapNodePointer=tmpMapNodePointer->left;
                }
            }
            else{cout << "Error in insertion of landmark in map" << endl;}

            i--;
            i2 = (unsigned int)i2/2;
        }
        // we are now at layer one at the bottom of the tree, and have to create a new leaf node to hold a pointer for the measurement!
        tmpMapNodePointer->l=newLandmark;

        mapNode* pointerForNewLeafNode = new mapNode;
        pointerForNewLeafNode->key_value = 0;
        pointerForNewLeafNode->left = NULL;
        pointerForNewLeafNode->right = NULL;
        pointerForNewLeafNode->l = newLandmark;
        pointerForNewLeafNode->referenced = 1;

        if(newLandmark->c > tmpMapNodePointer->key_value){ // we go to the right
            tmpMapNodePointer->right = pointerForNewLeafNode;
            //cout << "D5: Created new leaf to the right!" << endl;
        }
        else if(newLandmark->c <= tmpMapNodePointer->key_value){ // we go to the left
            tmpMapNodePointer->left = pointerForNewLeafNode;
            //cout << "D5: Created new leaf to the left!" << endl;
        }
        else{cout << "Error in insertion of landmark in map" << endl;}
    //}
N_Landmarks++;
}


void MapTree::creatNewLayers(int Needed_N_layers){
     int missinLayers = Needed_N_layers-N_layers;
     int i = 1;
     while (i <= missinLayers){
         mapNode* newRootNode = new mapNode;
         newRootNode->key_value = (int)pow(2,(N_layers+i)-1);
         newRootNode->left=root;
         newRootNode->right=NULL;
         newRootNode->l = NULL;
         newRootNode->referenced = 1;
         N_nodes++;
         //cout << "D4: N" << N_nodes << " keyvalue: " << newRootNode->key_value << endl;

         root = newRootNode;
         i++;
     }

     //cout << "Added layer. N_layers:" << Needed_N_layers << endl;
     N_layers = Needed_N_layers;
}

int MapTree::countNLayers(){
    int i = 0;
    mapNode* tmpMapNode = root;
    while (tmpMapNode->left != NULL){ // utilize the fact that we always have landmark 1 (l_GOT)
        tmpMapNode = tmpMapNode->left;
        i++;
    }
    return i;
}

landmark* MapTree::extractLandmarkNodePointer(unsigned int Landmark_identifier){

    mapNode* tmpNodePointer = root;

    while(tmpNodePointer->key_value != 0){

        if(Landmark_identifier > tmpNodePointer->key_value){ // we go left

            tmpNodePointer = tmpNodePointer->right;
        }
        else{

            tmpNodePointer = tmpNodePointer->left; // we go rigth
        }

    }

    return tmpNodePointer->l;
}

void MapTree::correctLandmark(landmark* newLandmarkData){
    mapNode* tmpMapNode = makeNewPath(newLandmarkData, root);
    removeReferenceToSubTree(root);
    root = tmpMapNode;
}

mapNode* MapTree::makeNewPath(landmark* newLandmarkData, mapNode* startNode){
    //cout << "D210 - keyvalue: " << startNode->key_value << endl;
    if(startNode->key_value > 0){
        // we need to make a new MapNode
        mapNode* pointerForNewMapNode = new mapNode;
        pointerForNewMapNode->l = NULL;
        pointerForNewMapNode->referenced = 1;

        if (newLandmarkData->c > startNode->key_value){ // we go right
            pointerForNewMapNode->left = startNode->left; // and do not change the left pointer
            if(pointerForNewMapNode->left != NULL){
                pointerForNewMapNode->left->referenced++;
            }
            pointerForNewMapNode->key_value = startNode->key_value; // the new node has the same key_value as the old
            pointerForNewMapNode->right = makeNewPath(newLandmarkData,startNode->right);

            //removeReferenceToSubTree(startNode); // we have to delete the SubTree if there is no more references for it!
        }
        else if(newLandmarkData->c <= startNode->key_value){ // we go left
            pointerForNewMapNode->right = startNode->right; // and do not change the right pointer
            if(pointerForNewMapNode->right != NULL){
                pointerForNewMapNode->right->referenced++;
            }
            pointerForNewMapNode->key_value = startNode->key_value; // the new node has the same key_value as the old
            pointerForNewMapNode->left = makeNewPath(newLandmarkData,startNode->left);
        }
        else{
            cout << "error in makeNewPath";
        }
        return pointerForNewMapNode;
    }
    else{ // we have reached the bottom of the tree and should make a new mapNode to hold the pointer for the updated landmark data
        //cout << "D26 ";
        mapNode* pointerForNewLeafNode = new mapNode;
        pointerForNewLeafNode->key_value = 0;
        pointerForNewLeafNode->left = NULL;
        pointerForNewLeafNode->right = NULL;
        pointerForNewLeafNode->l = newLandmarkData;
        pointerForNewLeafNode->referenced = 1;

        //removeReferenceToSubTree(startNode); // we have to delete the SubTree if there is no more references for it!
        //cout << "D27 ";
        return pointerForNewLeafNode;
    }
    //cout << "D28 ";
}

void MapTree::printAllLandmarkPositions(){
    //cout << endl << "D30 ";

    for(unsigned int i = 1;i<=N_Landmarks;i++){
        //cout << "D31 ";
        if (extractLandmarkNodePointer(i) != NULL){
            //cout << "D32 ";
             //cout << "l_" << i <<": "<< extractLandmarkNodePointer(i)->lhat.transpose() << endl;
        }
        else{
            //tmpMatrix.col(i-1) = Eigen::Vector3f::Zero();
            cout<<"Error: NULL pointer!";
        }
    }
    //cout << "D33 ";
}



/* ############################## Defines Path class ##############################  */
Eigen::IOFormat Path::OctaveFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
std::ofstream Path::dataFileStream;

Path::Path(VectorChiFastSLAMf S, unsigned int k){
    Node_Path* firstPathNode = new Node_Path;
    firstPathNode->S = S;
    firstPathNode->k = k;
    firstPathNode->Ts = 0;
    firstPathNode->nextNode = NULL;

    PathRoot = new Node_Path;
    PathRoot->nextNode = firstPathNode;
    firstPathNode->referenced = 1;

    PathLength = 1;
}

Path::Path(const Path &PathToCopy){
    PathRoot = new Node_Path;
    PathRoot->nextNode = PathToCopy.PathRoot->nextNode;
    PathToCopy.PathRoot->nextNode->referenced++;

    PathLength = 1;
}

Path::~Path(){
    deletePath();
}

void Path::deletePath(){
    if(PathRoot->nextNode != NULL){
        deletePath(PathRoot->nextNode);
    }
    PathRoot=NULL;
}

void Path::deletePath(Node_Path *PathNode){
    if(PathNode->referenced > 1){
        PathNode->referenced--;
    }
    else if (PathNode->nextNode != NULL){
        deletePath(PathNode->nextNode);
        delete PathNode;
    }
    PathLength--;
}

void Path::addPose(VectorChiFastSLAMf S, unsigned int k, float Ts){

    Node_Path* tmpPathNode = new Node_Path;
    tmpPathNode->S = S;
    tmpPathNode->k = k;
    tmpPathNode->Ts = Ts + PathRoot->nextNode->Ts;
    tmpPathNode->referenced = 0;
    tmpPathNode->nextNode = PathRoot->nextNode;

    PathRoot->nextNode = tmpPathNode;
    tmpPathNode->referenced++;

    PathLength++;
}

unsigned int Path::countLengthOfPath(){

    if (PathRoot==NULL){
        PathLength = 0;
        return PathLength;
    }
    else{
        unsigned int i = 1;
        Node_Path* tmp_pointer = PathRoot->nextNode;
        while(tmp_pointer->nextNode != NULL){
            tmp_pointer = tmp_pointer->nextNode;
            i++;
        }
        PathLength = i;
        return PathLength;
    }
}

VectorChiFastSLAMf* Path::getPose(){
    // return latest pose!
    return &(PathRoot->nextNode->S);
}

VectorChiFastSLAMf* Path::getPose(unsigned int k){
    // returns specific pose!
    //cout << "D10" << endl;
    if (PathRoot==NULL){
        //cout << "D11" << endl;
        return NULL;
    }
    else{
        //cout << "D12" << endl;

        Node_Path* tmp_pointer = PathRoot->nextNode;

        while(tmp_pointer->k != k){
            //cout << tmp_pointer->k << ","; // for debugging
            if (tmp_pointer->nextNode != NULL){
                tmp_pointer = tmp_pointer->nextNode;
            }
            else{
                return NULL;
            }
        }
        return &(tmp_pointer->S);
    }
}


/* ############################## Defines particle class ##############################  */
Particle::Particle(unsigned int GOT_ID, VectorChiFastSLAMf s0, MatrixChiFastSLAMf s_0_Cov, unsigned int k)   // default Constructor definition
{
    s = new Path(s0,k); // makes new path!
    map = new MapTree; // makes new mapTree
    w = 1;
    s_k_Cov = s_0_Cov;

    landmark* li = new landmark;
    li->c = GOT_ID;
    li->lhat = Eigen::Vector3f::Zero();
    li->lCov = Eigen::Matrix3f::Zero();
    map->insertLandmark(li);
}

Particle::Particle(const Particle &ParticleToCopy)   // Copy Constructor
{
    //cout << "Copying particle" << endl;
    s = new Path(*(ParticleToCopy.s)); //makes copy of s on the heap
    map = new MapTree(*(ParticleToCopy.map));
    w = ParticleToCopy.w;
    s_k_Cov = ParticleToCopy.s_k_Cov;
}

Particle::~Particle()
{
    //cout << "Deleting particle" << endl;
    delete s; // call destructor of s
    delete map; // call destructor of map
}

void Particle::updateParticle(MeasurementSet* z_Ex,MeasurementSet* z_New,VectorUFastSLAMf* u, unsigned int k, float Ts)
{
    VectorChiFastSLAMf s_proposale;

    if (k > 5){

        s_proposale = drawSampleFromProposaleDistribution(s->getPose(),u,z_Ex,Ts);

        s->addPose(s_proposale,k, Ts); // we are done estimating our pose and add it to the path!

        // OBS. In this code the importance weight is calculated differently and before the landmark corrections are done: https://github.com/bushuhui/fastslam/blob/master/src/fastslam_2.cpp#L593-L602
        if (z_Ex != NULL && z_Ex->nMeas != 0 ){

            calculateImportanceWeight(z_Ex,s_proposale);
        }

        updateLandmarkEstimates(s_proposale,z_Ex,z_New);


    }
    else{

        s_proposale = *(s->getPose());

        s->addPose(s_proposale,k, Ts); // we are done estimating our pose and add it to the path!

        updateLandmarkEstimates(s_proposale,NULL,z_New);

    }

}

void Particle::updateLandmarkEstimates(VectorChiFastSLAMf s_proposale, MeasurementSet* z_Ex, MeasurementSet* z_New){

    handleExMeas(z_Ex,s_proposale);

#if !ADD_LANDMARKS_AFTER_RESAMPLING
    handleNewMeas(z_New,s_proposale);
#endif

    //cout << "N_landmarks in map after update: " << map->N_Landmarks << endl;
}

void Particle::handleExMeas(MeasurementSet* z_Ex, VectorChiFastSLAMf s_proposale){
    if (z_Ex != NULL && z_Ex->nMeas != 0 ){
        //cout << "nMeas in z_Ex: " << z_Ex->nMeas << endl;

        for( int i = 1; i < z_Ex->nMeas; i = i + 1 ) {
            Measurement* z_tmp = z_Ex->getMeasurement(i);
            landmark* li_old = map->extractLandmarkNodePointer(z_tmp->c);

            Eigen::VectorXf z_hat = z_tmp->MeasurementModel(s_proposale,li_old->lhat); // (3.33)
            Eigen::VectorXf v = z_tmp->z - z_hat;

            Eigen::MatrixXf Hl;
            Hl = z_tmp->calculateHl(s_proposale,li_old->lhat);  // (3.34)

            //li_old->lhat = xfi
            //li_old->lCov = Pfi

#if USE_NUMERICAL_STABILIZED_KALMAN_FILTERS
            Eigen::VectorXf x = li_old->lhat;
            Eigen::MatrixXf P = li_old->lCov;
            Eigen::MatrixXf H = Hl;
            Eigen::MatrixXf R = z_tmp->getzCov();

            KF_cholesky_update(x, P, v, R, H);

            landmark* li_update = new landmark;
            li_update->c = z_tmp->c;
            li_update->lhat = x;
            li_update->lCov = P;
#endif

#if !USE_NUMERICAL_STABILIZED_KALMAN_FILTERS
            Eigen::MatrixXf Zk;
            Zk = z_tmp->getzCov() + Hl*li_old->lCov*Hl.transpose(); // (3.35)
            Eigen::MatrixXf Kk;
            Kk = li_old->lCov*Hl.transpose()*Zk.inverse(); // (3.36) - Kalman gain

            landmark* li_update = new landmark;
            li_update->c = z_tmp->c;
            li_update->lhat = li_old->lhat + Kk*(z_tmp->z - z_hat); // (3.37)            

            Eigen::MatrixXf tmpMatrix;
            tmpMatrix = Kk*Hl;
            li_update->lCov = (Eigen::MatrixXf::Identity(tmpMatrix.rows(),tmpMatrix.cols())-tmpMatrix)*li_old->lCov;// (3.38)*/
#endif

            //cout << "Correct landmark lhat: " << li_old->lhat << endl;

            if (li_update->lhat != li_update->lhat) {
                cout << "landmark update NaN err, ID: " << z_tmp->c << endl;
            }

            map->correctLandmark(li_update);
        }
    }
}


void Particle::handleNewMeas(MeasurementSet* z_New, VectorChiFastSLAMf s_proposale){

    if (z_New != NULL && z_New->nMeas != 0 ){

        for( int i = 1; i <= z_New->nMeas; i = i + 1 ) {

            Measurement* z_tmp = z_New->getMeasurement(i);

            landmark* li = new landmark;

            li->c = z_tmp->c;

            li->lhat = z_tmp->inverseMeasurementModel(s_proposale);

            //cout << "lhat" << li->lhat << endl;

            Eigen::MatrixXf Hl;
            Hl = z_tmp->calculateHl(s_proposale,li->lhat);

            Eigen::MatrixXf zCov_tmp = z_tmp->getzCov();

            li->lCov = (Hl.transpose()*zCov_tmp.inverse()*Hl).inverse(); // this is different from this line: https://github.com/bushuhui/fastslam/blob/master/src/fastslam_core.cpp#L567

            map->insertLandmark(li);
        }
    }
}

VectorChiFastSLAMf Particle::drawSampleFromProposaleDistribution(VectorChiFastSLAMf* s_old, VectorUFastSLAMf* u,MeasurementSet* z_Ex, float Ts)
{
    //cout << "D10" << endl;

    VectorChiFastSLAMf s_bar = motionModel(*s_old,u,Ts);
    //cout << endl << "s_bar" << endl << s_bar << endl;

    MatrixChiFastSLAMf sCov_proposale= sCov; // eq (3.28)
    VectorChiFastSLAMf sMean_proposale = s_bar; // eq (3.29)

    if (z_Ex != NULL){

        for(int i = 1; i <= z_Ex->nMeas; i = i + 1 ) {

            Measurement* z_tmp = z_Ex->getMeasurement(i);

            landmark* li_old = map->extractLandmarkNodePointer(z_tmp->c);
            //cout << "landmark in map: " << li_old->lhat << endl;

            Eigen::MatrixXf Hli;
            Hli = z_tmp->calculateHl(s_bar,li_old->lhat); //resizes automatically due to the "=" operator

            //cout << "prop Hli" << endl << Hli << endl;

            Eigen::MatrixXf Hsi;
            Hsi = z_tmp->calculateHs(s_bar,li_old->lhat); //resizes automatically due to the "=" operator

            Eigen::MatrixXf Zki;
            Eigen::MatrixXf zCov_tmp = z_tmp->getzCov();

/*
            if(li_old->c == 55){
                cout << endl << "li_55->lCov" << endl << li_old->lCov << endl;
                cout << endl << "li_55->lhat" << endl << li_old->lhat << endl;
            }
*/
            Zki = zCov_tmp + Hli*(li_old->lCov)*Hli.transpose();

            Eigen::VectorXf zhat;
            zhat = z_tmp->MeasurementModel(s_bar,li_old->lhat);

#if !USE_NUMERICAL_STABILIZED_KALMAN_FILTERS
            Eigen::MatrixXf Kk;
            Kk = sCov_proposale*Hsi.transpose() * (Hsi*sCov_proposale*Hsi.transpose() + Zki).inverse();

            sCov_proposale = (Hsi.transpose()*Zki.inverse()*Hsi + sCov_proposale.inverse()).inverse();  // eq (3.30)
            sMean_proposale = sMean_proposale + sCov_proposale*Hsi.transpose()*Zki.inverse()*(z_tmp->z - zhat); // eq (3.31)

            // Different approach with stabilizing implementation
            //sMean_proposale = sMean_proposale + Kk*(z_tmp->z - zhat); // eq (3.31)
            //sCov_proposale = (Eigen::MatrixXf::Identity(rows,cols) - Kk*Hsi) * sCov_proposale * (Eigen::MatrixXf::Identity(rows,cols) - Kk*Hsi).transpose() + Kk*Zki*Kk.transpose();  // eq (3.30)
#endif

#if USE_NUMERICAL_STABILIZED_KALMAN_FILTERS
            // Rk = Zki
            // Hk = Hsi
            // Pk = sCov_proposale

            /*Eigen::MatrixXf tmp;
            tmp = Kk * Hsi;
            int rows, cols;
            rows = tmp.rows();
            cols = tmp.cols();*/

            Eigen::VectorXf x = sMean_proposale;
            Eigen::MatrixXf P = sCov_proposale;
            Eigen::VectorXf v = (z_tmp->z - zhat);
            Eigen::MatrixXf R = Zki;
            Eigen::MatrixXf H = Hsi;

            cout << "################## proposale ##################" << endl;
            cout << "z: " << endl << z_tmp->z << endl << endl;
            cout << "z_hat: " << endl << zhat << endl << endl;
            cout << "error: " << endl << v << endl << endl;

            KF_cholesky_update(x, P, v, R, H);

            sMean_proposale = x;
            sCov_proposale = P;

            /*// ==== Implementation according to  https://github.com/bushuhui/fastslam/blob/master/src/fastslam_2.cpp#L575-L577
            Eigen::VectorXf xv = sMean_proposale;
            Eigen::MatrixXf Pv = sCov_proposale;

            Eigen::MatrixXf Hvi = Hsi;
            Eigen::MatrixXf Hfi = Hli;
            Eigen::MatrixXf Sf = Zki;
            Eigen::MatrixXf Sfi = Sf.inverse();

            Eigen::VectorXf vi = v;

            //proposal covariance
            Eigen::MatrixXf Pv_inv = Pv.llt().solve(Eigen::MatrixXf::Identity(Pv.rows(), Pv.cols()));
            Pv = Hvi.transpose() * Sfi * Hvi + Pv_inv; //Pv.inverse();
            Pv = Pv.llt().solve(Eigen::MatrixXf::Identity(Pv.rows(), Pv.cols()));//Pv.inverse();

            //proposal mean
            xv = xv + Pv * Hvi.transpose() * Sfi *vi;
            // =================*/

            //cout << endl << "--sCov_proposale (method 1)" << endl << sCov_proposale << endl;
            //cout << endl << "--sCov_proposale (method 2)" << endl << Pv << endl;
            //cout << endl << "--sCov_proposale (method 3)" << endl << P << endl;
#endif

            //cout << "i: " << i << "     z_tmp->c: " << z_tmp->c << endl;
            //cout << "z_tmp->z: " << endl << z_tmp->z << endl << endl;
            //cout << "li_old->lhat: " << endl << li_old->lhat << endl << endl;
            //cout << "z_tmp->MeasurementModel: " << endl << zhat << endl << endl;

            if (sCov_proposale(0,0) != sCov_proposale(0,0)) {
                cout << "sCov NaN err" << endl;
            }


        }
    }

    //cout << endl << "##############################" << endl;
    //cout << endl << "sMean_proposale" << endl << sMean_proposale << endl;
    //cout << endl << "sCov_proposale" << endl << sCov_proposale << endl;

    VectorChiFastSLAMf s_proposale = drawSampleRandomPose(sMean_proposale, sCov_proposale);

    //cout << endl << "s_proposale" << endl << s_proposale << endl;

    return s_proposale;

}


// Motion model Jacobian relative to pose - is only used in drawSampleFromProposaleDistributionNEW
MatrixChiFastSLAMf Particle::calculateFs(VectorChiFastSLAMf *s_k_minor_1){
    return MatrixChiFastSLAMf::Identity();
}

// drawSampleFromProposaleDistributionNEW keeps track of individual Kalman filters for the pose estimate and pose covariance of each individual particle
VectorChiFastSLAMf Particle::drawSampleFromProposaleDistributionNEW(VectorChiFastSLAMf* s_old, VectorUFastSLAMf* u,MeasurementSet* z_Ex, float Ts)
{
    //cout << "D10" << endl;
    //generate random noise with zero mean and the known model noise covariance
    VectorChiFastSLAMf wk = drawSampleRandomPose(VectorChiFastSLAMf::Zero(), sCov); // draw random noise

    // prediction step
    VectorChiFastSLAMf s_bar = motionModel(*s_old,u,Ts) + wk;

    MatrixChiFastSLAMf Fs = calculateFs(s_old);
    MatrixChiFastSLAMf sCov_proposale =  Fs.transpose()*s_k_Cov*Fs + sCov;

    //MatrixChiFastSLAMf << endl << "s_bar" << endl << s_bar << endl;
    //prediction step
    VectorChiFastSLAMf sMean_proposale = s_bar;
    if (z_Ex != NULL){
        for(int i = 1; i <= z_Ex->nMeas; i = i + 1 ) {

            Measurement* z_tmp = z_Ex->getMeasurement(i);

            landmark* li_old = map->extractLandmarkNodePointer(z_tmp->c);
            //cout << "landmark in map: " << li_old->lhat << endl;

            Eigen::MatrixXf Hli;
            Hli = z_tmp->calculateHl(s_bar,li_old->lhat); //resizes automatically due to the "=" operator

            //cout << "prop Hli" << endl << Hli << endl;

            Eigen::MatrixXf Hsi;
            Hsi = z_tmp->calculateHs(s_bar,li_old->lhat); //resizes automatically due to the "=" operator

            Eigen::MatrixXf Zki;
            Eigen::MatrixXf zCov_tmp = z_tmp->getzCov();
/*
            if(li_old->c == 55){
                cout << endl << "li_55->lCov" << endl << li_old->lCov << endl;
                cout << endl << "li_55->lhat" << endl << li_old->lhat << endl;
            }
*/
            Zki = zCov_tmp + Hli*(li_old->lCov)*Hli.transpose();


            // Rk = Zki
            // Hk = Hsi
            // Pk = sCov_proposale
            Eigen::MatrixXf Kk;
            Kk = sCov_proposale*Hsi.transpose() * (Hsi*sCov_proposale*Hsi.transpose() + Zki).inverse();

            Eigen::VectorXf zhat;
            zhat = z_tmp->MeasurementModel(s_bar,li_old->lhat);

            //cout << "i: " << i << "     z_tmp->c: " << z_tmp->c << endl;
            //cout << "z_tmp->z: " << endl << z_tmp->z << endl << endl;
            //cout << "li_old->lhat: " << endl << li_old->lhat << endl << endl;
            //cout << "z_tmp->MeasurementModel: " << endl << zhat << endl << endl;

            Eigen::MatrixXf tmp;
            tmp = Kk * Hsi;
            int rows, cols;
            rows = tmp.rows();
            cols = tmp.cols();

            Eigen::VectorXf x = sMean_proposale;
            Eigen::MatrixXf P = sCov_proposale;
            Eigen::VectorXf v = (z_tmp->z - zhat);
            Eigen::MatrixXf R = Zki;
            Eigen::MatrixXf H = Hsi;

            KF_cholesky_update(x, P, v, R, H);

            //sCov_proposale = (Hsi.transpose()*Zki.inverse()*Hsi + sCov_proposale.inverse()).inverse();  // eq (3.30)
            //sMean_proposale = sMean_proposale + sCov_proposale*Hsi.transpose()*Zki.inverse()*(z_tmp->z - zhat); // eq (3.31)
            sMean_proposale = sMean_proposale + Kk*(z_tmp->z - zhat); // eq (3.31)
            sCov_proposale = (Eigen::MatrixXf::Identity(rows,cols) - Kk*Hsi) * sCov_proposale * (Eigen::MatrixXf::Identity(rows,cols) - Kk*Hsi).transpose() + Kk*Zki*Kk.transpose();  // eq (3.30)
            //cout << endl << "--sCov_proposale (method 1)" << endl << sCov_proposale << endl;

            // ==== Implementation according to  https://github.com/bushuhui/fastslam/blob/master/src/fastslam_2.cpp#L575-L577
            Eigen::VectorXf xv = sMean_proposale;
            Eigen::MatrixXf Pv = sCov_proposale;

            Eigen::MatrixXf Hvi = Hsi;
            Eigen::MatrixXf Hfi = Hli;
            Eigen::MatrixXf Sf = Zki;
            Eigen::MatrixXf Sfi = Sf.inverse();

            Eigen::VectorXf vi = v;

            //proposal covariance
            Eigen::MatrixXf Pv_inv = Pv.llt().solve(Eigen::MatrixXf::Identity(Pv.rows(), Pv.cols()));
            Pv = Hvi.transpose() * Sfi * Hvi + Pv_inv; //Pv.inverse();
            Pv = Pv.llt().solve(Eigen::MatrixXf::Identity(Pv.rows(), Pv.cols()));//Pv.inverse();

            //proposal mean
            xv = xv + Pv * Hvi.transpose() * Sfi *vi;

            // =================

            //cout << endl << "--sCov_proposale (method 2)" << endl << Pv << endl;
            //cout << endl << "--sCov_proposale (method 3)" << endl << P << endl;
            sMean_proposale = x;
            sCov_proposale = P;

            if (sCov_proposale(0,0) != sCov_proposale(0,0)) {
                cout << "sCov NaN err" << endl;
            }
        }
    }

    //cout << endl << "sMean_proposale" << endl << sMean_proposale << endl;
    //cout << endl << "sCov_proposale" << endl << sCov_proposale << endl;
    //cout << endl << "s_proposale" << endl << s_proposale << endl;
    VectorChiFastSLAMf s_proposale = sMean_proposale;
    s_k_Cov = sCov_proposale;
    return s_proposale;
}


//http://moby.ihme.washington.edu/bradbell/mat2cpp/randn.cpp.xml
Eigen::MatrixXf randn(int m, int n)
{
    // use formula 30.3 of Statistical Distributions (3rd ed)
    // Merran Evans, Nicholas Hastings, and Brian Peacock
    int urows = m * n + 1;
    Eigen::VectorXf u(urows);

    //u is a random matrix
#if 1
    for (int r=0; r<urows; r++) {
        // FIXME: better way?
        u(r) = std::rand() * 1.0/RAND_MAX;
    }
#else
    u = ( (VectorXf::Random(urows).array() + 1.0)/2.0 ).matrix();
#endif


    Eigen::MatrixXf x(m,n);

    int     i, j, k;
    float   square, amp, angle;

    k = 0;
    for(i = 0; i < m; i++) {
        for(j = 0; j < n; j++) {
            if( k % 2 == 0 ) {
                square = - 2. * std::log( u(k) );
                if( square < 0. )
                    square = 0.;
                amp = sqrt(square);
                angle = 2. * M_PI * u(k+1);
                x(i, j) = amp * std::sin( angle );
            }
            else
                x(i, j) = amp * std::cos( angle );

            k++;
        }
    }

    return x;
}

Eigen::MatrixXf rand(int m, int n)
{
    Eigen::MatrixXf x(m,n);
    int i, j;
    float rand_max = float(RAND_MAX);

    for(i = 0; i < m; i++) {
        for(j = 0; j < n; j++)
            x(i, j) = float(std::rand()) / rand_max;
    }
    return x;
}

//add random measurement noise. We assume R is diagnoal matrix
void add_observation_noise(vector<Eigen::VectorXf> &z, Eigen::MatrixXf &R, int addnoise)
{
    if (addnoise == 1) {
        unsigned long len = z.size();
        if (len > 0) {
            Eigen::MatrixXf randM1 = randn(1,len);
            Eigen::MatrixXf randM2 = randn(1,len);

            for (unsigned long c=0; c<len; c++) {
                z[c][0] = z[c][0] + randM1(0,c)*sqrt(R(0,0));
                z[c][1] = z[c][1] + randM2(0,c)*sqrt(R(1,1));
            }
        }
    }
}


VectorChiFastSLAMf Particle::drawSampleRandomPose(VectorChiFastSLAMf sMean_proposale, MatrixChiFastSLAMf sCov_proposale)
{
    //choleksy decomposition
    Eigen::MatrixXf S = sCov_proposale.llt().matrixL();
    Eigen::MatrixXf X = randn(4,1);

    return S*X + sMean_proposale;
}

// Consider implementing as https://github.com/bushuhui/fastslam/blob/master/src/fastslam_core.cpp#L479-L488
/*VectorChiFastSLAMf Particle::drawSampleRandomPoseOld(VectorChiFastSLAMf sMean_proposale, Matrix6f sCov_proposale) {
    boost::normal_distribution<> nd(0.0, 1.0);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > randN(rng, nd); // see http://lost-found-wandering.blogspot.dk/2011/05/sampling-from-multivariate-normal-in-c.html

    Matrix6f S = (sCov_proposale + sCov_proposale.transpose()) * 0.5; // hack of making it make symmetric

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigenSolver(S);

    VectorChiFastSLAMf normal;
    normal << randN(), randN(), randN(), randN(), randN(), randN(); // generate 6 random numbers in the vector - these are distributed with mean 0 and sigma 1

    Eigen::MatrixXf U = eigenSolver.eigenvectors();
    Eigen::MatrixXf Lambda = eigenSolver.eigenvalues();
    Eigen::MatrixXf Sigma = Lambda.cwiseSqrt().asDiagonal();

    cout << "U" << endl << U << endl;
    cout << "Lambda" << endl << Lambda << endl;
    cout << "Sigma" << endl << Sigma << endl;

    VectorChiFastSLAMf sample = sMean_proposale + U * Sigma * normal;

    return sample;
    //return sMean_proposale;// + 0.000001*VectorChiFastSLAMf::Random();
}*/

VectorChiFastSLAMf Particle::motionModel(VectorChiFastSLAMf sold, VectorUFastSLAMf* u, float Ts) // Ts == sample time
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

    //cout << "#########################################" << endl;
    //cout << "s_old:" << endl << sold << endl;
    //cout << "s_k:" << endl << s_k << endl;
    //cout << "#########################################" << endl;

    return s_k;
}


// Motion model Jacobian relative to pose - is only used in drawSampleFromProposaleDistributionNEW
MatrixChiFastSLAMf Particle::calculateFs(VectorChiFastSLAMf *s_k_old){
    return MatrixChiFastSLAMf::Identity();
}

MatrixChiFastSLAMf Particle::calculateFu(VectorChiFastSLAMf *s_k_old){
    return MatrixChiFastSLAMf::Identity();
}

void Particle::calculateImportanceWeight(MeasurementSet* z_Ex, VectorChiFastSLAMf s_proposale){
    Eigen::MatrixXf wCov_i;
    double wi;
    double w_tmp;
    //cout << "imp s_proposale: " << endl << s_proposale << endl;

    if (z_Ex != NULL){
        for( int i = 1; i <= z_Ex->nMeas; i = i + 1 ) {
            Measurement* z_tmp = z_Ex->getMeasurement(i);
            landmark* li_old = map->extractLandmarkNodePointer(z_tmp->c);

            //cout << "imp s_proposale: " << endl << s_proposale << endl;
            //cout << "old li: " << endl << li_old->lhat << endl;
            Eigen::MatrixXf Hli;
            Hli = z_tmp->calculateHl(s_proposale,li_old->lhat); //resizes automatically due to the "=" operator
            //cout << "imp Hli" << endl << Hli << endl;

            Eigen::MatrixXf Hsi;
            Hsi = z_tmp->calculateHs(s_proposale,li_old->lhat); //resizes automatically due to the "=" operator
            //cout << "imp Hsi" << endl << Hsi << endl;

            Eigen::VectorXf zhat;
            zhat = z_tmp->MeasurementModel(s_proposale,li_old->lhat);
           // cout << "imp zhat" << endl << zhat << endl;

            Eigen::VectorXf z_diff;
            z_diff = z_tmp->z - zhat;
            //cout << "z" << endl << z_tmp->z << endl;

            cout << "################## importance weight ##################" << endl;
            cout << "z: " << endl << z_tmp->z << endl << endl;
            cout << "z_hat: " << endl << zhat << endl << endl;
            cout << "error: " << endl << z_diff << endl << endl;


            wCov_i = Hsi*sCov*Hsi.transpose() + Hli*li_old->lCov*Hli.transpose() + z_tmp->getzCov(); // (3.45)
//            cout << "imp wCov_i: " << wCov_i << endl;

            Eigen::MatrixXf expTerm;
            expTerm = z_diff.transpose()*wCov_i.inverse()*z_diff;
//            cout << "imp exp: " << expTerm << endl;

            double determinant = (2*pi*wCov_i).determinant();

            w_tmp = 10000000000000*1/(sqrt( determinant ))*exp( -0.5*expTerm(0,0) );// (3.46) and  (14.2) on page 459 in IPRP

            if (w_tmp != w_tmp){
                cout << "#####################################################" << endl;
                cout << "imp calculated weight tmp: " << w_tmp << endl;
                cout << "imp s_proposale: " << endl << s_proposale << endl;
                cout << "z" << endl << z_tmp->z << endl;
                cout << "old li: " << endl << li_old->lhat << endl;
                cout << "imp Hli" << endl << Hli << endl;
                cout << "imp Hsi" << endl << Hsi << endl;
                cout << "imp zhat" << endl << zhat << endl;
                cout << "imp wCov_i: " << endl << wCov_i << endl;
                cout << "imp determinant: " << determinant << endl;
                cout << "imp exp: " << expTerm << endl;
            }


            if (w_tmp == 0) {
                cout << "#####################################################" << endl;
                cout << "imp Weight = 0!" << endl;
                cout << "imp calculated weight tmp: " << w_tmp << endl;
                cout << "imp s_proposale: " << endl << s_proposale << endl;
                cout << "z" << endl << z_tmp->z << endl;
                cout << "old li: " << endl << li_old->lhat << endl;
                cout << "imp Hli" << endl << Hli << endl;
                cout << "imp Hsi" << endl << Hsi << endl;
                cout << "imp zhat" << endl << zhat << endl;
                cout << "imp wCov_i: " << endl << wCov_i << endl;
                cout << "imp determinant: " << determinant << endl;
                cout << "imp exp: " << expTerm << endl;
            }


            if (i==1){
                wi = w_tmp;
            }
            else{
                wi = wi*w_tmp;
            }
        }
        //cout << "wCov" << endl << wCov_i << endl << endl;
        //cout << "calculated weigth: " << wi << endl;
        w = wi; // (non-normalized) importance weight
    }
    else{ cout << "Error in calculation of importance weight! You should not have reached this point!" << endl; }
}


double Particle::getWeigth()
{
    return w;
}

MatrixChiFastSLAMf Particle::sCov = 0.1*MatrixChiFastSLAMf::Identity(); // static variable - has to be declared outside class!


boost::mt19937 Particle::rng; // Creating a new random number generator every time could be optimized
//rng.seed(static_cast<unsigned int>(time(0)));










/* ############################## Defines ParticleSet class ##############################  */
ParticleSet::ParticleSet(int Nparticles,unsigned int GOT_ID,VectorChiFastSLAMf s0,MatrixChiFastSLAMf s_0_Cov){
    k=0;
    sMean = new Path(s0,k); // makes new path to keep track of the estimated mean of the Particle filter!

    nParticles = Nparticles;
    Parray.reserve(nParticles);

    for(int i = 1; i<=nParticles; i++){
        Parray[i] = new Particle(GOT_ID,s0,s_0_Cov,k);
    }
    StartTime = ros::Time::now().toSec();

    KnownMarkers.push_back(GOT_ID); // add the GOT marker to the known list of landmark IDs
}

ParticleSet::~ParticleSet(){
    double endTime = ros::Time::now().toSec();
    double meanTime = (endTime - StartTime)/k;
    cout << "meanTime per particle update: " << meanTime << endl;

    for(int i = 1; i<=nParticles; i++){
        delete Parray[i];
    }
    delete sMean;
}

int ParticleSet::getNParticles(){
    return nParticles;
}

void ParticleSet::updateParticleSet(MeasurementSet* z, VectorUFastSLAMf u, float Ts){
    Node_MeasurementSet* tmp_pointer = NULL;
    MeasurementSet z_New;
    MeasurementSet z_Ex;
    unsigned int markerID;

    k++;

    if (z != NULL) {
       tmp_pointer = z->firstMeasNode;
    }
    if (tmp_pointer != NULL) { // if we have a non-emtpy measurement set update the particles, otherwise just do resampling
        // Traverse all measurements in measurement set
        do {
            markerID = tmp_pointer->meas->c;
            //cout << "Known landmarks: ";
            //for (std::vector<unsigned int>::const_iterator i = KnownMarkers.begin(); i != KnownMarkers.end(); ++i)
                //cout << *i << ' ';

            //cout << endl;
            if (find(KnownMarkers.begin(), KnownMarkers.end(), markerID) == KnownMarkers.end()) {
                //cout << "New landmark: " << markerID << endl;
                KnownMarkers.push_back(markerID);
                z_New.addMeasurement(tmp_pointer->meas);
            } else {
                z_Ex.addMeasurement(tmp_pointer->meas);
            }

            tmp_pointer = tmp_pointer->nextNode;
        } while (tmp_pointer != NULL);
    }

    for(int i = 1; i<=nParticles;i++){
        //cout << endl << "updating particle: " << i << endl;
        ((Particle*)Parray[i])->updateParticle(&z_Ex,&z_New,&u,k,Ts);
    }

    estimateDistribution(Ts);

    resample();

#if ADD_LANDMARKS_AFTER_RESAMPLING
    for(int i = 1; i<=nParticles;i++){
        ((Particle*)Parray[i])->handleNewMeas(&z_New,*(((Particle*)Parray[i])->s->getPose()));
    }
#endif

}

void ParticleSet::resample(){
    // Resampling wheel
    //cout << "resampling..." << endl;

    vector<Particle*> Parraytmp;
    Parraytmp.reserve(nParticles);

    // Find max w
    Particle* tmpP;

    double wmax = 0;
    for(int i = 1; i<=nParticles;i++){
        if(i==1){
            wmax = Parray[i]->w;
            tmpP = Parray[i];
        }
        else if(Parray[i]->w > wmax){
            wmax = Parray[i]->w;
            tmpP = Parray[i];
        }
    }

    //    cout << endl << "Current best pose estimate" << endl << *(tmpP->s->getPose()) << endl;
    if (wmax != 0){
        // generate random index between 1 and number of particles
        default_random_engine generator;
        uniform_int_distribution<int> distribution(1,nParticles);
        uniform_real_distribution<double> distribution2(0,1);

        int index = distribution(generator); // random index

        double beta = 0;

        for (int z = 1; z <= nParticles; z++){
            // generate random addition to beta
            double rand = distribution2(generator);
            beta = beta + rand*2*wmax;

            double weight = Parray[index]->w;
            while (beta > weight){
                beta = beta - weight;

                index = index + 1;
                if (index > nParticles){
                    index = 1;
                }
                weight = Parray[index]->w;
            }
            //cout << "index: " << index << endl;
            Parraytmp[z] = new Particle(*Parray[index]);
            //cout << "Parraytmp[" << z << "]:" << endl << *(Parraytmp[z]->s->getPose()) << endl;
        }

        for(int i = 1; i<=nParticles; i++){
            delete Parray[i];
        }
        for(int i = 1; i<=nParticles; i++){
            Parray[i] = Parraytmp[i];
        }
        //cout << "Done resampling!" << endl;
    }
    else{
        cout << "something went wrong, all particles have 0 weight" << endl;
    }
}



void ParticleSet::resampleSimple(){
    // primitiv resampling
    double wTotal=0;
    for(int i = 1; i<=nParticles;i++){
        wTotal = wTotal + Parray[i]->w;
    }

    for(int i = 1; i<=nParticles;i++){
        Parray[i]->w = Parray[i]->w/wTotal;
    }

    Particle* tmpPointer;
    double wtmp = 0;

    // primitiv resampling
    for(int i = 1; i<=nParticles;i++){
        if(i==1){
            wtmp = Parray[i]->w;
            tmpPointer = Parray[i];
        }
        else if(Parray[i]->w > wtmp){
            wtmp = Parray[i]->w;
            tmpPointer = Parray[i];
        }
    }

    Particle* Parray_tmp[nParticles];
    for(int i = 1; i<=nParticles;i++){
        Parray_tmp[i] = Parray[i];
    }

    for(int i = 1; i<=nParticles;i++){
        Parray[i] = new Particle(*tmpPointer);
    }

    for(int i = 1; i<=nParticles;i++){
        delete Parray_tmp[i];
    }
}

VectorChiFastSLAMf* ParticleSet::getLatestPoseEstimate(){
    return sMean->getPose();
}

void ParticleSet::estimateDistribution(float Ts){
    double wSum = 0;
    double wNorm = 0;

    for(int i = 1; i<=nParticles;i++){
        if (Parray[i]->w != Parray[i]->w) {
            cout << "Err NaN in Particle: " << i << endl;
        }
        wSum = wSum + Parray[i]->w;
    }

    //cout << "wSum - 1: " << wSum << endl;

    VectorChiFastSLAMf sTmp = VectorChiFastSLAMf::Zero();
    VectorChiFastSLAMf sMean_estimate = VectorChiFastSLAMf::Zero();
    double wSum_squared = 0;


    for(int i = 1; i<=nParticles;i++){

        sTmp = *(Parray[i]->s->getPose());

        wNorm = (Parray[i]->w)/wSum;

        sMean_estimate = sMean_estimate + wNorm*sTmp; // weighted mean!

    }


    MatrixChiFastSLAMf sCov_estimate = MatrixChiFastSLAMf::Zero();
    MatrixChiFastSLAMf sCov_estimate_tmp = MatrixChiFastSLAMf::Zero();
    VectorChiFastSLAMf sDiff = VectorChiFastSLAMf::Zero();

    for(int i = 1; i<=nParticles;i++){

        sTmp = *(Parray[i]->s->getPose());

        wNorm = (Parray[i]->w)/wSum;

        sDiff = sTmp-sMean_estimate;

        for(int j = 0; j<sDiff.rows();j++){
            for(int k = 0; k<sDiff.cols();k++){
                sCov_estimate_tmp(j,k) = sDiff(j)*sDiff(k)*(float)wNorm; // weighted mean!
            }
        }
        sCov_estimate = sCov_estimate + sCov_estimate_tmp;
        wSum_squared = wSum_squared + wNorm*wNorm;
    }

    sCov_estimate = 1/(1-wSum_squared)*sCov_estimate;

    //cout << endl << "sCov_estimate" << endl << sCov_estimate << endl;
    //cout << endl << "sMean_estimate" << endl << sMean_estimate << endl;

    sCov = sCov_estimate;
    sMean->addPose(sMean_estimate,k,Ts);
}




// save data
void ParticleSet::saveData(){
    string topDir = "Data";
    boost::filesystem::create_directories(topDir);

    string filename = topDir + "/t_" + to_string(k) + ".m";

    Path::dataFileStream.open(filename, ios::out);
    Path::dataFileStream << "t" << to_string(k) << " = struct('Particles',[],'meanPath',[]);" << endl;
    Path::dataFileStream.flush();
    Path::dataFileStream.close();

    sMean->saveData(filename);

    Path::dataFileStream.open(filename,ios::in | ios::ate);
    Path::dataFileStream << "t" << to_string(k) << ".meanPath = Path;" << endl;
    Path::dataFileStream << "clear Path" << endl;
    Path::dataFileStream.flush();
    Path::dataFileStream.close();
    int j = 1;
    int interval = 1;
    for(int i = 1; i<=nParticles; i = i + interval){
        Parray[i]->saveData(filename,KnownMarkers);

        if (i == 1){
            Path::dataFileStream.open(filename,ios::out | ios::app);
            Path::dataFileStream << "t" << to_string(k) << ".Particles = particle;" << endl;
            Path::dataFileStream << "clear particle Path map" << endl;
            Path::dataFileStream.flush();
            Path::dataFileStream.close();
        }
        else {
            Path::dataFileStream.open(filename,ios::out | ios::app);
            Path::dataFileStream << "t" << to_string(k) << ".Particles(" << to_string(j) << ") = particle;" << endl;
            Path::dataFileStream << "clear particle Path map" << endl;
            Path::dataFileStream.flush();
            Path::dataFileStream.close();
        }
        j++;
    }
}

void Particle::saveData(string filename,std::vector<unsigned int> LandmarksToSave){
    s->saveData(filename);
    map->saveData(filename, LandmarksToSave);
    Path::dataFileStream.open(filename,ios::out | ios::app);
    Path::dataFileStream << "particle = struct('Path',Path,'map',map);" << endl;
    Path::dataFileStream.flush();
    Path::dataFileStream.close();
}

void Path::saveData(string filename){
    Path::dataFileStream.open(filename,ios::out | ios::app);
    Path::dataFileStream << "Path = struct('PathLength',[],'Path',[],'Ts',[]);" << endl;
    Path::dataFileStream << "Path.PathLength = " << PathLength << ";" << endl;

    unsigned int j = 1;
    if (PathRoot!=NULL){
        Node_Path* tmp_pointer = PathRoot->nextNode;
        while (tmp_pointer->nextNode != NULL){
            tmp_pointer = tmp_pointer->nextNode;

            Path::dataFileStream << "Path.Path(:," << j << ") = " << tmp_pointer->S.format(Path::OctaveFmt) << ";" << endl;
            Path::dataFileStream << "Path.Ts(:," << j << ") = " << tmp_pointer->Ts << ";" << endl;
            j++;
        }
    }
    Path::dataFileStream.flush();
    Path::dataFileStream.close();
}

void MapTree::saveData(string filename,std::vector<unsigned int> LandmarksToSave){
    Path::dataFileStream.open(filename,ios::out | ios::app);
    Path::dataFileStream << "map = struct('nLandmarks',[],'mean',[],'cov',[],'identifier',[]);" << endl;
    Path::dataFileStream << "map.nLandmarks = " << N_Landmarks << ";" << endl;

    cout << "Size of LandmarksToSave:" << LandmarksToSave.size() << endl;
    for(unsigned int i = 0;i<LandmarksToSave.size();i++){
        unsigned int j = LandmarksToSave[i];
        if (extractLandmarkNodePointer(j) != NULL){
             Path::dataFileStream << "map.mean(:," << i+1 << ") = " << extractLandmarkNodePointer(j)->lhat.format(Path::OctaveFmt) << ";" << endl;
             Path::dataFileStream << "map.cov(:,:," << i+1 << ") = " << extractLandmarkNodePointer(j)->lCov.format(Path::OctaveFmt) << ";" << endl;
             Path::dataFileStream << "map.identifier(" << i+1 << ") = " << extractLandmarkNodePointer(j)->c << ";" << endl;
        }
        else{
            cout<<"Error: NULL pointer!";
        }
    }
    Path::dataFileStream.flush();
    Path::dataFileStream.close();
}



IIR::IIR(const float * coeff_a, int no_coeff_a, const float * coeff_b, int no_coeff_b)
{
    int i;

    oldOutputs.resize(no_coeff_a-1);
    oldOutputs << Eigen::VectorXf::Zero(no_coeff_a-1);
    oldInputs.resize(no_coeff_b-1);
    oldInputs << Eigen::VectorXf::Zero(no_coeff_b-1);
    a.resize(no_coeff_a,1);
    b.resize(no_coeff_b,1);
    for (i = 0; i < no_coeff_b; i++) {
        a(i) = coeff_a[i];
    }
    for (i = 0; i < no_coeff_b; i++) {
        b(i) = coeff_b[i];
    }

    aLen = no_coeff_a;
    bLen = no_coeff_b;
}

float IIR::Filter(float input)
{
    int j;
    float output;
    Eigen::MatrixXf oldInputContrib;
    Eigen::MatrixXf oldOutputContrib;
    oldInputContrib = oldInputs.transpose()*b.block(1,0,(bLen-1),1); // should just become a scalar
    oldOutputContrib = oldOutputs.transpose()*a.block(1,0,(aLen-1),1); // should just become a scalar

    // output = ( b(1)*input + oldInput'*b(2:end)' - oldOutput'*a(2:end)' ) / a(1);
    output = ( b(0)*input + oldInputContrib(0,0) - oldOutputContrib(0,0) ) / a(0);

    for (j = (aLen-2); j >= 1; j--) {
        oldOutputs(j) = oldOutputs(j-1);
    }
    oldOutputs(0) = output;
    for (j = (bLen-2); j >= 1; j--) {
        oldInputs(j) = oldInputs(j-1);
    }
    oldInputs(0) = input;

    //cout << "oldOutputs: " << oldOutputs << endl;
    //cout << "oldInputs: " << oldInputs << endl;

    return output;
}
