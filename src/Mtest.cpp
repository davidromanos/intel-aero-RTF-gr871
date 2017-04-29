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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <random>

using namespace std;
using namespace Eigen;

#define deg2rad(x)  (x*M_PI)/180.f
#define rad2deg(x)  (x*180.f)/M_PI

typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 6, 6> Matrix6f;
typedef Matrix<float, 6, 1> VectorUFastSLAMf; // velocities in the order: [x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot]
typedef Matrix<float, 6, Dynamic> Matrix6kf;
float pi = 3.14159265359;

/*template<typename M>
M load_csv_to_matrix (const std::string & path) {
    ifstream indata;
    indata.open(path);
    string line;
    vector<double> values;
    uint rows = 0;
    while (getline(indata, line)) {
        stringstream lineStream(line);
        string cell;
        while (getline(lineStream, cell, ',')) {
            values.insert(values.end(), stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}*/

MatrixXd load_csv_to_matrix (const std::string & path) {
    ifstream indata;
    indata.open(path);
    string line;
    vector<double> values;
    uint rows = 0;
    while (getline(indata, line)) {
        stringstream lineStream(line);
        string cell;
        while (getline(lineStream, cell, ',')) {
            values.insert(values.end(), stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<MatrixXd::Scalar, MatrixXd::RowsAtCompileTime, MatrixXd::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}

vector<vector<double> > load_csv (const string &path) {
    ifstream indata;
    indata.open(path.c_str());
    string line;
    vector<vector<double> > rowVectors;
    vector<double> values;
    uint rows = 0;
    while (getline(indata, line)) {
        stringstream lineStream(line);
        string cell;
        while (getline(lineStream, cell, ',')) {
            values.insert(values.end(), stod(cell));
        }
        rowVectors.insert(rowVectors.end(), values);
        values.clear();
        ++rows;
    }
    return rowVectors;
}



/* ############################## Defines measurement class ##############################  */

class Measurement
{
public:
    static MatrixXf zCov; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/

    /* variables */
    unsigned int c; 	/* measurement identifier - 0 for pose measurement, 1 for GOT and 2...N for landmark identifier */
    VectorXf z;	/* actual measurement - can take different sizes! */
    ros::Time timestamp;

    /* functions */
   // Measurement();
   // ~Measurement();
    virtual MatrixXf calculateHl(Vector6f pose, Vector3f l) = 0;		/* calculates derivative of measurement model with respect to landmark variable - l */
    virtual MatrixXf calculateHs(Vector6f pose, Vector3f l) = 0;		/* calculates derivative of measurement model with respect to pose variable - s */
    virtual VectorXf inverseMeasurementModel(Vector6f pose) = 0;
    virtual VectorXf MeasurementModel(Vector6f pose, Vector3f l) = 0;
    virtual MatrixXf getzCov() = 0;
private:
};

/* ############################## Defines GOTMeasurement class ##############################  */
class GOTMeasurement : public Measurement
{
    public:
    static MatrixXf zCov; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/

    GOTMeasurement(unsigned int i, Vector3f GOT_meas);
    MatrixXf calculateHs(Vector6f pose, Vector3f l);
    MatrixXf calculateHl(Vector6f pose, Vector3f l);
    VectorXf inverseMeasurementModel(Vector6f pose);
    VectorXf MeasurementModel(Vector6f pose, Vector3f l);
    MatrixXf getzCov();

private:
};

GOTMeasurement::GOTMeasurement(unsigned int i, Vector3f GOT_meas)
{
    c = i;
    z = GOT_meas;
    timestamp = ros::Time::now();
}

VectorXf GOTMeasurement::MeasurementModel(Vector6f pose, Vector3f l)
{
    Vector3f z = l - pose.topRows<3>();
    return z;
}

VectorXf GOTMeasurement::inverseMeasurementModel(Vector6f pose)
{
    Vector6f s = pose; // temp variable to make it look like equations
    Vector3f l = s.topRows<3>() + z;
    return l;
}

MatrixXf GOTMeasurement::calculateHs(Vector6f pose, Vector3f l)
{    
    MatrixXf Hs(3, 6);
    Hs << -1.0*Matrix3f::Identity(3,3), Matrix3f::Zero(3,3);
    //cout << " Hs" << Hs << endl;
    return Hs;
}

MatrixXf GOTMeasurement::calculateHl(Vector6f pose, Vector3f l)
{
    //s = pose; // temp variable to make it look like equations
    Matrix3f Hl = Matrix3f::Identity();
    return Hl;
};

MatrixXf GOTMeasurement::getzCov(){
    return zCov;
}

MatrixXf GOTMeasurement::zCov = 0.05*Matrix3f::Identity(); // static variable - has to be declared outside class!


/* ############################## Defines ImgMeasurement class ##############################  */
class ImgMeasurement : public Measurement
{
    public:
    static Matrix3f zCov; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/    

    ImgMeasurement(unsigned int i, Vector3f img_me);
    VectorXf inverseMeasurementModel(Vector6f pose);
    MatrixXf calculateHs(Vector6f pose, Vector3f l);
    MatrixXf calculateHl(Vector6f pose, Vector3f l);
    VectorXf MeasurementModel(Vector6f pose, Vector3f l);
    MatrixXf getzCov();

private:
    /* Camera coefficients */
    static constexpr float ax = 617.85888671875;
    static constexpr float ay = 623.442626953125;
    static constexpr float x0 = 321.3709716796875;
    static constexpr float y0 = 253.7631072998047;
};

ImgMeasurement::ImgMeasurement(unsigned int i, Vector3f img_meas)
{
    c = i;
    z = img_meas;
    timestamp = ros::Time::now();
}

VectorXf ImgMeasurement::MeasurementModel(Vector6f pose, Vector3f l)
{
    Vector3f z;

    float c_psi = cos(pose(5));
    float s_psi = sin(pose(5));
    float c_theta = cos(pose(4));
    float s_theta = sin(pose(4));
    float c_phi = cos(pose(3));
    float s_phi = sin(pose(3));

    // Calculate world coordinate of landmark in the camera frame - Notice we use Roll-Pitch-Yaw angle convention
    float c_xl = (-c_psi*s_theta*s_phi + s_psi*c_phi)*(l(0) - pose(0)) + (-s_psi*s_theta*s_phi-c_psi*c_phi)*(l(1) - pose(1)) - (c_theta*s_phi)*(l(2) - pose(2));
    float c_yl = (-c_psi*s_theta*c_phi-s_psi*s_phi)*(l(0) - pose(0)) + (-s_psi*s_theta*c_phi+c_psi*s_phi)*(l(1) - pose(1)) - (c_theta*s_phi)*(l(2) - pose(2));
    float c_zl = (c_psi*c_theta)*(l(0) - pose(0)) + (s_psi*c_theta)*(l(1) - pose(1)) - s_theta*(l(2) - pose(2));

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

VectorXf ImgMeasurement::inverseMeasurementModel(Vector6f pose)
{
    float c_psi = cos(pose(5));
    float s_psi = sin(pose(5));
    float c_theta = cos(pose(4));
    float s_theta = sin(pose(4));
    float c_phi = cos(pose(3));
    float s_phi = sin(pose(3));

    Matrix3f R; // Rotation matrix corresponding to: BC_R' * EB_R'
    R << (-c_psi*s_theta*s_phi + s_psi*c_phi), (-s_psi*s_theta*s_phi-c_psi*c_phi), -(c_theta*s_phi),
            (-c_psi*s_theta*c_phi-s_psi*s_phi),  (-s_psi*s_theta*c_phi+c_psi*s_phi), -(c_theta*c_phi),
            (c_psi*c_theta),                     (s_psi*c_theta),                    -(s_theta);

    float c_zl = z(2);
    float c_xl = (z(0)*c_zl - x0*c_zl) / ax;
    float c_yl = (z(1)*c_zl - y0*c_zl) / ay;

    Vector3f CamLandmark;
    CamLandmark << c_xl, c_yl, c_zl;

    Vector3f pose_xyz;
    pose_xyz << pose(0), pose(1), pose(2);

    Vector3f TempLandmark = CamLandmark + R*pose_xyz;

    Vector3f WorldLandmark = R.transpose() * TempLandmark; // rot.transpose() corresponds to EB_R * BC_R

    return WorldLandmark;
}

MatrixXf ImgMeasurement::calculateHs(Vector6f pose, Vector3f l)
{
    float c_psi = cos(pose(5));
    float s_psi = sin(pose(5));
    float c_theta = cos(pose(4));
    float s_theta = sin(pose(4));
    float c_phi = cos(pose(3));
    float s_phi = sin(pose(3));

    MatrixXf Hs(3, 6);

    float den = powf((c_theta*c_psi*(pose(0) - l(0)) - s_theta*(pose(2) - l(2)) + c_theta*s_psi*(pose(1) - l(1))),2);

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

    return Hs;
}

MatrixXf ImgMeasurement::calculateHl(Vector6f pose, Vector3f l)
{
    float c_psi = cos(pose(5));
    float s_psi = sin(pose(5));
    float c_theta = cos(pose(4));
    float s_theta = sin(pose(4));
    float c_phi = cos(pose(3));
    float s_phi = sin(pose(3));

    Matrix3f R; // Rotation matrix corresponding to: BC_R' * EB_R'
    R << (-c_psi*s_theta*s_phi + s_psi*c_phi), (-s_psi*s_theta*s_phi-c_psi*c_phi), -(c_theta*s_phi),
            (-c_psi*s_theta*c_phi-s_psi*s_phi),  (-s_psi*s_theta*c_phi+c_psi*s_phi), -(c_theta*c_phi),
            (c_psi*c_theta),                     (s_psi*c_theta),                    -(s_theta);

    Matrix3f Hl;

    float den1 = powf((R(2,0)*(pose(0) - l(0)) + R(2,1)*(pose(1) - l(1)) + R(2,2)*(pose(2) - l(2))),2);
    float den2 = (R(2,0)*(pose(0) - l(0)) + R(2,1)*(pose(1) - l(1)) + R(2,2)*(pose(2) - l(2)));

    Hl(0,0) = (R(2,0)*ax*(R(0,0)*(pose(0) - l(0)) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/den1 - (R(0,0)*ax)/den2;
    Hl(1,0) = (R(2,0)*ay*(R(1,0)*(pose(0) - l(0)) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/den1 - (R(1,0)*ay)/den2;
    Hl(2,0) = R(2,0);
    Hl(0,1) = (R(2,1)*ax*(R(0,0)*(pose(0) - l(0)) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/den1 - (R(0,1)*ax)/den2;
    Hl(1,1) = (R(2,1)*ay*(R(1,0)*(pose(0) - l(0)) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/den1 - (R(1,1)*ay)/den2;
    Hl(2,1) = R(2,1);
    Hl(0,2) = (R(2,2)*ax*(R(0,0)*(pose(0) - l(0)) + R(0,1)*(pose(1) - l(1)) + R(0,2)*(pose(2) - l(2))))/den1 - (R(0,2)*ax)/den2;
    Hl(1,2) = (R(2,2)*ay*(R(1,0)*(pose(0) - l(0)) + R(1,1)*(pose(1) - l(1)) + R(1,2)*(pose(2) - l(2))))/den1 - (R(1,2)*ay)/den2;
    Hl(2,2) = R(2,2);

    return Hl;
};

MatrixXf ImgMeasurement::getzCov(){
    return zCov;
}

Matrix3f ImgMeasurement::zCov = 5*Matrix3f::Identity(); // static variable - has to be declared outside class!




/* ############################## Defines measurement set class ##############################  */
struct Node_MeasurementSet {
    Measurement* meas;
    Node_MeasurementSet *nextNode;
    int measIdentifier;
};

class MeasurementSet
{
public:
    /* variables */
    Node_MeasurementSet *firstMeasNode;
    int nMeas;

    /* functions */
    MeasurementSet();
    MeasurementSet(Measurement *meas);
    ~MeasurementSet();
    void deleteMeasurementSet();
    void addMeasurement(Measurement *meas);
    int countNumberOfMeasurements();
    int getNumberOfMeasurements();
    Measurement* getMeasurement(int i);

private:
    void deleteMeasurementSet(Node_MeasurementSet *MeasNode);
};

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
    cout << "n1: " << nMeas << endl;
}

MeasurementSet::~MeasurementSet(){
    deleteMeasurementSet();
}

void MeasurementSet::deleteMeasurementSet(){
    if(firstMeasNode != NULL){
        if(firstMeasNode->nextNode != NULL){
            deleteMeasurementSet(firstMeasNode->nextNode);
        }
        delete firstMeasNode;
        nMeas--;
    }
}

void MeasurementSet::deleteMeasurementSet(Node_MeasurementSet *MeasNode){
    if(MeasNode->nextNode != NULL){
        deleteMeasurementSet(MeasNode->nextNode);
    }
    delete MeasNode;
    nMeas--;
}

void MeasurementSet::addMeasurement(Measurement *meas){
    if (firstMeasNode == NULL){
        firstMeasNode = new Node_MeasurementSet;
        firstMeasNode->meas = meas;
        firstMeasNode->nextNode = NULL;
        nMeas = 1;
        firstMeasNode->measIdentifier = nMeas;
    }
    else{
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

unsigned int globalLandmarkCounter; // can be used to check if number of landmarks does not grow without bound
unsigned int globalMapNodeCounter; // can be used to check if number of MapNodes does not grow without bound

struct landmark
{
  unsigned int c; 		/* landmark identifier */
  Vector3f lhat;
  Matrix3f lCov;
  landmark() //Constructor
  {
      globalLandmarkCounter++;
  }
  ~landmark(){//Destructor
      globalLandmarkCounter--;
  }

};

struct mapNode
{
  unsigned int key_value;    /* landmark identifier */
  mapNode *left;               /* pointer for the left node */
  mapNode *right;              /* pointer for the right node */
  landmark *l;              /* pointer for a landmark; is used when *left == NULL or *right == NULL */
  unsigned int referenced;  /* how many nodes/paticles points to this node? if zero the node should be deleted! */

  mapNode() //Constructor
  {
      globalMapNodeCounter++;
  }
  ~mapNode(){//Destructor
      globalMapNodeCounter--;
  }
};


int mapTreeIdentifierCounter = 1;


class MapTree
{
    public:
        // variables
        int mapTreeIdentifier;
        unsigned int N_Landmarks;
        int N_layers;
        unsigned int N_nodes;
        mapNode* root;

        // functions
        MapTree();
        MapTree(const MapTree &MapToCopy); // copy constructer
        ~MapTree();
        void insertLandmark(landmark* newLandmark);
        void creatNewLayers(int Needed_N_layers);
        int countNLayers();
        void removeReferenceToSubTree(mapNode* nodeToStartFrom);
        void correctLandmark(landmark* newLandmarkData);
        landmark* extractLandmarkNodePointer(unsigned int Landmark_identifier);
        void printAllLandmarkPositions();

    private:
        mapNode* makeNewPath(landmark* newLandmarkData, mapNode* startNode);
};

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
    if(N_Landmarks==0){
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
    else{
        float c_tmp = (float)newLandmark->c;
        int Needed_N_layers = (int)ceil(log2(c_tmp));

        if (Needed_N_layers>N_layers){
            creatNewLayers(Needed_N_layers);
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
    }
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
            pointerForNewMapNode->left->referenced++;
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
             cout << "l_" << i <<": "<< extractLandmarkNodePointer(i)->lhat.transpose() << endl;
        }
        else{
            //tmpMatrix.col(i-1) = Vector3f::Zero();
            cout<<"Error: NULL pointer!";
        }
    }
    //cout << "D33 ";
}




/* ############################## Defines Path class ##############################  */
struct Node_Path {
    Vector6f S;
    unsigned int k;
    Node_Path *nextNode;
    unsigned int referenced;
};

class Path
{
public:
    /* variables */
    Node_Path *PathRoot;
    unsigned int PathLength;

    /* functions */
    Path(Vector6f S, unsigned int k);
    Path(const Path &PathToCopy); // copy constructer
    ~Path();
    void deletePath();
    void addPose(Vector6f S, unsigned int k);
    unsigned int countLengthOfPath();
    Vector6f* getPose();
    Vector6f* getPose(unsigned int k);

private:
    void deletePath(Node_Path *PathNode);
};

Path::Path(Vector6f S, unsigned int k){
    Node_Path* firstPathNode = new Node_Path;
    firstPathNode->S = S;
    firstPathNode->k = k;
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

void Path::addPose(Vector6f S, unsigned int k){

    Node_Path* tmpPathNode = new Node_Path;
    tmpPathNode->S = S;
    tmpPathNode->k = k;
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

Vector6f* Path::getPose(){
    // return latest pose!
    return &(PathRoot->nextNode->S);
}

Vector6f* Path::getPose(unsigned int k){
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
            cout << tmp_pointer->k << ","; // for debugging
            tmp_pointer = tmp_pointer->nextNode;
        }
        return &(tmp_pointer->S);
    }
}




/* ############################## Defines particle class ##############################  */

class Particle
{
public:
    /* variables */
    Path* s;
    MapTree* map;
    double w;

    static boost::mt19937 rng; // Creating a new random number generator every time could be optimized
    //rng.seed(static_cast<unsigned int>(time(0)));


    /* functions */
    Particle(Vector6f s0 = Vector6f::Constant(0), unsigned int k = 0); 		// Initialize a standard particle with "zero-pose" or custom pose
    Particle(const Particle &ParticleToCopy);       // Copy constructer used in case where we need to make a copy of a Particle
    ~Particle();
    void updateParticle(MeasurementSet* z_Ex,MeasurementSet* z_New, VectorUFastSLAMf* u, unsigned int k, float Ts);
    double getWeigth();

private:
    /* variables */
    static Matrix6f sCov; // motion model covariance - does not change?

    /* functions */
    Vector6f drawSampleFromProposaleDistribution(Vector6f* s_old, VectorUFastSLAMf* u, MeasurementSet* z_Ex, float Ts);
    Vector6f motionModel(Vector6f sold, VectorUFastSLAMf* u, float Ts);
    void handleExMeas(MeasurementSet* z_Ex, Vector6f s_proposale);
    void handleNewMeas(MeasurementSet* z_New, Vector6f s_proposale);
    void updateLandmarkEstimates(Vector6f s_proposale, MeasurementSet* z_Ex, MeasurementSet* z_New);
    Vector6f drawSampleRandomPose(Vector6f sMean_proposale, Matrix6f sCov_proposale);
    void calculateImportanceWeight(MeasurementSet* z_Ex, Vector6f s_proposale);
};

Particle::Particle(Vector6f s0, unsigned int k)   // default Constructor definition
{
    s = new Path(s0,k); // makes new path!
    map = new MapTree; // makes new mapTree
    w = 1;
}

Particle::Particle(const Particle &ParticleToCopy)   // Copy Constructor
{
    //cout << "Copying particle" << endl;
    s = new Path(*(ParticleToCopy.s)); //makes copy of s on the heap
    map = new MapTree(*(ParticleToCopy.map));
    w = ParticleToCopy.w;
}

Particle::~Particle()
{    
    //cout << "Deleting particle" << endl;
    delete s; // call destructor of s
    delete map; // call destructor of map
}

void Particle::updateParticle(MeasurementSet* z_Ex,MeasurementSet* z_New,VectorUFastSLAMf* u, unsigned int k, float Ts)
{
    Vector6f s_proposale = drawSampleFromProposaleDistribution(s->getPose(),u,z_Ex,Ts);
    s->addPose(s_proposale,k); // we are done estimating our pose and add it to the path!

    updateLandmarkEstimates(s_proposale,z_Ex,z_New);

    if (z_Ex != NULL && z_Ex->nMeas != 0 ){
        calculateImportanceWeight(z_Ex,s_proposale);
    }
}

void Particle::updateLandmarkEstimates(Vector6f s_proposale, MeasurementSet* z_Ex, MeasurementSet* z_New){
    handleExMeas(z_Ex,s_proposale);
    handleNewMeas(z_New,s_proposale);
    //cout << "N_landmarks in map after update: " << map->N_Landmarks << endl;
}

void Particle::handleExMeas(MeasurementSet* z_Ex, Vector6f s_proposale){
    if (z_Ex != NULL && z_Ex->nMeas != 0 ){
        //cout << "nMeas in z_Ex: " << z_Ex->nMeas << endl;

        for( int i = 1; i < z_Ex->nMeas; i = i + 1 ) {
            Measurement* z_tmp = z_Ex->getMeasurement(i);
            landmark* li_old = map->extractLandmarkNodePointer(z_tmp->c);
            VectorXf z_hat = z_tmp->MeasurementModel(s_proposale,li_old->lhat); // (3.33)
            MatrixXf Hl;
            Hl = z_tmp->calculateHl(s_proposale,li_old->lhat);  // (3.34)
            MatrixXf Zk;
            Zk = z_tmp->getzCov() + Hl*li_old->lCov*Hl.transpose(); // (3.35)
            MatrixXf Kk;
            Kk = li_old->lCov*Hl.transpose()*Zk.inverse(); // (3.36) - Kalman gain

            landmark* li_update = new landmark;
            li_update->c = z_tmp->c;
            li_update->lhat = li_old->lhat + Kk*(z_tmp->z - z_hat); // (3.37)

            MatrixXf tmpMatrix;
            tmpMatrix = Kk*Hl;
            li_update->lCov = (MatrixXf::Identity(tmpMatrix.rows(),tmpMatrix.cols())-tmpMatrix)*li_old->lCov;// (3.38)

            map->correctLandmark(li_update);
        }
    }
}


void Particle::handleNewMeas(MeasurementSet* z_New, Vector6f s_proposale){
    if (z_New != NULL && z_New->nMeas != 0 ){
        for( int i = 1; i <= z_New->nMeas; i = i + 1 ) {
            Measurement* z_tmp = z_New->getMeasurement(i);

            landmark* li = new landmark;
            li->c = z_tmp->c;
            li->lhat = z_tmp->inverseMeasurementModel(s_proposale);

            MatrixXf Hl;
            Hl = z_tmp->calculateHl(s_proposale,li->lhat);

            MatrixXf zCov_tmp = z_tmp->getzCov();

            li->lCov = (Hl.transpose()*zCov_tmp.inverse()*Hl).inverse();

            map->insertLandmark(li);
        }
    }
}

Vector6f Particle::drawSampleFromProposaleDistribution(Vector6f* s_old, VectorUFastSLAMf* u,MeasurementSet* z_Ex, float Ts)
{
    //cout << "D10" << endl;
    Vector6f s_bar = motionModel(*s_old,u,Ts);

    //cout << endl << "s_bar" << endl << s_bar << endl;

    Matrix6f sCov_proposale= sCov; // eq (3.28)
    Vector6f sMean_proposale = s_bar; // eq (3.29)
    if (z_Ex != NULL){
        for(int i = 1; i <= z_Ex->nMeas; i = i + 1 ) {

            Measurement* z_tmp = z_Ex->getMeasurement(i);

            landmark* li_old = map->extractLandmarkNodePointer(z_tmp->c);

            MatrixXf Hli;
            Hli = z_tmp->calculateHl(s_bar,li_old->lhat); //resizes automatically due to the "=" operator

            MatrixXf Hsi;
            Hsi = z_tmp->calculateHs(s_bar,li_old->lhat); //resizes automatically due to the "=" operator

            MatrixXf Zki;
            MatrixXf zCov_tmp = z_tmp->getzCov();
/*
            if(li_old->c == 55){
                cout << endl << "li_55->lCov" << endl << li_old->lCov << endl;
                cout << endl << "li_55->lhat" << endl << li_old->lhat << endl;
            }
*/
            Zki = zCov_tmp + Hli*(li_old->lCov)*Hli.transpose();

            VectorXf zhat;
            zhat = z_tmp->MeasurementModel(s_bar,li_old->lhat);

            //cout << "i: " << i << "     z_tmp->c: " << z_tmp->c << endl;
            //cout << "z_tmp->z: " << endl << z_tmp->z << endl << endl;
            //cout << "li_old->lhat: " << endl << li_old->lhat << endl << endl;
            //cout << "z_tmp->MeasurementModel: " << endl << zhat << endl << endl;

            sCov_proposale = (Hsi.transpose()*Zki.inverse()*Hsi + sCov_proposale.inverse()).inverse();  // eq (3.30)
            sMean_proposale = sMean_proposale + sCov_proposale*Hsi.transpose()*Zki.inverse()*(z_tmp->z - zhat); // eq (3.31)
        }
    }

    //cout << endl << "sMean_proposale" << endl << sMean_proposale << endl;
    //cout << endl << "sCov_proposale" << endl << sCov_proposale << endl;

    Vector6f s_proposale = drawSampleRandomPose(sMean_proposale, sCov_proposale);

    //cout << endl << "s_proposale" << endl << s_proposale << endl;

    return s_proposale;
}


Vector6f Particle::drawSampleRandomPose(Vector6f sMean_proposale, Matrix6f sCov_proposale){
    boost::normal_distribution<> nd(0.0, 1.0);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > randN(rng, nd); // see http://lost-found-wandering.blogspot.dk/2011/05/sampling-from-multivariate-normal-in-c.html

    SelfAdjointEigenSolver<MatrixXf> eigenSolver(sCov_proposale);

    Vector6f normal;
    normal << randN(), randN(), randN(), randN(), randN(), randN(); // generate 6 random numbers in the vector - these are distributed with mean 0 and sigma 1

    MatrixXf U = eigenSolver.eigenvectors();
    MatrixXf Lambda = eigenSolver.eigenvalues();
    MatrixXf Sigma = Lambda.cwiseSqrt().asDiagonal();
    Vector6f sample = sMean_proposale + U * Sigma * normal;

    return sample;
    //return sMean_proposale;// + 0.000001*Vector6f::Random();
}

Vector6f Particle::motionModel(Vector6f sold, VectorUFastSLAMf* u, float Ts) // Ts == sample time
{
    Vector6f s_k = sold; // s(k) = f(s(k-1),u(k))

    // Kinematic motion model where u=[x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot]
    //s_k += Ts*u;

    return s_k;
}


void Particle::calculateImportanceWeight(MeasurementSet* z_Ex, Vector6f s_proposale){
    MatrixXf wCov_i;
    double wi;
    double w_tmp;

    if (z_Ex != NULL){
        for( int i = 1; i <= z_Ex->nMeas; i = i + 1 ) {
            Measurement* z_tmp = z_Ex->getMeasurement(i);
            landmark* li_old = map->extractLandmarkNodePointer(z_tmp->c);

            MatrixXf Hli;
            Hli = z_tmp->calculateHl(s_proposale,li_old->lhat); //resizes automatically due to the "=" operator
            //cout << "Hli" << endl << Hli << endl;

            MatrixXf Hsi;
            Hsi = z_tmp->calculateHs(s_proposale,li_old->lhat); //resizes automatically due to the "=" operator
            //cout << "Hsi" << endl << Hsi << endl;

            VectorXf zhat;
            zhat = z_tmp->MeasurementModel(s_proposale,li_old->lhat);
            //cout << "zhat" << endl << zhat << endl;

            VectorXf z_diff;
            z_diff = z_tmp->z - zhat;

            wCov_i = Hsi*sCov*Hsi.transpose() + Hli*li_old->lCov*Hli.transpose() + z_tmp->getzCov(); // (3.45)


            MatrixXf expTerm;
            expTerm = z_diff.transpose()*wCov_i.inverse()*z_diff;

            w_tmp = 1/(sqrt( (2*pi*wCov_i).determinant() ))*exp( -0.5*expTerm(0,0) );// (3.46) and  (14.2) on page 459 in IPRP

            //cout << "calculated weigth tmp: " << w_tmp << endl;

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

Matrix6f Particle::sCov = 0.05*Matrix6f::Identity(); // static variable - has to be declared outside class!

boost::mt19937 Particle::rng; // Creating a new random number generator every time could be optimized
//rng.seed(static_cast<unsigned int>(time(0)));
















/* ############################## Defines ParticleSet class ##############################  */

class ParticleSet
{
public:
    /* variables */
    vector<Particle*> Parray;
    Matrix6f sCov;
    unsigned int k; // number of interations since time zero

    /* functions */
    ParticleSet(int Nparticles = 10,Vector6f s0 = Vector6f::Constant(0)); 		/* Initialize a standard particle set with 100 particles */
    ~ParticleSet();
    void updateParticleSet(MeasurementSet* z_Ex, MeasurementSet* z_New, VectorUFastSLAMf u, float Ts);
    Vector6f* getLatestPoseEstimate();
    int getNParticles();

    private:
    /* variables */
    int nParticles;
    Path* sMean;                    // instance of path Class to keep track of the estimated mean of the Particle filter!
    double StartTime;


    /* functions */
    void resample();
    void estimateDistribution();
    void resampleSimple();
};

ParticleSet::ParticleSet(int Nparticles,Vector6f s0){
    k=0;
    sMean = new Path(s0,k); // makes new path to keep track of the estimated mean of the Particle filter!

    nParticles = Nparticles;
    Parray.reserve(nParticles);

    for(int i = 1; i<=nParticles; i++){
        Parray[i] = new Particle;
    }
    StartTime = ros::Time::now().toSec();
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

void ParticleSet::updateParticleSet(MeasurementSet* z_Ex, MeasurementSet* z_New, VectorUFastSLAMf u, float Ts){
    k++;
    for(int i = 1; i<=nParticles;i++){
        //cout << endl << "updating particle: " << i << endl;
        Parray[i]->updateParticle(z_Ex,z_New,&u,k,Ts);
    }

    resample();
    estimateDistribution();
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

    cout << endl << "Current best pose estimate" << endl << *(tmpP->s->getPose()) << endl;

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

        Parraytmp[z] = new Particle(*Parray[index]);
    }

    for(int i = 1; i<=nParticles; i++){
        delete Parray[i];
    }
    for(int i = 1; i<=nParticles; i++){
        Parray[i] = Parraytmp[i];
    }
    //cout << "Done resampling!" << endl;
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

Vector6f* ParticleSet::getLatestPoseEstimate(){
    return sMean->getPose();
}

void ParticleSet::estimateDistribution(){
    double wSum = 0;
    double wNorm = 0;

    for(int i = 1; i<=nParticles;i++){
        wSum = wSum + Parray[i]->w;
    }

    Vector6f sTmp = Vector6f::Zero();
    Vector6f sMean_estimate = Vector6f::Zero();
    double wSum_squared = 0;

    for(int i = 1; i<=nParticles;i++){
        sTmp = *(Parray[i]->s->getPose());
        wNorm = (Parray[i]->w)/wSum;

        sMean_estimate = sMean_estimate + wNorm*sTmp; // weighted mean!
    }

    Matrix6f sCov_estimate = Matrix6f::Zero();
    Matrix6f sCov_estimate_tmp = Matrix6f::Zero();
    Vector6f sDiff = Vector6f::Zero();

    for(int i = 1; i<=nParticles;i++){
        sTmp = *(Parray[i]->s->getPose());
        wNorm = (Parray[i]->w)/wSum;
        sDiff = sTmp-sMean_estimate;

        for(int j = 0; j<=5;j++){
            for(int k = 0; k<=5;k++){
                sCov_estimate_tmp(j,k) = sDiff(j)*sDiff(k)*(float)wNorm; // weighted mean!
            }
        }
        sCov_estimate = sCov_estimate + sCov_estimate_tmp;
        wSum_squared = wSum_squared + wNorm*wNorm;
    }
    sCov_estimate = 1/(1-wSum_squared)*sCov_estimate;

    //cout << endl << "sCov_estimate" << endl << sCov_estimate << endl;
    cout << endl << "sMean_estimate" << endl << sMean_estimate << endl;

    sCov = sCov_estimate;
    sMean->addPose(sMean_estimate,k);
}









int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mtest");
    ros::NodeHandle nh;

    cout << ros::Time::now() << endl;

    // malte playing with particle Sets
    int Nparticles = 50;
    Vector6f s0 = Vector6f::Constant(0);

    ParticleSet* Pset = new ParticleSet(Nparticles,s0);


    VectorUFastSLAMf u = VectorUFastSLAMf::Zero();

    unsigned int k=1;
    unsigned int j=0;
    int Nlandmarks = 50;
    int Nk = 40;

    MeasurementSet* z_Ex = new MeasurementSet;
    MeasurementSet* z_New = new MeasurementSet;
    GOTMeasurement* z_tmp;
    Vector3f V_tmp;
    bool toggle = true;

    float measNoise = 0.0;
    float measSign = -1;


    while (k<=Nk){
        cout << "#################################### k=" << k << " ####################################" << endl;
        delete z_New;
        z_New = new MeasurementSet;
        delete z_Ex;
        z_Ex = new MeasurementSet;

        if(j+10<Nlandmarks){
            if(toggle){
                for(unsigned int i = 1; i<=10; i++){
                    V_tmp = Vector3f::Constant(i)+measNoise*Vector3f::Random();
                    z_tmp = new GOTMeasurement(i,measSign*V_tmp); //generate random numbers...
                    z_New->addMeasurement(z_tmp);
                }
                toggle = false;
            }
            else{
                for(unsigned int i = 1+j; i<=10+j; i++){
                    V_tmp = Vector3f::Constant(i)+measNoise*Vector3f::Random();
                    z_tmp = new GOTMeasurement(i,measSign*V_tmp); //generate random numbers...
                    z_Ex->addMeasurement(z_tmp);

                    V_tmp = Vector3f::Constant(i+10)+measNoise*Vector3f::Random();
                    z_tmp = new GOTMeasurement(i+10,measSign*V_tmp); //generate random numbers...
                    z_New->addMeasurement(z_tmp);
                }
                j = j + 10;
            }
        }
        else{
            unsigned int rand1 = (int)(((float)rand()/(float)RAND_MAX)*(float)Nlandmarks);
            unsigned int rand2 = (int)(((float)rand()/(float)RAND_MAX)*(float)Nlandmarks);

            if(rand1>Nlandmarks){rand1=Nlandmarks;}
            if(rand2>Nlandmarks){rand2=Nlandmarks;}
            if(rand2<rand1){
                unsigned int tmp = rand2;
                rand2=rand1;
                rand1=tmp;
            }

            cout << "random numbers: "<< rand1 << ", " << rand2 << endl;

            for(unsigned int i = rand1; i<=rand2; i++){
                V_tmp = Vector3f::Constant(i)+measNoise*Vector3f::Random();

                z_tmp = new GOTMeasurement(i,measSign*V_tmp); //generate random numbers...
                z_Ex->addMeasurement(z_tmp);
            }
        }
    Pset->updateParticleSet(z_Ex,z_New,u,0);
    //cout << endl << "latest pose" << endl << *(Pset->getLatestPoseEstimate()) << endl << endl;

    cout << "globalLandmarkCounter: " << globalLandmarkCounter << endl;
    cout << "globalMapNodeCounter: " << globalMapNodeCounter << endl;

    k++;
    }

    delete z_New;
    delete z_Ex;
    delete Pset;



    // malte playing with particles
/*
    int Nparticles = 200;
    Particle* Parray[Nparticles];

    for(int i = 1; i<=Nparticles;i++){
        Parray[i] = new Particle;
    }

    VectorUFastSLAMf u = VectorUFastSLAMf::Zero();

    unsigned int k=1;
    unsigned int j=0;
    int Nlandmarks = 100;
    int Nk = 20;

    MeasurementSet* z_Ex = new MeasurementSet;
    MeasurementSet* z_New = new MeasurementSet;
    GOTMeasurement* z_tmp;
    Vector3f V_tmp;
    bool toggle = true;

    float measNoise = 0.001;
    float measSign = -1;


    while (k<=Nk){
        cout << "#################################### k=" << k << " ####################################" << endl;
        delete z_New;
        z_New = new MeasurementSet;
        delete z_Ex;
        z_Ex = new MeasurementSet;

        if(j+10<Nlandmarks){
            if(toggle){
                for(unsigned int i = 1; i<=10; i++){
                    V_tmp = Vector3f::Constant(i)+measNoise*Vector3f::Random();
                    z_tmp = new GOTMeasurement(i,measSign*V_tmp); //generate random numbers...
                    z_New->addMeasurement(z_tmp);

                    //cout << "i" << endl << i << endl;
                    //cout << "V_tmp" << endl << V_tmp << endl;
                }
                toggle = false;
            }
            else{
                for(unsigned int i = 1+j; i<=10+j; i++){
                    V_tmp = Vector3f::Constant(i)+measNoise*Vector3f::Random();
                    z_tmp = new GOTMeasurement(i,measSign*V_tmp); //generate random numbers...
                    z_Ex->addMeasurement(z_tmp);

                    //cout << "i" << endl << i << endl;
                    //cout << "V_tmp" << endl << V_tmp << endl;

                    V_tmp = Vector3f::Constant(i+10)+measNoise*Vector3f::Random();
                    z_tmp = new GOTMeasurement(i+10,measSign*V_tmp); //generate random numbers...
                    z_New->addMeasurement(z_tmp);

                    //cout << "i+10" << endl << i+10 << endl;
                    //cout << "V_tmp" << endl << V_tmp << endl;
                }
                j = j + 10;
            }
        }
        else{
            unsigned int rand1 = (int)(((float)rand()/(float)RAND_MAX)*(float)100);
            unsigned int rand2 = (int)(((float)rand()/(float)RAND_MAX)*(float)100);

            if(rand1>Nlandmarks){rand1=Nlandmarks;}
            if(rand2>Nlandmarks){rand2=Nlandmarks;}
            if(rand2<rand1){
                unsigned int tmp = rand2;
                rand2=rand1;
                rand1=tmp;
            }

            cout << "random numbers: "<< rand1 << ", " << rand2 << endl;

            for(unsigned int i = rand1; i<=rand2; i++){
                V_tmp = Vector3f::Constant(i)+measNoise*Vector3f::Random();


                //cout << "V_tmp" << endl << V_tmp << endl;

                z_tmp = new GOTMeasurement(i,measSign*V_tmp); //generate random numbers...
                z_Ex->addMeasurement(z_tmp);
            }
        }

        //cout << "z_Ex->nMeas: " << z_Ex->nMeas << endl;
        //cout << "z_New->nMeas: " << z_New->nMeas << endl;

        for(int i = 1; i<=Nparticles;i++){
            cout << endl << "updating particle: " << i << endl;
            Parray[i]->updateParticle(z_Ex,z_New,&u,k,0);
        }

        double wTotal=0;
        for(int i = 1; i<=Nparticles;i++){
            wTotal = wTotal + Parray[i]->w;
        }

        for(int i = 1; i<=Nparticles;i++){
            Parray[i]->w = Parray[i]->w/wTotal;
        }


        cout << "P1 weight: " << Parray[1]->getWeigth() << endl;
        cout << " Current s norm:" << endl << (*(Parray[1]->s->getPose())).norm() << endl << endl;

        cout << "P2 weight: " << Parray[2]->getWeigth() << endl;
        cout << " Current s norm:" << endl << (*(Parray[2]->s->getPose())).norm() << endl << endl;

        cout << "P3 weight: " << Parray[3]->getWeigth() << endl;
        cout << " Current s norm:" << endl << (*(Parray[3]->s->getPose())).norm() << endl << endl;

        cout << "P4 weight: " << Parray[4]->getWeigth() << endl;
        cout << " Current s norm:" << endl << (*(Parray[4]->s->getPose())).norm() << endl << endl;

        cout << "P5 weight: " << Parray[5]->getWeigth() << endl;
        cout << " Current s norm:" << endl << (*(Parray[5]->s->getPose())).norm() << endl << endl;

        cout << "globalLandmarkCounter: " << globalLandmarkCounter << endl;
        cout << "globalMapNodeCounter: " << globalMapNodeCounter << endl;

        Particle* tmpPointer;
        double wtmp = 0;

        // primitiv resampling
        //cout << "D1" << endl;
        for(int i = 1; i<=Nparticles;i++){
            if(i==1){
                wtmp = Parray[i]->w;
                tmpPointer = Parray[i];
            }
            else if(Parray[i]->w > wtmp){
                wtmp = Parray[i]->w;
                tmpPointer = Parray[i];
            }
        }

        cout << "Best P weight: " << tmpPointer->getWeigth() << endl;
        cout << " Current s norm:" << endl << (*(tmpPointer->s->getPose())).norm() << endl << endl;

        tmpPointer->map->printAllLandmarkPositions();

        //cout << "D2" << endl;
        Particle* Parray_tmp[Nparticles];
        for(int i = 1; i<=Nparticles;i++){
            Parray_tmp[i] = Parray[i];
        }

        //cout << "D3" << endl;
        for(int i = 1; i<=Nparticles;i++){
            //cout << "D3: " << i << endl;
            Parray[i] = new Particle(*tmpPointer);
        }

        //cout << "D4" << endl;
        for(int i = 1; i<=Nparticles;i++){
            delete Parray_tmp[i];
        }

        //cout << "D5" << endl;
        k++;
    }

    delete z_New;
    delete z_Ex;

    for(int i = 1; i<=Nparticles;i++){
        delete Parray[i];
    }

    cout << "globalLandmarkCounter: " << globalLandmarkCounter << endl;
    cout << "globalMapNodeCounter: " << globalMapNodeCounter << endl;
*/

    // malte playing with Paths
/*
    Path* P = new Path(Vector6f::Constant(1),1);

    for(unsigned int i = 1; i < 100;i++){
        P->addPose(Vector6f::Constant(i),i);
    }

    Path* P2 = new Path(*P); //makes copy of P on the heap
    Path* P3 = new Path(*P); //makes copy of P on the heap

    for(unsigned int i = 100; i < 200;i++){
        P2->addPose(Vector6f::Constant(i),i);
    }

    for(unsigned int i = 200; i < 300;i++){
        P3->addPose(Vector6f::Constant(i),i);
    }

    cout << "references: " << P->PathRoot->nextNode->referenced << endl;
    Vector6f* S = P->getPose(30);
    cout << endl << endl << *S << endl;
    delete P;
    S = P2->getPose(30);
    cout << endl << endl << *S << endl;

    S = P3->getPose(20);
    cout << endl << endl << *S << endl;

    delete P2;
    S = P3->getPose(10);
    cout << endl << endl << *S << endl;

    S = P3->getPose();
    cout << "current pose P3: " << *S << endl;
*/

    // malte testing measurement classes
/*    Vector3f zGOT;
    zGOT << 5,5,5;
    int zGOT_identifier = 0;

    GOTMeasurement z1(zGOT_identifier,zGOT);


    Vector3f zIMG;
    zIMG << 2,2,2;
    int zIMG_identifier = 234;

    ImgMeasurement z2(zIMG_identifier,zIMG);

    cout << z1.z << endl;
    cout << z2.z << endl;

    cout << "making MeasurementSet..." << endl;
    MeasurementSet measSet(z1);

    for( int a = 1; a < 10; a = a + 1 ) {
        measSet.addMeasurement(z2);
    }

    cout << "N Meas:" << endl;
    cout << measSet.getNumberOfMeasurements() << endl;


    cout << "Meas:" << endl;
    Measurement tmpMeas = measSet.getMeasurement();
    cout << tmpMeas.z << endl;
    tmpMeas = measSet.getMeasurement();
    cout << tmpMeas.z << endl;

    cout << "N Meas:" << endl;
    cout << measSet.getNumberOfMeasurements() << endl;
    measSet.deleteMeasurementSet();

    cout << "N Meas:" << endl;
    cout << measSet.getNumberOfMeasurements() << endl;
*/

    // malte testing tree map classes
/*
    MapTree* T = new MapTree;

    unsigned N_landmarks = 2*2*2;

    for(unsigned int i = 1; i<=N_landmarks;i++){
        landmark* newLandmark = new landmark;
        newLandmark->c = i;
        newLandmark->lhat = Vector3f::Constant(i);

        cout << i << endl;
        T->insertLandmark(newLandmark);
    }

    MapTree* T2 = new MapTree(*T); //makes copy of T on the heap
    MapTree* T3 = new MapTree(*T);
    MapTree* T4 = new MapTree(*T);
    MapTree* T5 = new MapTree(*T);

    cout << "T2 N References for root:" << endl;
    cout << T2->root->referenced << endl;

    cout << "Landmarks in T2:" << endl;
    T2->printAllLandmarkPositions();
    cout << "Landmarks in T3:" << endl;
    T3->printAllLandmarkPositions();

    landmark* newLandmark2 = new landmark;
    newLandmark2->c = 1;
    newLandmark2->lhat = Vector3f::Constant(9.1);
    T2->correctLandmark(newLandmark2);
    cout << endl;

    landmark* newLandmark3 = new landmark;
    newLandmark3->c = 2;
    newLandmark3->lhat = Vector3f::Constant(9.2);
    T3->correctLandmark(newLandmark3);

    cout << "Landmarks in T:" << endl;
    T->printAllLandmarkPositions();
    cout << "Landmarks in T2:" << endl;
    T2->printAllLandmarkPositions();
    cout << "Landmarks in T3:" << endl;
    T3->printAllLandmarkPositions();
    cout << "Landmarks in T4:" << endl;
    T4->printAllLandmarkPositions();
    cout << "Landmarks in T5:" << endl;
    T5->printAllLandmarkPositions();

    cout << "globalLandmarkCounter: " << globalLandmarkCounter << endl;
    cout << "globalMapNodeCounter: " << globalMapNodeCounter << endl;

    delete T4;
    delete T5;
    delete T;

    delete T2;

    cout << "globalLandmarkCounter: " << globalLandmarkCounter << endl;
    cout << "globalMapNodeCounter: " << globalMapNodeCounter << endl;
    delete T3;

    cout << "globalLandmarkCounter: " << globalLandmarkCounter << endl;
    cout << "globalMapNodeCounter: " << globalMapNodeCounter << endl;
*/
    //MatrixXd testa = load_csv_to_matrix("text.csv");
    //cout << testa << endl;

    cout << endl << "program ended"<< endl;
  	return 0;
}

