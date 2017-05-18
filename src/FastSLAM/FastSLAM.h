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

#define deg2rad(x)  (x*M_PI)/180.f
#define rad2deg(x)  (x*180.f)/M_PI

#define pi M_PI

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 4, 1> VectorUFastSLAMf; // velocities in the order: [x_dot, y_dot, z_dot, yaw_difference]
typedef Eigen::Matrix<float, 4, 1> VectorChiFastSLAMf; // state vector [x,y,z,yaw]
typedef Eigen::Matrix<float, 4, 4> MatrixChiFastSLAMf; // used for covariance of state vector
typedef Eigen::Matrix<float, 6, Eigen::Dynamic> Matrix6kf;


/* ############################## Defines measurement class ##############################  */
class Measurement
{
public:
    static Eigen::MatrixXf zCov; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/

    /* variables */
    unsigned int c; 	/* measurement identifier - 0 for pose measurement, 1 for GOT and 2...N for landmark identifier */
    Eigen::VectorXf z;	/* actual measurement - can take different sizes! */
    ros::Time timestamp;

    /* functions */
   // Measurement();
   // ~Measurement();
    virtual Eigen::MatrixXf calculateHl(VectorChiFastSLAMf pose, Eigen::Vector3f l) = 0;		/* calculates derivative of measurement model with respect to landmark variable - l */
    virtual Eigen::MatrixXf calculateHs(VectorChiFastSLAMf pose, Eigen::Vector3f l) = 0;		/* calculates derivative of measurement model with respect to pose variable - s */
    virtual Eigen::VectorXf inverseMeasurementModel(VectorChiFastSLAMf pose) = 0;
    virtual Eigen::VectorXf MeasurementModel(VectorChiFastSLAMf pose, Eigen::Vector3f l) = 0;
    virtual Eigen::MatrixXf getzCov() = 0;
private:
};


/* ############################## Defines GOTMeasurement class ##############################  */
class GOTMeasurement : public Measurement
{
    public:
    static Eigen::MatrixXf zCov; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/

    GOTMeasurement(unsigned int i, Eigen::Vector3f GOT_meas);
    Eigen::MatrixXf calculateHs(VectorChiFastSLAMf pose, Eigen::Vector3f l);
    Eigen::MatrixXf calculateHl(VectorChiFastSLAMf pose, Eigen::Vector3f l);
    Eigen::VectorXf inverseMeasurementModel(VectorChiFastSLAMf pose);
    Eigen::VectorXf MeasurementModel(VectorChiFastSLAMf pose, Eigen::Vector3f l);
    Eigen::MatrixXf getzCov();

private:
};


/* ############################## Defines ImgMeasurement class ##############################  */
class ImgMeasurement : public Measurement
{
    public:
    static Eigen::MatrixXf zCov; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/
    float roll;
    float pitch;

    //ImgMeasurement(unsigned int i, Eigen::Vector3f img_me);
    ImgMeasurement(unsigned int i, Eigen::Vector3f img_me,float roll_, float pitch_);
    Eigen::VectorXf inverseMeasurementModel(VectorChiFastSLAMf pose);
    Eigen::MatrixXf calculateHs(VectorChiFastSLAMf pose, Eigen::Vector3f l);
    Eigen::MatrixXf calculateHl(VectorChiFastSLAMf pose, Eigen::Vector3f l);
    Eigen::VectorXf MeasurementModel(VectorChiFastSLAMf pose, Eigen::Vector3f l);
    Eigen::MatrixXf getzCov();

private:
    /* Camera coefficients */
    static constexpr float ax = 308.92944335938; // also known as fx
    static constexpr float ay = 311.72131347656; // also known as fy
    static constexpr float x0 = 160.68548583984; // also known as ppx
    static constexpr float y0 = 126.8815536499; // also known as ppy
};


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
    void emptyMeasurementSet();
    void addMeasurement(Measurement *meas);
    int countNumberOfMeasurements();
    int getNumberOfMeasurements();
    Measurement* getMeasurement(int i);

private:
    void emptyMeasurementSet(Node_MeasurementSet *MeasNode);
};


/* ############################## Defines Maps class ##############################  */
struct landmark
{
  static unsigned int globalLandmarkCounter; // can be used to check if number of landmarks does not grow without bound
  unsigned int c; 		/* landmark identifier */
  Eigen::Vector3f lhat;
  Eigen::Matrix3f lCov;
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
  static unsigned int globalMapNodeCounter; // can be used to check if number of MapNodes does not grow without bound
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

class MapTree
{
    static int mapTreeIdentifierCounter;
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
        void saveData(std::string filename,std::vector<unsigned int> LandmarksToSave);

    private:
        mapNode* makeNewPath(landmark* newLandmarkData, mapNode* startNode);
};


/* ############################## Defines Path class ##############################  */
struct Node_Path {
    VectorChiFastSLAMf S;
    unsigned int k;
    Node_Path *nextNode;
    unsigned int referenced;
};

class Path
{
public:
    static Eigen::IOFormat OctaveFmt;
    static std::ofstream dataFileStream;
    /* variables */
    Node_Path *PathRoot;
    unsigned int PathLength;

    /* functions */
    Path(VectorChiFastSLAMf S, unsigned int k);
    Path(const Path &PathToCopy); // copy constructer
    ~Path();
    void deletePath();
    void addPose(VectorChiFastSLAMf S, unsigned int k);
    unsigned int countLengthOfPath();
    VectorChiFastSLAMf* getPose();
    VectorChiFastSLAMf* getPose(unsigned int k);
    void saveData(std::string filename);

private:
    void deletePath(Node_Path *PathNode);
};


/* ############################## Defines particle class ##############################  */
class Particle
{
public:
    /* variables */
    Path* s;
    MapTree* map;
    double w;
    MatrixChiFastSLAMf s_k_Cov; // particle covariance
    static MatrixChiFastSLAMf sCov; // motion model covariance - does not change?

    static boost::mt19937 rng; // Creating a new random number generator every time could be optimized
    //rng.seed(static_cast<unsigned int>(time(0)));


    /* functions */
    Particle(VectorChiFastSLAMf s0 = VectorChiFastSLAMf::Constant(0), MatrixChiFastSLAMf s_0_Cov = 0.01*MatrixChiFastSLAMf::Identity(), unsigned int k = 0); 		// Initialize a standard particle with "zero-pose" or custom pose
    Particle(const Particle &ParticleToCopy);       // Copy constructer used in case where we need to make a copy of a Particle
    ~Particle();
    void updateParticle(MeasurementSet* z_Ex,MeasurementSet* z_New, VectorUFastSLAMf* u, unsigned int k, float Ts);
    double getWeigth();
    void saveData(std::string filename,std::vector<unsigned int> LandmarksToSave);

private:
    /* variables */

    /* functions */
    VectorChiFastSLAMf drawSampleFromProposaleDistribution(VectorChiFastSLAMf* s_old, VectorUFastSLAMf* u, MeasurementSet* z_Ex, float Ts);
    VectorChiFastSLAMf drawSampleFromProposaleDistributionNEW(VectorChiFastSLAMf* s_old, VectorUFastSLAMf* u,MeasurementSet* z_Ex, float Ts);
    VectorChiFastSLAMf motionModel(VectorChiFastSLAMf sold, VectorUFastSLAMf* u, float Ts);
    void handleExMeas(MeasurementSet* z_Ex, VectorChiFastSLAMf s_proposale);
    void handleNewMeas(MeasurementSet* z_New, VectorChiFastSLAMf s_proposale);
    void updateLandmarkEstimates(VectorChiFastSLAMf s_proposale, MeasurementSet* z_Ex, MeasurementSet* z_New);
    VectorChiFastSLAMf drawSampleRandomPose(VectorChiFastSLAMf sMean_proposale, MatrixChiFastSLAMf sCov_proposale);
    void calculateImportanceWeight(MeasurementSet* z_Ex, VectorChiFastSLAMf s_proposale);
    MatrixChiFastSLAMf calculateFs(VectorChiFastSLAMf *s_k_minor_1);
};



/* ############################## Defines ParticleSet class ##############################  */
class ParticleSet
{
public:
    /* variables */
    std::vector<Particle*> Parray;
    MatrixChiFastSLAMf sCov;
    unsigned int k; // number of interations since time zero
    std::vector<unsigned int> KnownMarkers;

    /* functions */
    ParticleSet(int Nparticles = 10,VectorChiFastSLAMf s0 = VectorChiFastSLAMf::Constant(0), MatrixChiFastSLAMf s_0_Cov = 0.1*MatrixChiFastSLAMf::Identity()); 		/* Initialize a standard particle set with 100 particles */
    ~ParticleSet();
    void updateParticleSet(MeasurementSet* z, VectorUFastSLAMf u, float Ts);
    VectorChiFastSLAMf* getLatestPoseEstimate();
    int getNParticles();
    void saveData();

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

class IIR
{
    public:
        IIR(const float * coeff_a, int no_coeff_a, const float * coeff_b, int no_coeff_b);
        float Filter(float input);

    private:
        Eigen::VectorXf a;
        Eigen::VectorXf b;
        Eigen::VectorXf oldInputs;
        Eigen::VectorXf oldOutputs;
        int aLen;
        int bLen;
};
