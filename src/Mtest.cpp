#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>



#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <math.h>

using namespace std;
using namespace Eigen;



typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 6, 6> Matrix6f;
typedef Matrix<float, 6, Dynamic> Matrix6kf;


/* ############################## Defines measurement class ##############################  */

class Measurement
{
public:
    static MatrixXf zCov; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/

    /* variables */
    unsigned int c; 	/* measurement identifier - 0 for pose measurement, 1 for GOT and 2...N for landmark identifier */
    Vector3f z;	/* actual measurement - can take different sizes! */
    ros::Time timestamp;

    /* functions */
   // Measurement();
   // ~Measurement();
    MatrixXf calculateHl();		/* calculates derivative of measurement model with respect to landmark variable - l */
    MatrixXf calculateHs(Vector6f pose,Vector3f l);		/* calculates derivative of measurement model with respect to pose variable - s */
    VectorXf inverseMeasurementModel(Vector6f pose);
private:
};

/* ############################## Defines GOTMeasurement class ##############################  */
class GOTMeasurement : public Measurement
{
    public:
    static Matrix3f zCov; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/

    GOTMeasurement(unsigned int i, Vector3f GOT_meas);
    VectorXf inverseMeasurementModel(Vector6f pose);
    MatrixXf calculateHs();
    Matrix3f calculateHl();

private:
};

GOTMeasurement::GOTMeasurement(unsigned int i, Vector3f GOT_meas)
{
    c = i;
    z << GOT_meas;
    timestamp = ros::Time::now();
}

VectorXf GOTMeasurement::inverseMeasurementModel(Vector6f pose)
{
    Vector6f s = pose; // temp variable to make it look like equations
    VectorXf l = s.topRows<3>() - z;
    return l;
}

MatrixXf GOTMeasurement::calculateHs()
{    
    MatrixXf Hs(3, 6);
    Hs << Matrix3f::Identity(3,3), Matrix3f::Zero(3,3);
    return Hs;
}

Matrix3f GOTMeasurement::calculateHl()
{
    //s = pose; // temp variable to make it look like equations
    Matrix3f Hl = Matrix3f::Identity();
    return Hl;
};

Matrix3f GOTMeasurement::zCov = 0.1*Matrix3f::Identity(); // static variable - has to be declared outside class!


/* ############################## Defines ImgMeasurement class ##############################  */
class ImgMeasurement : public Measurement
{
    public:
    static Matrix3f zCov; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/

    ImgMeasurement(unsigned int i, Vector3f img_meas);
    VectorXf inverseMeasurementModel(Vector6f pose);
    MatrixXf calculateHs();
    Matrix3f calculateHl();

private:
};

ImgMeasurement::ImgMeasurement(unsigned int i, Vector3f img_meas)
{
    c = i;
    z << img_meas;
    timestamp = ros::Time::now();
}

VectorXf ImgMeasurement::inverseMeasurementModel(Vector6f pose)
{
    Vector6f s = pose; // temp variable to make it look like equations
    VectorXf l = s.topRows<3>() - z;
    return l;
}

MatrixXf ImgMeasurement::calculateHs()
{
    MatrixXf Hs(3, 6);
    Hs << Matrix3f::Identity(3,3), Matrix3f::Zero(3,3);
    return Hs;
}

Matrix3f ImgMeasurement::calculateHl()
{
    //s = pose; // temp variable to make it look like equations
    Matrix3f Hl = Matrix3f::Identity();
    return Hl;
};

Matrix3f ImgMeasurement::zCov = 5*Matrix3f::Identity(); // static variable - has to be declared outside class!






/* ############################## Defines measurement set class ##############################  */
struct Node_MeasurementSet {
    Measurement meas;
    Node_MeasurementSet *nextNode;
};

class MeasurementSet
{
public:
    /* variables */
    Node_MeasurementSet *firstMeasNode;
    int nMeas;

    /* functions */
    MeasurementSet(Measurement &meas);
    ~MeasurementSet();
    void deleteMeasurementSet();
    void addMeasurement(Measurement &meas);
    int countNumberOfMeasurements();
    int getNumberOfMeasurements();
    Measurement getMeasurement();

private:
    void deleteMeasurementSet(Node_MeasurementSet *MeasNode);
};

MeasurementSet::MeasurementSet(Measurement &meas){

    firstMeasNode = new Node_MeasurementSet;
    firstMeasNode->meas = meas;
    firstMeasNode->nextNode = NULL;
    nMeas = 1;
}

MeasurementSet::~MeasurementSet(){
    deleteMeasurementSet();
}

void MeasurementSet::deleteMeasurementSet(){
    if(firstMeasNode->nextNode != NULL){
        deleteMeasurementSet(firstMeasNode->nextNode);
    }
    delete firstMeasNode;
    nMeas--;
}

void MeasurementSet::deleteMeasurementSet(Node_MeasurementSet *MeasNode){
    if(MeasNode->nextNode != NULL){
        deleteMeasurementSet(MeasNode->nextNode);
    }
    delete MeasNode;
    nMeas--;
}

void MeasurementSet::addMeasurement(Measurement &meas){

    Node_MeasurementSet* tmp_pointer = firstMeasNode;

    while(tmp_pointer->nextNode != NULL){
        tmp_pointer = tmp_pointer->nextNode;
    }

    tmp_pointer->nextNode = new Node_MeasurementSet;
    tmp_pointer->nextNode->meas = meas;
    tmp_pointer->nextNode->nextNode = NULL;
    nMeas++;
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

Measurement MeasurementSet::getMeasurement(){
    Measurement tmp_meas = firstMeasNode->meas;
    Node_MeasurementSet* tmp_measNodePointer = firstMeasNode;
    firstMeasNode = firstMeasNode->nextNode;

    delete tmp_measNodePointer;
    nMeas--;
    return tmp_meas;
}





/* ############################## Defines Maps class ##############################  */

struct landmark
{
  unsigned int c; 		/* landmark identifier */
  Vector3f lhat;
  Matrix3f lCov;
};

struct mapNode
{
  unsigned int key_value;    /* landmark identifier */
  mapNode *left;               /* pointer for the left node */
  mapNode *right;              /* pointer for the right node */
  landmark *l;              /* pointer for a landmark; is used when *left == NULL or *right == NULL */
  unsigned int referenced;  /* how many nodes/paticles points to this node? if zero the node should be deleted! */
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
    MapToCopy.root->referenced++;

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
    cout << "deleting MapTree: " << mapTreeIdentifier << " References to root: " << root->referenced << " Debugging: ";
    removeReferenceToSubTree(root);
    cout << endl;
}

void MapTree::removeReferenceToSubTree(mapNode* nodeToStartFrom){
    if ((nodeToStartFrom != NULL) && nodeToStartFrom->referenced != 0){
        nodeToStartFrom->referenced--;
    }
    if(nodeToStartFrom->referenced < 1){ // we have to delete the node! since the nodeToStartFrom
        cout << "D50 ";
        //if( (nodeToStartFrom->left != NULL) && (nodeToStartFrom->left->referenced <= 1)){
        if(nodeToStartFrom->left != NULL){
            cout << "D51 ";
            removeReferenceToSubTree(nodeToStartFrom->left);
        }
        //if( (nodeToStartFrom->right != NULL) && (nodeToStartFrom->right->referenced <= 1)){
        if(nodeToStartFrom->right != NULL){
            cout << "D52 ";
            removeReferenceToSubTree(nodeToStartFrom->right);
        }
        if (nodeToStartFrom->key_value == 0){ // we are at a leaf node and have to delete the landmark
            cout << "D53 ";
            delete nodeToStartFrom->l;
        }
        cout << "D55 ";
        delete nodeToStartFrom;
        nodeToStartFrom = NULL;
    }
    else{ // we should not delete the node!
        cout << "D54 ";
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
        cout << "D1: N" << N_nodes << " keyvalue: " << root->key_value << endl;
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
                    cout << "D:R" << endl;
                }
                else{ //tmpMapNodePointer->right != NULL does not point to anything we have to creat a new node!
                    tmpMapNodePointer->right = new mapNode;
                    tmpMapNodePointer->right->key_value = tmpMapNodePointer->key_value + i2;
                    tmpMapNodePointer->right->left = NULL;
                    tmpMapNodePointer->right->right = NULL;
                    tmpMapNodePointer->right->l = NULL;
                    tmpMapNodePointer->right->referenced = 1;
                    N_nodes++;
                    cout << "D2: N" << N_nodes << " keyvalue: " << tmpMapNodePointer->right->key_value <<endl;

                    tmpMapNodePointer=tmpMapNodePointer->right;
                }
            }
            else if(newLandmark->c <= tmpMapNodePointer->key_value){
                if(tmpMapNodePointer->left != NULL){
                    tmpMapNodePointer=tmpMapNodePointer->left;
                    cout << "D:L" << endl;
                }
                else{ //tmpMapNodePointer->right != NULL does not point to anything we have to creat a new node!
                    tmpMapNodePointer->left = new mapNode;
                    tmpMapNodePointer->left->key_value = tmpMapNodePointer->key_value - i2;
                    tmpMapNodePointer->left->left = NULL;
                    tmpMapNodePointer->left->right = NULL;
                    tmpMapNodePointer->left->l = NULL;
                    tmpMapNodePointer->left->referenced = 1;
                    N_nodes++;
                    cout << "D3: N" << N_nodes << " keyvalue: " << tmpMapNodePointer->left->key_value << endl;

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
            cout << "D5: Created new leaf to the right!" << endl;
        }
        else if(newLandmark->c <= tmpMapNodePointer->key_value){ // we go to the left
            tmpMapNodePointer->left = pointerForNewLeafNode;
            cout << "D5: Created new leaf to the left!" << endl;
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
         cout << "D4: N" << N_nodes << " keyvalue: " << newRootNode->key_value << endl;

         root = newRootNode;
         i++;
     }

     cout << "Added layer. N_layers:" << Needed_N_layers << endl;
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
    cout << "D20 ";
    mapNode* tmpMapNode = makeNewPath(newLandmarkData, root);

    removeReferenceToSubTree(root);

    root = tmpMapNode;
    cout << "D29 ";
}

mapNode* MapTree::makeNewPath(landmark* newLandmarkData, mapNode* startNode){
    cout << "D21 ";
    if(startNode->key_value > 0){
        cout << "D22 ";

        // we need to make a new MapNode
        mapNode* pointerForNewMapNode = new mapNode;
        pointerForNewMapNode->l = NULL;
        pointerForNewMapNode->referenced = 1;

        if (newLandmarkData->c > startNode->key_value){ // we go right
            cout << "D23 ";
            pointerForNewMapNode->left = startNode->left; // and do not change the left pointer
            pointerForNewMapNode->left->referenced++;
            pointerForNewMapNode->key_value = startNode->key_value; // the new node has the same key_value as the old
            pointerForNewMapNode->right = makeNewPath(newLandmarkData,startNode->right);

            //removeReferenceToSubTree(startNode); // we have to delete the SubTree if there is no more references for it!
        }
        else if(newLandmarkData->c <= startNode->key_value){ // we go left
            cout << "D24 ";
            pointerForNewMapNode->right = startNode->right; // and do not change the right pointer
            pointerForNewMapNode->right->referenced++;
            pointerForNewMapNode->key_value = startNode->key_value; // the new node has the same key_value as the old
            pointerForNewMapNode->left = makeNewPath(newLandmarkData,startNode->left);

            //removeReferenceToSubTree(startNode); // we have to delete the SubTree if there is no more references for it!
        }
        else{
            cout << "error in makeNewPath";
        }
        cout << "D25 ";
        return pointerForNewMapNode;
    }
    else{ // we have reached the bottom of the tree and should make a new mapNode to hold the pointer for the updated landmark data
        cout << "D26 ";
        mapNode* pointerForNewLeafNode = new mapNode;
        pointerForNewLeafNode->key_value = 0;
        pointerForNewLeafNode->left = NULL;
        pointerForNewLeafNode->right = NULL;
        pointerForNewLeafNode->l = newLandmarkData;
        pointerForNewLeafNode->referenced = 1;

        //removeReferenceToSubTree(startNode); // we have to delete the SubTree if there is no more references for it!
        cout << "D27 ";
        return pointerForNewLeafNode;
    }
    cout << "D28 ";
}

void MapTree::printAllLandmarkPositions(){
    //cout << endl << "D30 ";

    MatrixXf tmpMatrix(3,N_Landmarks);
    for(unsigned int i = 1;i<=N_Landmarks;i++){
        //cout << "D31 ";
        if (extractLandmarkNodePointer(i) != NULL){
            //cout << "D32 ";
            tmpMatrix.col(i-1) = extractLandmarkNodePointer(i)->lhat;
        }
        else{
            tmpMatrix.col(i-1) = Vector3f::Zero();
            cout<<"Error: NULL pointer!";
        }
    }
    //cout << "D33 ";
    cout << endl << tmpMatrix << endl;
}























int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mtest");
    ros::NodeHandle nh;


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
    MapTree* T = new MapTree;

    unsigned N_landmarks = 2*2*2;

    for(unsigned int i = 1; i<=N_landmarks;i++){
        landmark* newLandmark = new landmark;
        newLandmark->c = i;
        newLandmark->lhat = Vector3f::Constant(i);

        cout << i << endl;
        T->insertLandmark(newLandmark);
    }

    //MapTree* T2 = new MapTree;
    //T2 = T; //makes copy of T

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


    delete T4;
    delete T5;
    delete T;

    delete T2;
    delete T3;

    cout << endl << "program ended"<< endl;


  	return 0;
}

