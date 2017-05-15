#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>

#include "FastSLAM.h"

using namespace std;
using namespace Eigen;

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







int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mtest");
    ros::NodeHandle nh;

    cout << ros::Time::now() << endl;

    // malte playing with particle Sets
    int Nparticles = 200;
    Vector6f s0 = Vector6f::Constant(0);
    Matrix6f s_0_Cov = 0.01*Matrix6f::Identity();

    ParticleSet* Pset = new ParticleSet(Nparticles,s0,s_0_Cov);


    VectorUFastSLAMf u = VectorUFastSLAMf::Zero();

    unsigned int k=1;
    unsigned int j=0;
    int Nlandmarks = 4;
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

        if(j+2<Nlandmarks){
            if(toggle){
                for(unsigned int i = 1; i<=2; i++){
                    V_tmp = Vector3f::Constant(i)+measNoise*Vector3f::Random();
                    z_tmp = new GOTMeasurement(i,measSign*V_tmp); //generate random numbers...
                    z_New->addMeasurement(z_tmp);
                }
                toggle = false;
            }
            else{
                for(unsigned int i = 1+j; i<=2+j; i++){
                    V_tmp = Vector3f::Constant(i)+measNoise*Vector3f::Random();
                    z_tmp = new GOTMeasurement(i,measSign*V_tmp); //generate random numbers...
                    z_Ex->addMeasurement(z_tmp);

                    V_tmp = Vector3f::Constant(i+2)+measNoise*Vector3f::Random();
                    z_tmp = new GOTMeasurement(i+2,measSign*V_tmp); //generate random numbers...
                    z_New->addMeasurement(z_tmp);
                }
                j = j + 2;
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

    cout << "globalLandmarkCounter: " << landmark::globalLandmarkCounter << endl;
    cout << "globalMapNodeCounter: " << mapNode::globalMapNodeCounter << endl;

    k++;
    }

    Pset->saveData();

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

