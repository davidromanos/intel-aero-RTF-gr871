/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>

#include "ekf.h"
#include "ekf_terminate.h"
#include "ekf_initialize.h"
#include "ekf_types.h"


#include "utils.h"

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "rtwtypes.h"




mavros_msgs::State current_state;
geometry_msgs::PoseStamped position;
geometry_msgs::Twist twist;
sensor_msgs::Imu imuData;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    imuData = *msg;
}


class Derivative{
    public:
        Derivative(double updateRate); //constructor
        void Update(double input);
        double state;
    private:
        double updateRate;
        double oldInput;
};

Derivative::Derivative(double updateRate)
{
    this->state = 0;
    this->oldInput = 0;
    this->updateRate = updateRate;
}

void Derivative::Update(double input)
{
    this->state = (input-oldInput)*updateRate;
    oldInput = input;
}


class Integrator{
   public:
    Integrator(double initValue,double updateRate); // constructor
    double state;
    void Update(double input);
   private:
    double updateRate;

};

Integrator::Integrator(double initValue,double updateRate)
{
    this->state=initValue;
    this->updateRate = updateRate;
}
void Integrator::Update(double input)
{
    this->state=this->state + this->updateRate*input;

}
tf::Quaternion q1;

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    position = *msg;
}

void twist_cb(const geometry_msgs::Twist::ConstPtr& msg){
    twist = *msg;
}

std::vector<double> vectorAdd(std::vector<double> a,std::vector<double> b)
{
    std::vector<double> tmp;
    tmp.assign(a.size(),0);
    uint i = 0;
    for(i = 0;i<a.size();i++)
    {
        tmp[i] = a[i] + b[i];
    }
    return tmp;
}

class Matrix{
 public:
    std::vector<std::vector<double> > data;
    Matrix(uint8_T cols,uint8_T rows);
    void setEntry(double value,uint8_T col,uint8_T row);
    double getEntry(uint8_T col,uint8_T row);
    void printMatrix();
    std::vector<double> multiplyVector(std::vector<double> input);
    uint8_T cols,rows;
};
Matrix::Matrix(uint8_T cols, uint8_T rows)
{
    std::vector<std::vector<double> > tmp(cols, std::vector<double>(rows));
    this->data = tmp;
    this->cols = cols;
    this->rows = rows;
}
void Matrix::setEntry(double value,uint8_T col, uint8_T row)
{
    if(col<this->cols && row < this->rows)
    {
        data[col][row] = value;
    }
    else
    {
        ROS_ERROR("Assigning values to a non existing entry Matrix::setEntry()");
    }
}
double Matrix::getEntry(uint8_T col, uint8_T row)
{
    if(col<this->cols && row < this->rows)
    {
        return data[col][row];
    }
    else
    {
        ROS_ERROR("Assigning values to a non existing entry Matrix::setEntry()");
        return -1;
    }
}
void Matrix::printMatrix()
{
    uint16_T i = 0;
    uint16_T k = 0;
    for(i=0;i<this->cols;i++)
    {
        for(k=0;k<this->rows;k++)
        {
            std::cout << this->data[i][k] << " ";
        }
        std::cout << "\n";
    }
}
std::vector<double> Matrix::multiplyVector(std::vector<double> input)
{
    uint16_T i = 0;
    uint16_T k = 0;
    std::vector<double> output;
    output.assign(this->data.size(),0); // create output vector with the samme number of rows as the matrix
    if(input.size() == this->data[0].size())
    {
        for(i=0;i<this->cols;i++)
        {
            for(k=0;k<this->rows;k++)
            {
                output[i] += this->data[i][k]*input[k];
                //std::cout << i << " " << this->data[i][k] << "*" << input[k] << " " << output[i] << "\n";
            }
        }
    }
    return output;
}

class Zcontroller{
    public:
        double update(double setpoint,double *estimatedZStates);
        Zcontroller(double updateRate); //constructor
        std::vector<double> thrust;
        Matrix gainZcontroller;
        double integratorGainZ;
    private:
        Integrator integrator;
        double zerror;

        double updateRate;        
        double estimatedZStates[4];
        std::vector<double> zStates;


};
Zcontroller::Zcontroller(double updateRate) :
    // Member initializer list //
    integrator(0.0,updateRate),
    gainZcontroller(1,3)

{   // Contructor decleration //
    this->updateRate = updateRate;
    this->zerror = 0.0;
    this->integratorGainZ = -1*-0.3368;
    gainZcontroller.setEntry(-1*0.8974,0,0);
    gainZcontroller.setEntry(-1*1.1745,0,1);
    gainZcontroller.setEntry(-1*0.8963,0,2);
    thrust.assign(1,0);
    zStates.assign(3,0);
}

double Zcontroller::update(double setpoint,double *estimatedZStates)
{
    zStates[0] = estimatedZStates[13]; //note the observer is outputting the estimated output and the estimated states
    zStates[1] = estimatedZStates[14];
    zStates[2] = estimatedZStates[15];
    zerror = setpoint - estimatedZStates[13];
    integrator.Update(zerror);

    thrust = gainZcontroller.multiplyVector(zStates);
    thrust[0] = thrust[0] + integrator.state*integratorGainZ + 0.587;
    if(thrust[0]>1.0)
    {
        thrust[0] = 1.0;
    }
    else if(thrust[0]<0.0)
    {
        thrust[0] = 0.0;
    }
    return thrust[0];
}
class stateFeedbackController{
    public:
        void update(double setpoint[],double *meas);
        std::vector<double> output;
        Matrix K0;
        Matrix K0integrators;
        Matrix Kx;
        Matrix Ky;
        stateFeedbackController(double updateRate); //constructor
    private:
        double updateRate;
        Integrator xIntegrator;
        Integrator yIntegrator;
        std::vector<double> Xstates;
        std::vector<double> Ystates;
        std::vector<double> XY;
        std::vector<double> error;
};
stateFeedbackController::stateFeedbackController(double updateRate) :
    // Member initializer list //
    K0(2,2),
    K0integrators(2,2),
    Kx(1,5),
    Ky(1,5),
    xIntegrator(0,updateRate),
    yIntegrator(0,updateRate)

{   // Contructor decleration //
    this->updateRate = updateRate;
    output.assign(2,0);

    Xstates.assign(5,0);
    Ystates.assign(5,0);
    XY.assign(2,0);

    error.assign(2,0);
}

void stateFeedbackController::update(double setpoint[],double *meas)
{
    error[0] = setpoint[0]- meas[0];
    error[1] = setpoint[1]- meas[1];

    /*if(error[0] > 1) error[0] = 1;
    if(error[0] < -1) error[0] = -1;
    if(error[1] > 1) error[1] = 1;
    if(error[1] < -1) error[1] = -1;*/

    Xstates[0] = meas[2];
    Xstates[1] = meas[3];
    Xstates[2] = meas[4];
    Xstates[3] = meas[5];
    Xstates[4] = meas[6];
    Ystates[0] = meas[7];
    Ystates[1] = meas[8];
    Ystates[2] = meas[9];
    Ystates[3] = meas[10];
    Ystates[4] = meas[11];


    Matrix tmpK0(2,2);
    /*tmpK0.setEntry(K0.getEntry(0,0)*cos(meas[12]),0,0);
    tmpK0.setEntry(K0.getEntry(0,0)*-1*sin(meas[12]),0,1);
    tmpK0.setEntry(K0.getEntry(1,1)*sin(meas[12]),1,0);
    tmpK0.setEntry(K0.getEntry(1,1)*cos(meas[12]),1,1);*/

    tmpK0.setEntry(K0.getEntry(0,0)*cos(meas[12]),0,0);
    tmpK0.setEntry(K0.getEntry(0,0)*sin(meas[12]),0,1);
    tmpK0.setEntry(K0.getEntry(1,1)*-1*sin(meas[12]),1,0);
    tmpK0.setEntry(K0.getEntry(1,1)*cos(meas[12]),1,1);




    //output = vectorAdd(tmpK0.multiplyVector(error),output);
    XY[0] = meas[0];
    XY[1] = meas[1];
    Matrix tmpK0Integrators(2,2);

    /*tmpK0Integrators.setEntry(K0integrators.getEntry(0,0)*cos(meas[12]),0,0);
    tmpK0Integrators.setEntry(K0integrators.getEntry(0,0)*-1*sin(meas[12]),0,1);
    tmpK0Integrators.setEntry(K0integrators.getEntry(1,1)*sin(meas[12]),1,0);
    tmpK0Integrators.setEntry(K0integrators.getEntry(1,1)*cos(meas[12]),1,1);*/

    tmpK0Integrators.setEntry(K0integrators.getEntry(0,0)*cos(meas[12]),0,0);
    tmpK0Integrators.setEntry(K0integrators.getEntry(0,0)*sin(meas[12]),0,1);
    tmpK0Integrators.setEntry(K0integrators.getEntry(1,1)*-1*sin(meas[12]),1,0);
    tmpK0Integrators.setEntry(K0integrators.getEntry(1,1)*cos(meas[12]),1,1);


    xIntegrator.Update(error[0]);
    yIntegrator.Update(error[1]);
    error[0] = xIntegrator.state;
    error[1] = yIntegrator.state;
    //std::cout << "x state:  " << xIntegrator.state << "  y state:  " << yIntegrator.state<< "\n";
    std::vector<double>tmp;

    tmp = Kx.multiplyVector(Xstates);
    output[0] = tmp[0];

    tmp = Ky.multiplyVector(Ystates);
    output[1] =tmp[0];
    output = vectorAdd(tmpK0Integrators.multiplyVector(error),output);
    output = vectorAdd(tmpK0.multiplyVector(XY),output);


    //if(output[0] > 0.6) output[0] = 0.6;
    //if(output[0] < -0.6) output[0] = -0.6;
}

double yawReference(double x,double y)
{
    double output = 0;
    double norm = sqrt(x*x + y*y);

    if(y>=0)
    {
        output = acos(x/norm);
    }
    else
    {
        output = -1*acos(x/norm);
    }
    //std::cout << "norm:  " << norm << "  x:  " << x << "  y:  " << y << "  out:  " << output << "\n";
    return output;
}



double dotProduct(std::vector<double> a,std::vector<double> b)
{
    uint8_T i = 0;
    double dotProduct = 0;
    for(i=0;i<a.size();i++)
    {
        dotProduct += a[i]*b[i];
    }
    return dotProduct;
}

struct waypoint
{
    double x,y,z;
    waypoint(double x,double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    waypoint()
    {
        x = 0;
        y = 0;
        z = 0;
    }
};

int main(int argc, char **argv)
{    
    Zcontroller zcontroller(0.05);

    /////////////////  xy CONTROLLER ///////////////
    stateFeedbackController xyController(0.05);


    xyController.K0.setEntry(-1*0.2099,0,0);
    xyController.K0.setEntry(0,0,1);
    xyController.K0.setEntry(0,1,0);
    xyController.K0.setEntry(-1*-0.2088,1,1);

    xyController.Kx.setEntry(-1*0.3267,0,0);
    xyController.Kx.setEntry(-1*0.1234,0,1);
    xyController.Kx.setEntry(-1*-0.0477,0,2);
    xyController.Kx.setEntry(-1*-0.0423,0,3);
    xyController.Kx.setEntry(-1*-0.0122,0,4);

    xyController.Ky.setEntry(-1*-0.3231,0,0);
    xyController.Ky.setEntry(-1*0.1209,0,1);
    xyController.Ky.setEntry(-1*-0.0357,0,2);
    xyController.Ky.setEntry(-1*-0.0005,0,3);
    xyController.Ky.setEntry(-1*0.0076,0,4);

    xyController.K0integrators.setEntry(-1*-0.0664,0,0);
    xyController.K0integrators.setEntry(0,0,1);
    xyController.K0integrators.setEntry(0,1,0);
    xyController.K0integrators.setEntry(-1*0.0664,1,1);



    ////////////////////////////////////////////////
    zcontroller.integratorGainZ = -1*-0.3368;
    zcontroller.gainZcontroller.setEntry(-1*0.8974,0,0);
    zcontroller.gainZcontroller.setEntry(-1*1.1745,0,1);
    zcontroller.gainZcontroller.setEntry(-1*0.8963,0,2);

    double estimatedStates[19];

    std::vector<waypoint> listOfWaypoints;
    listOfWaypoints.push_back(waypoint(0,0,1));
    listOfWaypoints.push_back(waypoint(2,0,2));
    listOfWaypoints.push_back(waypoint(0,1,1));



    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/mocap/pose", 10, pos_cb);
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>
            ("twist", 10, twist_cb);

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, imu_cb);


    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher attitude_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);


    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float64>
            ("mavros/setpoint_attitude/att_throttle", 10);



    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ekf_initialize();

    double roll, pitch, yaw;
    double imuRoll, imuPitch,imuYaw;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    int k = 0;
    int i = 0;
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    double setpoints[3] = {0,0,1};
    double fastslamMeas[6];
    double covariansfastslam[36] = {1,0,0,0,0,0,
                                    0,1,0,0,0,0,
                                    0,0,1,0,0,0,
                                    0,0,0,1,0,0,
                                    0,0,0,0,1,0,
                                    0,0,0,0,0,1};
    double imuMeas[2] = {0,0};
    double gyroMeas[3] = {0,0,0};
    geometry_msgs::PoseStamped pose;

    waypoint currentWaypoint,oldWaypoint;

    currentWaypoint = listOfWaypoints[0];
    oldWaypoint = listOfWaypoints[listOfWaypoints.size()-1];

    setpoints[0] = currentWaypoint.x;
    setpoints[1] = currentWaypoint.y;
    setpoints[2] = currentWaypoint.z;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    pose.pose.orientation.w = 0;
    pose.pose.orientation.z = 1;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.x = 0;
    std_msgs::Float64 thrustInput;
    thrustInput.data = 0.0;


    double yawRef = 0;

    //send a few setpoints before starting
    for(int i = 30; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_request1 = ros::Time::now();


    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if(current_state.mode == "OFFBOARD" && current_state.armed)//&& ros::Time::now()-last_request1 > ros::Duration(2.0)
        {

            q1.setW(position.pose.orientation.w);
            q1.setX(position.pose.orientation.x);
            q1.setY(position.pose.orientation.y);
            q1.setZ(position.pose.orientation.z);

            tf::Matrix3x3 m(q1);

            m.getRPY(roll, pitch, yaw);

            q1.setW(imuData.orientation.w);
            q1.setX(imuData.orientation.x);
            q1.setY(imuData.orientation.y);
            q1.setZ(imuData.orientation.z);
            m.setRotation(q1);
            m.getRPY(imuRoll,imuPitch,imuYaw);
            //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

            fastslamMeas[0] = position.pose.position.x;
            fastslamMeas[1] = position.pose.position.y;
            fastslamMeas[2] = position.pose.position.z;
            fastslamMeas[3] = roll;
            fastslamMeas[4] = pitch;
            fastslamMeas[5] = yaw;

            imuMeas[0] = roll;
            imuMeas[1] = pitch;
            gyroMeas[0] = imuData.angular_velocity.x;
            gyroMeas[1] = imuData.angular_velocity.y;
            gyroMeas[2] = imuData.angular_velocity.z;

            gyroMeas[0] = 0;
            gyroMeas[1] = 0;
            gyroMeas[2] = 0;




            yawRef = yawReference(currentWaypoint.x-oldWaypoint.x,currentWaypoint.y-oldWaypoint.y);

            //ekf(1,fastslamMeas,covariansfastslam,imuMeas,gyroMeas,xyController.output[1],xyController.output[0],yawRef-1*M_PI/2,zcontroller.thrust[0],estimatedStates);
            ekf(1,fastslamMeas,covariansfastslam,imuMeas,gyroMeas,xyController.output[1],xyController.output[0],yawRef,zcontroller.thrust[0],estimatedStates);
            //ekf(1,fastslamMeas,covariansfastslam,imuMeas,xyController.output[1],xyController.output[0],yawRef-1*M_PI/2,zcontroller.thrust[0],estimatedStates);
            //estimatedStates[12] = yaw;


            xyController.update(setpoints,estimatedStates);



            k++;
            if(k%300 == 0)
            {
                i++;
                if(i>listOfWaypoints.size()-1)
                {
                    i = 0;
                }
                oldWaypoint = currentWaypoint;
                currentWaypoint = listOfWaypoints[i];

            }

            if(abs(yaw-yawRef)< 0.3)
            {
                setpoints[0] = currentWaypoint.x;
                setpoints[1] = currentWaypoint.y;
                setpoints[2] = currentWaypoint.z;
            }

            //setpoints[0] = 0;
            //setpoints[1] = 0;
           // std::cout <<"estx:  "<< estimatedStates[0] <<"  xout:  "<< xyController.output[0] <<"esty:  "<< estimatedStates[1]<<"   yout:   "<< xyController.output[1] << "\n";
            //q1.setRPY(xyController.output[1],xyController.output[0],M_PI/4);
            q1.setRPY(xyController.output[1],xyController.output[0],yawRef);

            //q1.setRPY(xyController.output[0],xyController.output[1],yawReference(setpoints[0],setpoints[1]));
            //q1.setRPY(0,0,yawReference(setpoints[0],setpoints[1]));
            //std::cout << yawReference(setpoints[0],setpoints[1]) << "  yaw:"  << yaw << "\n";
            //q1.setRPY(ydotController.output[0],xdotController.output[0],0);
            //q1.setEuler(0.0,0.1,0.1);

            //q1.setRPY(0.0,0.0,0.0);
            pose.pose.orientation.x = q1.getX();
            pose.pose.orientation.y = q1.getY();
            pose.pose.orientation.z = q1.getZ();
            pose.pose.orientation.w = q1.getW();

            thrustInput.data = zcontroller.update(setpoints[2],estimatedStates); // the z states is the last states

            //local_pos_pub.publish(pose);

        }
        else
        {
            last_request1 = ros::Time::now();
        }
        attitude_pub.publish(pose);


        thrust_pub.publish(thrustInput);
        gyroMeas[0] = imuData.angular_velocity.x;
        gyroMeas[1] = imuData.angular_velocity.y;
        gyroMeas[2] = imuData.angular_velocity.z;
        logToFile("/home/chris/Dropbox/P8 (CA2)/Controller/logs/controllerlog.txt","%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",estimatedStates[0],estimatedStates[1],estimatedStates[2],estimatedStates[3],estimatedStates[4],estimatedStates[5],estimatedStates[6],estimatedStates[7],estimatedStates[8],estimatedStates[9],estimatedStates[10],estimatedStates[11],estimatedStates[12],estimatedStates[13],estimatedStates[14],estimatedStates[15],position.pose.position.x,position.pose.position.y,position.pose.position.z,pitch,roll,yaw,xyController.output[0],xyController.output[1],zcontroller.thrust[0],setpoints[0],setpoints[1],setpoints[2],twist.linear.x,twist.linear.y,twist.linear.z,imuPitch,imuRoll,gyroMeas[0],gyroMeas[1],gyroMeas[2],yawRef);
        last_request1 = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
    ekf_terminate();
}

