/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>


#include "observer_z.h"
#include "observer_z_terminate.h"
#include "observer_z_initialize.h"
#include "observer_z_types.h"

#include "observer_xdot.h"
#include "observer_xdot_terminate.h"
#include "observer_xdot_initialize.h"
#include "observer_xdot_types.h"

#include "observer_ydot.h"
#include "observer_ydot_terminate.h"
#include "observer_ydot_initialize.h"
#include "observer_ydot_types.h"

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "rtwtypes.h"




mavros_msgs::State current_state;
geometry_msgs::PoseStamped position;
geometry_msgs::Twist twist;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
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
    if(this->state > 20)
    {
        this->state = 20;
    }
    if(this->state < -20)
    {
        this->state = -20;
    }

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
        double update(double setpoint,double meas);
        Zcontroller(double updateRate); //constructor
    private:
        Integrator integrator;
        double zerror;
        double integratorGainZ;
        double updateRate;
        Matrix gainZcontroller;
        double estimatedZStates[4];
        std::vector<double> zStates;
        std::vector<double> thrust;

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
    observer_z_initialize();
}

double Zcontroller::update(double setpoint,double zmeas)
{
    observer_z(zmeas,thrust[0]- 0.587,estimatedZStates);

    zStates[0] = estimatedZStates[1]; //note the observer is outputting the estimated output and the estimated states
    zStates[1] = estimatedZStates[2];
    zStates[2] = estimatedZStates[3];
    zerror = setpoint - estimatedZStates[0];
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
        Matrix Kx;
        Matrix Ky;
        stateFeedbackController(double updateRate); //constructor
    private:
        double updateRate;
        std::vector<double> Xstates;
        std::vector<double> Ystates;
        std::vector<double> error;
};
stateFeedbackController::stateFeedbackController(double updateRate) :
    // Member initializer list //
    K0(2,2),
    Kx(1,5),
    Ky(1,5)
{   // Contructor decleration //
    this->updateRate = updateRate;
    output.assign(2,0);

    Xstates.assign(5,0);
    Ystates.assign(5,0);

    error.assign(2,0);
}

void stateFeedbackController::update(double setpoint[],double *meas)
{
    error[0] = meas[0] - setpoint[0];
    error[1] = meas[1] - setpoint[1];
    if(error[0] > 1) error[0] = 1;
    if(error[0] < -1) error[0] = -1;
    if(error[1] > 1) error[1] = 1;
    if(error[1] < -1) error[1] = -1;

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


    /*Matrix rot(2,2);
    rot.setEntry(cos(meas[12]),0,0);
    rot.setEntry(sin(meas[12]),0,1);
    rot.setEntry(-sin(meas[12]),1,0);
    rot.setEntry(cos(meas[12]),1,1);*/
    Matrix tmpK0(2,2);
    std::cout << tmpK0.getEntry(0,0) << "\n";
    tmpK0.setEntry(K0.getEntry(0,0)*cos(meas[12]),0,0);
    tmpK0.setEntry(K0.getEntry(0,0)*sin(meas[12]),0,1);
    tmpK0.setEntry(K0.getEntry(1,1)*-1*sin(meas[12]),1,0);
    tmpK0.setEntry(K0.getEntry(1,1)*cos(meas[12]),1,1);


    /*tmpK0.setEntry(K0.getEntry(0,0)*rot.getEntry(0,0)+K0.getEntry(0,1)*rot.getEntry(1,0),0,0);
    tmpK0.setEntry(K0.getEntry(0,0)*rot.getEntry(0,1)+K0.getEntry(0,1)*rot.getEntry(1,1),0,1);
    tmpK0.setEntry(K0.getEntry(1,0)*rot.getEntry(0,0)+K0.getEntry(1,1)*rot.getEntry(1,0),1,0);
    tmpK0.setEntry(K0.getEntry(1,0)*rot.getEntry(1,0)+K0.getEntry(1,1)*rot.getEntry(1,1),1,1);*/

    output = vectorAdd(Kx.multiplyVector(Xstates),Ky.multiplyVector(Ystates));
    output = vectorAdd(tmpK0.multiplyVector(error),output);
    if(output[0] > 0.6) output[0] = 0.6;
    if(output[0] < -0.6) output[0] = -0.6;
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

int main(int argc, char **argv)
{    
    Zcontroller zcontroller(0.05);

    /////////////////  xy CONTROLLER ///////////////
    stateFeedbackController xyController(0.05);

    xyController.K0.setEntry(-1*0.0305,0,0);
    xyController.K0.setEntry(0,0,1);
    xyController.K0.setEntry(0,1,0);
    xyController.K0.setEntry(-1*-0.0218,1,1);

    xyController.Kx.setEntry(-1*0.1328,0,0);
    xyController.Kx.setEntry(-1*0.0656,0,1);
    xyController.Kx.setEntry(-1*-0.0318,0,2);
    xyController.Kx.setEntry(-1*-0.0161,0,3);
    xyController.Kx.setEntry(-1*-0.0075,0,4);

    xyController.Ky.setEntry(-1*-0.1009,0,0);
    xyController.Ky.setEntry(-1*0.0413,0,1);
    xyController.Ky.setEntry(-1*-0.0108,0,2);
    xyController.Ky.setEntry(-1*0.0018,0,3);
    xyController.Ky.setEntry(-1*0.0031,0,4);



    ////////////////////////////////////////////////

    double estimatedStates[16];


    Derivative xdot(2);
    Derivative ydot(2);





    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/mocap/pose", 10, pos_cb);
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>
            ("twist", 10, twist_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher attitude_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);



    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float64>
            ("mavros/setpoint_attitude/att_throttle", 10);

    ros::Publisher rate_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_attitude/cmd_vel", 10);

    ros::Publisher rate1_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_attitude", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    observer_xdot_initialize();
    observer_ydot_initialize();

    double speed = 0;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    int k = 0;
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    double setpoints[2] = {2,2};

    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    pose.pose.orientation.w = 0;
    pose.pose.orientation.z = 1;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.x = 0;
    std_msgs::Float64 thrustInput;
    thrustInput.data = 0.0;
    geometry_msgs::TwistStamped yawRateInput;
    yawRateInput.twist.angular.z = 3.14;

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
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


        q1.setW(position.pose.orientation.w);
        q1.setX(position.pose.orientation.x);
        q1.setY(position.pose.orientation.y);
        q1.setZ(position.pose.orientation.z);

        tf::Matrix3x3 m(q1);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
        double xdotmeas[2];
        xdot.Update(position.pose.position.x);
        ydot.Update(position.pose.position.y);
        //xdotmeas[0] = std::cos(yaw)*xdot.state - std::sin(yaw)*ydot.state;
        xdotmeas[0] = std::cos(yaw)*twist.linear.x - std::sin(yaw)*twist.linear.y;
        xdotmeas[1] = pitch;

        xyController.update(setpoints,estimatedStates);


        speed = sqrt(xdot.state*xdot.state + ydot.state*ydot.state);


        double ydotmeas[2];
        //ydotmeas[0] = std::sin(yaw)*xdot.state + std::cos(yaw)*ydot.state;
        ydotmeas[0] = std::sin(yaw)*twist.linear.x + std::cos(yaw)*twist.linear.y;
        ydotmeas[1] = roll;

        if(k%300 < 100)
        {


        }
        else if(k%300 < 200)
        {

        }
        else
        {

        }
        if(k%10 == 0)
        {
            xdot.Update(position.pose.position.x);
            ydot.Update(position.pose.position.y);
        }


        //q1.setRPY(xyController.output[0],xyController.output[1],yawReference(setpoints[0],setpoints[1]));
        q1.setRPY(0,0,yawReference(setpoints[0],setpoints[1]));
        std::cout << yawReference(setpoints[0],setpoints[1]) << "  yaw:"  << yaw << "\n";
        //q1.setRPY(ydotController.output[0],xdotController.output[0],0);
        //q1.setEuler(0.0,0.1,0.1);

        //q1.setRPY(0.0,0.0,0.0);
        pose.pose.orientation.x = q1.getX();
        pose.pose.orientation.y = q1.getY();
        pose.pose.orientation.z = q1.getZ();
        pose.pose.orientation.w = q1.getW();

        thrustInput.data = zcontroller.update(2,position.pose.position.z);

        local_pos_pub.publish(pose);
        //attitude_pub.publish(pose);
        //thrust_pub.publish(thrustInput);

        last_request1 = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}

