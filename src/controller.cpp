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

void savecopy(double *to,double *from)
{
    //if(*from != *from) // Only returns true if from is NaN
    if(std::isnan(*from)) // Only returns true if from is NaN
    {
        std::cout << "NaN \n";
    }
    else
    {
        *to = *from;
    }
}

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
        void reset();
        Zcontroller(); //constructor
        std::vector<double> thrust;
        Matrix gainZcontroller;

    private:
        std::vector<double> zStates;


};
Zcontroller::Zcontroller() :
    // Member initializer list //    
    gainZcontroller(1,3)

{   // Contructor decleration //            
    gainZcontroller.setEntry(-1*0.2335*1,0,0);
    gainZcontroller.setEntry(-1*0.1838*1,0,1);
    gainZcontroller.setEntry(-1*0.1367*1,0,2);
    //gainZcontroller.setEntry(-1*0.8974,0,0);
    //gainZcontroller.setEntry(-1*1.1745,0,1);
    //gainZcontroller.setEntry(-1*0.8963,0,2);
    thrust.assign(1,0);
    zStates.assign(3,0);
}

double Zcontroller::update(double setpoint,double *estimatedZStates)
{
    zStates[0] = estimatedZStates[13]-setpoint; //note the observer is outputting the estimated output and the estimated states
    zStates[1] = estimatedZStates[14];
    zStates[2] = estimatedZStates[15];

    //Saturate the tracking error to 1 meter.
    if(zStates[0]> 1){zStates[0] = 1;}
    if(zStates[0]< -1){zStates[0] = -1;}

    thrust = gainZcontroller.multiplyVector(zStates);
    thrust[0] = thrust[0] - estimatedZStates[16] + 0.587;
    //thrust[0] = thrust[0] + 0.587;

    /*if(thrust[0]>0.7)
    {
        thrust[0] = 0.7;
    }
    else if(thrust[0]<0.3)
    {
        thrust[0] = 0.3;
    }*/
    if(thrust[0]>1)
    {
        thrust[0] = 1;
    }
    else if(thrust[0]<0)
    {
        thrust[0] = 0.3;
    }
    return thrust[0];
}
void Zcontroller::reset(void)
{
    this->thrust[0] = 0.587;
}
class stateFeedbackController{
    public:
        void update(double setpoint[],double *meas);
        void reset();
        std::vector<double> output;
        Matrix Kx;
        Matrix Ky;
        stateFeedbackController(); //constructor
    private:

        double errorLimit = 5.0;

        std::vector<double> Xstates;
        std::vector<double> Ystates;

};
stateFeedbackController::stateFeedbackController() :
    // Member initializer list //        
    Kx(1,6),
    Ky(1,6)

{   // Contructor decleration //

    output.assign(2,0);

    Xstates.assign(6,0);
    Ystates.assign(6,0);



}

void stateFeedbackController::reset()
{
    output[0] = 0;
    output[1] = 0;
}

void stateFeedbackController::update(double setpoint[],double *meas)
{
    Xstates[0] = meas[0]-(cos(meas[12])*setpoint[0] + sin(meas[12])*setpoint[1]);
    Xstates[1] = meas[2];
    Xstates[2] = meas[3];
    Xstates[3] = meas[4];
    Xstates[4] = meas[5];
    Xstates[5] = meas[6];
    Ystates[0] = meas[1] - (-sin(meas[12])*setpoint[0] + cos(meas[12])*setpoint[1]);
    Ystates[1] = meas[7];
    Ystates[2] = meas[8];
    Ystates[3] = meas[9];
    Ystates[4] = meas[10];
    Ystates[5] = meas[11];

    //Saturate the tracking error to 1 meter.
    if(Xstates[0]> 1){Xstates[0] = 1;}
    if(Xstates[0]< -1){Xstates[0] = -1;}
    if(Ystates[0]> 1){Ystates[0] = 1;}
    if(Ystates[0]< -1){Ystates[0] = -1;}


    std::vector<double>tmp;

    tmp = Kx.multiplyVector(Xstates);
    output[0] = tmp[0] - meas[17];

    tmp = Ky.multiplyVector(Ystates);
    output[1] =tmp[0] - meas[18];

    if(std::isnan(output[0]) == 1){output[0] = 0;}
    if(std::isnan(output[1]) == 1){output[1] = 0;}


    //if(output[0] > 0.6) output[0] = 0.6;
    //if(output[0] < -0.6) output[0] = -0.6;
    //if(output[1] > 0.6) output[1] = 0.6;
    //if(output[1] < -0.6) output[1] = -0.6;
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
    Zcontroller zcontroller;

    /////////////////  xy CONTROLLER ///////////////
    stateFeedbackController xyController;


    /*xyController.K0.setEntry(-1*0.2099,0,0);
    xyController.K0.setEntry(0,0,1);
    xyController.K0.setEntry(0,1,0);
    xyController.K0.setEntry(-1*-0.2088,1,1);*/

    /*xyController.Kx.setEntry(-1*0.2099,0,0);
    xyController.Kx.setEntry(-1*0.3267,0,1);
    xyController.Kx.setEntry(-1*0.1234,0,2);
    xyController.Kx.setEntry(-1*-0.0477,0,3);
    xyController.Kx.setEntry(-1*-0.0423,0,4);
    xyController.Kx.setEntry(-1*-0.0122,0,5);

    xyController.Ky.setEntry(-1*-0.2088,0,0);
    xyController.Ky.setEntry(-1*-0.3231,0,1);
    xyController.Ky.setEntry(-1*0.1209,0,2);
    xyController.Ky.setEntry(-1*-0.0357,0,3);
    xyController.Ky.setEntry(-1*-0.0005,0,4);
    xyController.Ky.setEntry(-1*0.0076,0,5);*/

    xyController.Kx.setEntry(-1*0.0969,0,0);
    xyController.Kx.setEntry(-1*0.1661,0,1);
    xyController.Kx.setEntry(-1*0.0630,0,2);
    xyController.Kx.setEntry(-1*-0.0251,0,3);
    xyController.Kx.setEntry(-1*-0.0217,0,4);
    xyController.Kx.setEntry(-1*-0.0064,0,5);

    xyController.Ky.setEntry(-1*-0.0968,0,0);
    xyController.Ky.setEntry(-1*-0.1635,0,1);
    xyController.Ky.setEntry(-1*0.0653,0,2);
    xyController.Ky.setEntry(-1*-0.0156,0,3);
    xyController.Ky.setEntry(-1*-0.0008,0,4);
    xyController.Ky.setEntry(-1*0.0041,0,5);

    /*xyController.K0integrators.setEntry(-1*-0.0664,0,0);
    xyController.K0integrators.setEntry(0,0,1);
    xyController.K0integrators.setEntry(0,1,0);
    xyController.K0integrators.setEntry(-1*0.0664,1,1);*/



    ////////////////////////////////////////////////

    //zcontroller.gainZcontroller.setEntry(-1*0.8974,0,0);
    //zcontroller.gainZcontroller.setEntry(-1*1.1745,0,1);
    //zcontroller.gainZcontroller.setEntry(-1*0.8963,0,2);

    double estimatedStates[19];

    std::vector<waypoint> listOfWaypoints;
    listOfWaypoints.push_back(waypoint(0,0,1));
    listOfWaypoints.push_back(waypoint(2,0,2));
    listOfWaypoints.push_back(waypoint(0,1,1));
    /*listOfWaypoints.push_back(waypoint(0,0,1));
    listOfWaypoints.push_back(waypoint(0,0,2));
    listOfWaypoints.push_back(waypoint(0,0,1));
    listOfWaypoints.push_back(waypoint(0,0,3));*/
    //listOfWaypoints.push_back(waypoint(0,0,1));





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

    double VarYaw = 10;
    double fastslamMeas[4];
    double covariansVelocities[9] = {1,0,0,
                                    0,1,0,
                                    0,0,1};
    double covariansfastslam[16] = {1,0,0,0,
                                    0,1,0,0,
                                    0,0,1,0,
                                    0,0,0,1};
    

    double PX4Meas[3] = {0,0,0};
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

    int simulation = 0;

    while(ros::ok()){

        if(simulation == 1)
        {
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
        }
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

        savecopy(&PX4Meas[0],&imuRoll);
        savecopy(&PX4Meas[1],&imuPitch);
        savecopy(&PX4Meas[2],&imuYaw);

        savecopy(&fastslamMeas[0],&position.pose.position.x);
        savecopy(&fastslamMeas[1],&position.pose.position.y);
        savecopy(&fastslamMeas[2],&position.pose.position.z);
        savecopy(&fastslamMeas[3],&yaw);





        if(current_state.mode == "OFFBOARD" && current_state.armed)
        {

            ekf(1,fastslamMeas,covariansfastslam,PX4Meas,xyController.output[1]+estimatedStates[18],xyController.output[0]+estimatedStates[17],yawRef,zcontroller.thrust[0],estimatedStates,covariansVelocities,&VarYaw);
            xyController.update(setpoints,estimatedStates);


            if(simulation == 1)
            {
                yawRef = yawReference(currentWaypoint.x-oldWaypoint.x,currentWaypoint.y-oldWaypoint.y);
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


                if(abs(yaw-yawRef)< 0.15)
                {
                    setpoints[0] = currentWaypoint.x;
                    setpoints[1] = currentWaypoint.y;
                    setpoints[2] = currentWaypoint.z;
                }
            }
            else
            {
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
                    std::cout << "current waypoint :" << currentWaypoint.x <<"," << currentWaypoint.y <<"," << currentWaypoint.z  <<  "\n";

                }
                setpoints[0] = currentWaypoint.x;
                setpoints[1] = currentWaypoint.y;
                setpoints[2] = currentWaypoint.z;
            }


            q1.setRPY(xyController.output[1],xyController.output[0],yawRef);

            thrustInput.data = zcontroller.update(setpoints[2],estimatedStates);
        }
        else
        {
            yawRef = yaw;
            ekf(1,fastslamMeas,covariansfastslam,PX4Meas,0+estimatedStates[18],0+estimatedStates[17],yawRef,0.587,estimatedStates,covariansVelocities,&VarYaw);

            //ekf(1,fastslamMeas,covariansfastslam,PX4Meas,0,0,yaw,0.587,estimatedStates,covariansVelocities,&VarYaw);

            zcontroller.reset();
            thrustInput.data = zcontroller.thrust[0];
            //xyController.reset();
            setpoints[0] = position.pose.position.x;
            setpoints[1] = position.pose.position.y;
            setpoints[2] = position.pose.position.z;
            std::cout << "Resetting to:" << setpoints[0] <<"," << setpoints[1] <<"," << setpoints[2]  <<  "\n";

            if(simulation == 0)
            {
                k = 0;
                i = 0;
                listOfWaypoints.clear();
                //listOfWaypoints.push_back(waypoint(setpoints[0],setpoints[1],setpoints[2]));
                //listOfWaypoints.push_back(waypoint(setpoints[0],setpoints[1],setpoints[2]+1));
                //listOfWaypoints.push_back(waypoint(setpoints[0],setpoints[1],setpoints[2]-0.25));

                listOfWaypoints.push_back(waypoint(setpoints[0],setpoints[1],setpoints[2]));
                listOfWaypoints.push_back(waypoint(setpoints[0]-0.75,setpoints[1]-0.75,setpoints[2]));

                listOfWaypoints.push_back(waypoint(setpoints[0]-0.75,setpoints[1]+0.75,setpoints[2]));
                listOfWaypoints.push_back(waypoint(setpoints[0]+0.75,setpoints[1]+0.75,setpoints[2]));
                listOfWaypoints.push_back(waypoint(setpoints[0]+0.75,setpoints[1]-0.75,setpoints[2]));

                //listOfWaypoints.push_back(waypoint(setpoints[0],setpoints[1],setpoints[2]));
            }
            currentWaypoint = listOfWaypoints[0];
            last_request1 = ros::Time::now();

            q1.setRPY(0,0,yawRef);
        }
        //q1.setRPY(0,0,yaw);
        pose.pose.orientation.x = q1.getX();
        pose.pose.orientation.y = q1.getY();
        pose.pose.orientation.z = q1.getZ();
        pose.pose.orientation.w = q1.getW();
        //pose.pose.orientation.x = 0;
        //pose.pose.orientation.y = 0;
        //pose.pose.orientation.z = 1;
        //pose.pose.orientation.w = 0;




        attitude_pub.publish(pose);
        //local_pos_pub.publish(pose);

        thrust_pub.publish(thrustInput);

        logToFile("/home/joan/flightlog.txt","%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",estimatedStates[0],estimatedStates[1],estimatedStates[2],estimatedStates[3],estimatedStates[4],estimatedStates[5],estimatedStates[6],estimatedStates[7],estimatedStates[8],estimatedStates[9],estimatedStates[10],estimatedStates[11],estimatedStates[12],estimatedStates[13],estimatedStates[14],estimatedStates[15],position.pose.position.x,position.pose.position.y,position.pose.position.z,pitch,roll,yaw,xyController.output[0],xyController.output[1],zcontroller.thrust[0],setpoints[0],setpoints[1],setpoints[2],twist.linear.x,twist.linear.y,twist.linear.z,imuPitch,imuRoll,yawRef,estimatedStates[16],estimatedStates[17],estimatedStates[18]);
        last_request1 = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
    ekf_terminate();
}



