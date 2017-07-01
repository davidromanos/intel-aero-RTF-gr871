#Autonomous indoor navigation for drones using vision-based guidance#

ROS nodes for simulating and performing SLAM with the RealSense R200 RGB-D camera on the Intel Aero Ready To Fly drone equipped with the Intel Aero compute board.
Visit http://blog.tkjelectronics.dk for more information

License: GPLv2
Developers: Chris Jeppesen, David Romanos, Joan Calvet Molinas, Malte Rørmose Damgaard, Thomas Kølbæk Jespersen


#ROS bags for testing#

Actual ROS bags of recorded footage from flying which can be used for testing the FastSLAM node can be downloaded from the following Dropbox links:

Manual flying where the drone is moved around manually with the RC transmitter
https://www.dropbox.com/s/4krzzsakguiloqd/ManualFlyingSmooth.bag

Controlled flying where the developed position controllers using Vicon inputs move the drone in a 3D cube
https://www.dropbox.com/s/4iyldxeo6s56wut/VerticalBox_Test2.bag?dl=0


#Vicon_j ROS node#

For controller test and verification the Vicon_j ROS package has been used for sending Vicon position data to the drone
Package credit goes to risengineering.com
Original package from https://bitbucket.org/uav_research/vicon_j

Package receives data from Vicon and then pushes it to the Pixhawk (running the PX4 stack) using MAVROS.

Video on how to use this: https://www.youtube.com/watch?v=hjmDPUNs1DM


#Modified Vicon_j publisher for Windows#

The Windows machine running the Vicon Tracker software needs to run the Vicon_j publisher which connects to the Vicon TCP service and transmits over a UDP port to the Vicon_j ROS node.
The original Vicon_j publisher is found at: https://bitbucket.org/uav_research/vicon_complex

The Vicon_j publisher has been modified to send the position and rotation as a quaternion instead of Euler angles.
The project has also been modified to compile with Visual Studio 2010. At problems please try and compile with Visual Studio 2008.
The modified code and compiled exe file can be downloaded from: https://www.dropbox.com/s/1tzmdmh4blzo22m/uav_research-vicon_complex.zip