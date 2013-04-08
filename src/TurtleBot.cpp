// TurtleBot.cpp : Defines the entry point for the console application.
//
#include "Kinect/MyFreenectDevice.hpp"
#include "Kinect/VisionDetector.hpp"
#include "ROSRobot/RobotDriver.hpp"
#include <iostream>

using ROSRobot::RobotDriver;
int main(int argc, char* argv[])
{
	//Creates an ROS object we can use to control motor capabilities	
	ros::init(argc, argv, "turtlebot_driver");
	ros::NodeHandle nodeHandle("turtlebot_driver");
	RobotDriver robit(nodeHandle);


	//Creates a Freenect object we can use to monitor camera
	Freenect::Freenect freenect;
	MyFreenectDevice& myDevice = freenect.createDevice<MyFreenectDevice>(0);
	VisionDetector myCamera(myDevice);
	myDevice.startDepth();

	
	//Start the program
	std::cout << "Program will start in 5 seconds" << std::endl;
	sleep(5);
	std::cout << "Start" << std::endl;

	//----------------------------------------------------------
	//My code here

	for (int i=0; i<3; i++)
	{
		std::cout << "Driving forward.\n";
		robit.drive_forward_until(myCamera);

		std::cout << "Turning left.\n";
		robit.turn_left();
	}

	//----------------------------------------------------------
	
	//End the program
	std::cout << "End" << std::endl;
	myDevice.stopDepth();
	exit (0);
	
}

