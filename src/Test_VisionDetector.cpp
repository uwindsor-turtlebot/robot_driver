#include "ROSRobot/ROSRobot.hpp"
#include "Kinect/VisionDetector.hpp"
#include "Test.h"
#include <iostream>

using Freenect::Freenect;
using ROSRobot::RobotCommander;
using ROSRobot::RobotOdometry;
using ROSRobot::RotationDetector;

Freenect::Freenect freenect;
MyFreenectDevice* device = &freenect.createDevice<MyFreenectDevice>(0);
	

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Test_VisionDetector");
	ros::NodeHandle nodeHandle("Test_VisionDetector");

	RobotCommander driver(nodeHandle);
	RobotOdometry odometry(nodeHandle);


	VisionDetector detector(*device);

	device->startDepth();

	//test<VisionDetector>(driver, detector);

	for(int i=0; i<20; i++)
	{
		if (detector())
		{
			std::cout << "TOO CLOSE\n";
		}
		else
		{
			std::cout << "TOO FAR\n";
		}
		sleep(1);
	}

	device->stopDepth();

	
	return 0;
}
