#include "ROSRobot/ROSRobot.hpp"
#include "Test.h"
#include <iostream>

using ROSRobot::RobotCommander;
using ROSRobot::RobotSensors;
using ROSRobot::FrontBumperDetector;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Test_FrontBumperDetector");
	ros::NodeHandle nodeHandle(argv[0]);

	RobotCommander driver(nodeHandle);
	RobotSensors sensors(nodeHandle);

	FrontBumperDetector detector(sensors);
	
	test<FrontBumperDetector>(driver, detector);
	
	return 0;
}

