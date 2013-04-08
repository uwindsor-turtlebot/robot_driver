#include "ROSRobot/ROSRobot.hpp"
#include "Test.h"
#include <iostream>

using ROSRobot::RobotCommander;
using ROSRobot::TimeDetector;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Test_TimeDetector");
	ros::NodeHandle nodeHandle(argv[0]);

	RobotCommander driver(nodeHandle);

	TimeDetector detector(2);

	test<TimeDetector>(driver, detector);

	return 0;
}

