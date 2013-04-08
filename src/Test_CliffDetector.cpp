#include "ROSRobot/ROSRobot.hpp"
#include "Test.h"
#include <iostream>

using ROSRobot::RobotCommander;
using ROSRobot::RobotSensors;
using ROSRobot::CliffDetector;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Test_CliffDetector");
	ros::NodeHandle nodeHandle(argv[0]);

	RobotCommander driver(nodeHandle);
	RobotSensors sensors(nodeHandle);
	
	CliffDetector detector(sensors);
	test<CliffDetector>(driver, detector);

	return 0;
}

