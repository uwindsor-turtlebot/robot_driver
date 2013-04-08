#include "ROSRobot/ROSRobot.hpp"
#include "Test.h"
#include <iostream>

using ROSRobot::RobotCommander;
using ROSRobot::RobotOdometry;
using ROSRobot::RotationDetector;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Test_RotationDetector");
	ros::NodeHandle nodeHandle(argv[0]);

	RobotCommander driver(nodeHandle);
	RobotOdometry odometry(nodeHandle);

	RotationDetector detector(odometry, 1);
	
	test<RotationDetector>(driver, detector);
	
	return 0;
}
