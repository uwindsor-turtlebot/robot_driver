#include "ROSRobot/ROSRobot.hpp"
#include "Test.h"
#include <iostream>

using ROSRobot::RobotCommander;
using ROSRobot::RobotOdometry;
using ROSRobot::MovementDetector;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Test_MovementDetector");
	ros::NodeHandle nodeHandle(argv[0]);

	RobotCommander driver(nodeHandle);
	RobotOdometry odometry(nodeHandle);

	MovementDetector detector(odometry, 1);
	
	test<MovementDetector>(driver, detector);
	
	return 0;
}
