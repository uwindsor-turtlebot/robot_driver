
rosbuild_add_library(ROSRobot
	RobotDriver.cpp
	RobotCommander.cpp
	RobotOdometry.cpp 
	RobotSensors.cpp
	MovementDetector.cpp
	RotationDetector.cpp
	CliffDetector.cpp
	FrontBumperDetector.cpp
	TimeDetector.cpp)
