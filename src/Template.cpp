#include "ROSRobot/RobotDriver.hpp"
#include "Kinect/VisionDetector.hpp"

/*Example Commands:
	robot.drive_forward(1);
	robot.turn_left();
	robot.drive_forward_until(vision_obscureded_detector);*/

int main(int argc, char*[] argv)
{
	/*Initialization*/
	ros::init(argc, argv, "turtlebot_template");
	ros::NodeHandle nodeHandle("turtlebot_template");
	RobotDriver robot(nodeHandle);
	
	Freenect::Freenect freenect;
	MyFreenectDevice* myDevice = &freenect.createDevice<MyFreenectDevice>(0);
	VisionDetector vision_obscured_detector(myDevice);

	/*Your Code Here*/

	/*Shut Down*/
	exit(0);
}
