#include "ROSRobot/RobotDriver.hpp"
#include "iostream";

int main(int argc, char*[] argv)
{
	ros::init(argc, argv, "turtlebot_patroller");
	ros::NodeHandle nodeHandle("turtlebot_patroller");
	RobotDriver myBase(nodeHandle);
	
	Freenect::Freenect freenect;
	MyFreenectDevice* myDevice = &freenect.createDevice<MyFreenectDevice>(0);
	VisionDetector myCamera(myDevice);

	for (int i=0; i<3; i++)
	{
		std::cout << "Driving forward.\n";
		myBase.drive_forward_until(myCamera);
		std::cout << "Turning around.\n"
		myBase.turn_around();
	}

	return 0;
}
