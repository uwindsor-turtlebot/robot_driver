#include "ROSRobot/RobotCommander.hpp"

template<class Detector>
void test(ROSRobot::RobotCommander& driver, Detector& detector)
{
	std::cout << "Start" << std::endl;
	driver.drive_until<Detector>(1.0, detector);
	driver.turn_for(1.507, 2);
	driver.drive_for(1.0, 1);
	std::cout << "End" << std::endl;
}
