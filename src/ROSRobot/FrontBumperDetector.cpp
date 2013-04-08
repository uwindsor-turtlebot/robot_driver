#include "FrontBumperDetector.hpp"

namespace ROSRobot
{
	FrontBumperDetector::FrontBumperDetector(RobotSensors& sensors)
		: _sensors(sensors)
	{
	}

	bool FrontBumperDetector::operator()(void) const
	{
		return this->_sensors.wall_front();
	}
}
