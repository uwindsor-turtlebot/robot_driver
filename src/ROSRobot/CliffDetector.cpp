#include "CliffDetector.hpp"

namespace ROSRobot
{
	CliffDetector::CliffDetector(RobotSensors& sensors)
		: _sensors(sensors)
	{
	}

	bool CliffDetector::operator()(void) const
	{
		return this->_sensors.cliff();
	}
}
