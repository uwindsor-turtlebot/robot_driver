#include "RobotSensors.hpp"

#ifndef _FRONT_BUMPER_DETECTOR_
#define _FRONT_BUMPER_DETECTOR_

namespace ROSRobot
{
	class FrontBumperDetector
	{
	private:
		RobotSensors& _sensors;
	public: /*Constructor*/
		FrontBumperDetector(RobotSensors&);

	public: /*Accessors*/

	public:
		bool operator()(void) const;
	};
}

#endif
