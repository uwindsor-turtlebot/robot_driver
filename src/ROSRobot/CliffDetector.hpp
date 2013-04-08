#include "RobotSensors.hpp"

#ifndef _CLIFF_DETECTOR_
#define _CLIFF_DETECTOR_
namespace ROSRobot
{
	class CliffDetector
	{
	private:
		RobotSensors& _sensors;
	public: /*Constructors*/
		CliffDetector(RobotSensors&);

	public:
		bool operator()(void) const;
	};
}
#endif
