#include "RobotOdometry.hpp"

#ifndef _ROTATION_DETECTOR_
#define _ROTATION_DETECTOR_
namespace ROSRobot
{
	class RotationDetector
	{
	private:
		RobotOdometry& _odometry;
		float _starting_orientation;
		float _rotation_goal;
	public: /*Constructors*/
		RotationDetector(RobotOdometry&, float);

	public:
		float get_starting_orientation() const;
		float get_orientation() const;
		float get_rotation_goal() const;
		float get_rotation() const;

	public:
		bool operator()(void) const;		
	};
}
#endif
