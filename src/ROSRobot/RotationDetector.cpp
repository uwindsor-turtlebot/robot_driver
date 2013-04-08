#include "RotationDetector.hpp"
#include <cmath>
#define PI 3.1415926

namespace ROSRobot
{
	RotationDetector::RotationDetector(RobotOdometry& odometry, float rotation_goal)
		: _odometry(odometry),
		_rotation_goal(rotation_goal)
	{
		while(2 * PI <= abs(_rotation_goal))
		{
			_rotation_goal -= 2 * PI;
		}

		if(this->_rotation_goal > PI)
		{
			this->_rotation_goal = 2 * PI - _rotation_goal;	
		}
		else if(this->_rotation_goal < -1 * PI)
		{
			this->_rotation_goal = 2 * PI + _rotation_goal;
		}
	}
	
	float RotationDetector::get_starting_orientation() const
	{
		return this->_starting_orientation;
	}

	float RotationDetector::get_orientation() const
	{
		return this->_odometry.get_orientation();
	}

	float RotationDetector::get_rotation_goal() const
	{
		return this->_rotation_goal;
	}

	float RotationDetector::get_rotation() const
	{
		return this->get_orientation() - this->get_starting_orientation();
	}

	bool RotationDetector::operator()(void) const
	{
		return this->get_rotation_goal() < this->get_rotation();
	}
}
