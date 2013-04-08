#include "TimeDetector.hpp"

namespace ROSRobot
{
	TimeDetector::TimeDetector(double time_elapsed_goal)
		: _start_time(time(NULL)),
		_time_elapsed_goal(time_elapsed_goal)
	{
		
	}

	time_t TimeDetector::get_starting_time() const
	{
		return this->_start_time;
	}

	double TimeDetector::get_time_elapsed() const
	{
		return difftime(time(NULL), this->get_starting_time());
	}

	double TimeDetector::get_time_elapsed_goal() const
	{
		return this->_time_elapsed_goal;
	}

	bool TimeDetector::operator()(void) const
	{
		return this->get_time_elapsed_goal() < this->get_time_elapsed();
	}
	
}
