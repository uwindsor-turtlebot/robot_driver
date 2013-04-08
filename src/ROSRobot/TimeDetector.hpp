#include <time.h>

#ifndef _TIME_DETECTOR_
#define _TIME_DETECTOR_

namespace ROSRobot
{
	class TimeDetector
	{
	private:
		time_t _start_time;
		double _time_elapsed_goal;

	public: /*Constructors*/
		TimeDetector(double time_elapsed_goal);

	public: /*Accessors*/
		time_t get_starting_time() const;
		double get_time_elapsed() const;
		double get_time_elapsed_goal() const;	
	public:
		bool operator()(void) const;
	};
}

#endif
