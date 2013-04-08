#include "RobotOdometry.hpp"

#ifndef _MOVEMENT_DETECTOR_
#define _MOVEMENT_DETECTOR_

namespace ROSRobot
{
	class MovementDetector
	{
	private:
		ROSRobot::RobotOdometry& _odometry;
		float _starting_x;
		float _starting_y;
		float _distance_moved_goal_squared;

	public: /*Constructors*/
		MovementDetector(ROSRobot::RobotOdometry&, float);
	
	public: /*Accessors*/
		float get_starting_x() const;
		float get_starting_y() const;
		float get_x() const;
		float get_y() const;
		float get_dx() const;
		float get_dy() const;
		float get_distance_moved() const;
		float get_distance_moved_squared() const;
		float get_distance_moved_goal() const;
		float get_distance_moved_goal_squared() const;
		
	public:
		bool operator ()(void) const;	
	};
}

#endif
