#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <time.h>

#ifndef _ROBOT_ODOMETRY_
#define _ROBOT_ODOMETRY_

namespace ROSRobot
{
	class RobotOdometry
	{
	private:
		ros::Subscriber _odometry_subscriber;
		float _x;
		float _y;
		float _vx;
		float _vy;
		float _orientation;
		time_t _last_update;
	
	public: /*Constructors*/
		RobotOdometry(ros::NodeHandle);	
	public: /*Destructors*/
		~RobotOdometry(void);

	public:
		float get_x() const;
		float get_y() const;
		float get_vx() const;
		float get_vy() const;
		float get_orientation() const;
		bool is_updated() const;
	private:
		void on_odometry_update(nav_msgs::Odometry);
	};
}

#endif
