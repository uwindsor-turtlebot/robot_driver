#include <turtlebot_node/TurtlebotSensorState.h>
#include <ros/ros.h>
#include <ctime>

#ifndef _ROBOT_SENSORS_
#define _ROBOT_SENSORS_

namespace ROSRobot
{
	class RobotSensors 
	{
	private:
		bool _wall;
		bool _cliff_left;
		bool _cliff_front_left;
		bool _cliff_front_right;
		bool _cliff_right;
		bool _virtual_wall;
		time_t _last_update;
		ros::Subscriber _sensor_state_subscriber;
	public: /*Constructors*/
		RobotSensors(ros::NodeHandle&);

	public: /*Destructors*/
		~RobotSensors(void);

	public: /*Accessors*/
		bool wall_front() const;
		bool wall_or_cliff_front() const;
		bool cliff() const;
		bool cliff_left() const;
		bool cliff_front() const;
		bool cliff_right() const;
		bool is_updated() const;
	
	private:
		void on_sensor_update(turtlebot_node::TurtlebotSensorState);
	};
}

#endif
