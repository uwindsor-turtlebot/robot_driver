#include "RobotSensors.hpp"
#include <turtlebot_node/TurtlebotSensorState.h>
#include <ros/ros.h>
#include <string.h>

namespace ROSRobot
{
	RobotSensors::RobotSensors(ros::NodeHandle& nodeHandle)
		: _wall(false),
		_cliff_left(false),
		_cliff_front_left(false),
		_cliff_front_right(false),
		_cliff_right(false),
		_virtual_wall(false),
		_last_update(0),
		_sensor_state_subscriber(nodeHandle.subscribe<turtlebot_node::TurtlebotSensorState>(std::string("/turtlebot_node/sensor_state"), 10, &RobotSensors::on_sensor_update, this))
	{
	}

	RobotSensors::~RobotSensors(void)
	{
	}
	
	bool RobotSensors::wall_or_cliff_front() const
	{
		return this->wall_front() ||
			this->cliff_front();
	}

	bool RobotSensors::wall_front() const
	{
		return this->_wall ||
			this->_virtual_wall;
	}

	bool RobotSensors::cliff() const
	{
		return this->_cliff_left ||
			this->_cliff_front_left ||
			this->_cliff_front_right ||
			this->_cliff_right;
	}

	bool RobotSensors::cliff_left() const
	{
		return this->_cliff_left ||
			this->_cliff_front_left;
	}

	bool RobotSensors::cliff_front() const
	{
		return this->_cliff_front_left ||
			this->_cliff_front_right;
	}

	bool RobotSensors::cliff_right() const
	{
		return this->_cliff_front_right ||
			this->_cliff_right;
	}

	bool RobotSensors::is_updated() const
	{
		return difftime(time(NULL), this->_last_update) < 2;
	}
	void RobotSensors::on_sensor_update(turtlebot_node::TurtlebotSensorState sensor_state)
	{
		time(&this->_last_update);
		this->_wall = sensor_state.wall;
		this->_cliff_left = sensor_state.cliff_left;
		this->_cliff_front_left = sensor_state.cliff_front_left;
		this->_cliff_front_right = sensor_state.cliff_front_right;
		this->_cliff_right = sensor_state.cliff_right;
		this->_virtual_wall = sensor_state.virtual_wall;
	}
}

