#include "RobotOdometry.hpp"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <time.h>

namespace ROSRobot
{
	RobotOdometry::RobotOdometry(ros::NodeHandle nodeHandle) :
		_odometry_subscriber(
			nodeHandle.subscribe<nav_msgs::Odometry>(std::string("/odom"),
			 10, 
			&RobotOdometry::on_odometry_update,
			 this)),
		_x(0), 
		_y(0),
		_vx(0),
		_vy(0),
		_orientation(0),
		_last_update(0)
	{
	}

	RobotOdometry::~RobotOdometry()
	{
	}

	float RobotOdometry::get_x() const
	{
		return this->_x;
	}

	float RobotOdometry::get_y() const
	{
		return this->_y;
	}

	float RobotOdometry::get_vx() const
	{
		return this->_vx;
	}

	float RobotOdometry::get_vy() const
	{
		return this->_vy;
	}

	float RobotOdometry::get_orientation() const
	{
		return this->_orientation;
	}
	
	bool RobotOdometry::is_updated() const
	{
		return difftime(time(NULL), _last_update) < 1.0;
	}

	void RobotOdometry::on_odometry_update(nav_msgs::Odometry odom)
	{
		if(!this->is_updated())
		{
			std::cout << "Odometry updated.\n";
		}
		time(&this->_last_update);
		this->_x = odom.pose.pose.position.x;
		this->_y = odom.pose.pose.position.y;
		this->_vx = odom.twist.twist.linear.x;
		this->_vy = odom.twist.twist.linear.y;
		this->_orientation = odom.pose.pose.orientation.w;
		std::cout << "@(" << this->_x << ", " << this->_y << ")\n";
	}
}
