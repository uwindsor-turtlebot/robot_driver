#include "RobotCommander.hpp"
#include <string>
#include <math.h>
#include <time.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace ROSRobot
{
	RobotCommander::RobotCommander(ros::NodeHandle& nodeHandle) :
		_command_publisher(nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1))
	{
	}

	void RobotCommander::drive_for(float forwardVelocity, float angularVelocity, int duration)
	/*Causes the robot to drive at the desired velocities until it has moved for the specified time.
	Input:
		Forward Velocity (meters/second),
		Angular Velocity (radians/second),
		Time (seconds)
	Output:
		None*/
	{
		time_t startTime;
		startTime = time(NULL);

		while(time(NULL) - startTime < duration)
		{
			publish_command(forwardVelocity, angularVelocity);
		}
		this->publish_command(0.0f, 0.0f);
	}

	void RobotCommander::drive_for(float forwardVelocity, int duration)
	{
		this->drive_for(forwardVelocity, 0, duration);
	}

	void RobotCommander::drive_while(float forwardVelocity, float angularVelocity, bool (*predicate)(void))
	/*Causes the robot to drive at the desired velocities until the inputted predicate returns false.
	Input:
		Forward Velocity (meters/second),
		Angular Velocity (radians/second),
		Predicate bool (*)(void)
	Output:
		None.
	*/
	{
		while(predicate())
		{
			this->publish_command(forwardVelocity, angularVelocity);
		}
		this->publish_command(0.0f, 0.0f);
	}

	void RobotCommander::drive_while(float forwardVelocity, bool (*predicate)(void))
	{
		this->drive_until(forwardVelocity, 0, predicate);
	}

	void RobotCommander::drive_until(float forwardVelocity, float angularVelocity, bool (*predicate)(void))
	/*Causes the robot to drive at the desired velocities until the inputted predicate returns false.
	Input:
		Forward Velocity (meters/second),
		Angular Velocity (radians/second),
		Predicate bool (*)(void)
	Output:
		None.
	*/
	{
		while(!predicate())
		{
			this->publish_command(forwardVelocity, angularVelocity);
		}
		this->publish_command(0.0f, 0.0f);
	}

	void RobotCommander::drive_until(float forwardVelocity, bool (*predicate)(void))
	{
		this->drive_until(forwardVelocity, 0, predicate);
	}

	void RobotCommander::turn_for(float angularVelocity, int duration)
	{
		this->drive_for(0, angularVelocity, duration);
	}

	void RobotCommander::turn_until(float angularVelocity, bool (*predicate)())
	{
		this->drive_until(0, angularVelocity, predicate);
	}

	void RobotCommander::publish_command(float forwardVelocity, float angularVelocity)
	{
		geometry_msgs::Twist command;
		command.linear.x = forwardVelocity;
		command.angular.z = angularVelocity;
		this->_command_publisher.publish(command);
	}
}
