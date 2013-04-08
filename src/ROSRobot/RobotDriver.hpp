#include "RobotCommander.hpp"
#include "RobotOdometry.hpp"
#include "RobotSensors.hpp"
#include <ros/ros.h>

#ifndef _ROBOT_DRIVER_
#define _ROBOT_DRIVER_

namespace ROSRobot
{
	class RobotDriver
	{
	private:
		RobotCommander _commander;
		RobotOdometry _odometry;
		RobotSensors _sensors;

		float _forward_velocity;
		float _angular_velocity;

	public:
		RobotDriver(ros::NodeHandle&);
		
	public: /*Odometry-Based Drivers*/
		void drive_forward(float);
		
		void drive_backward(float);

		void turn_left();
		void turn_left(float);

		void turn_right();
		void turn_right(float);

		void turn_around();
	
	public: /*Time-Based Drivers (Fallbacks)*/
		void drive_forward_for(double);
		void drive_backward_for(double);
		void turn_left_for(double);
		void turn_right_for(double);

	public:	/*General Functions*/
		template<class T>
		void drive_forward_until(T&);
		
		template<class T>
		void drive_backward_until(T&);

		template<class T>
		void turn_left_until(T&);

		template<class T>
		void turn_right_until(T&);	
	public:
		RobotCommander& get_commander();
		RobotOdometry& get_odometry();
		RobotSensors& get_sensors();

		float get_x() const;
		float get_y() const;
		float get_orientation() const;

		float get_forward_velocity() const;
		float get_angular_velocity() const;
		
		void set_forward_velocity(float);
		void set_angular_velocity(float);
	};

	template<class T>
	void RobotDriver::drive_forward_until(T& detector)
	{
		this->_commander.drive_until<T>(this->_forward_velocity, detector);
	}

	template<class T>
	void RobotDriver::drive_backward_until(T& detector)
	{
		this->_commander.drive_until<T>(this->_forward_velocity, detector);
	}
	
	template<class T>
	void RobotDriver::turn_left_until(T& detector)
	{
		this->_commander.turn_until<T>(this->_angular_velocity, detector);
	}

	template<class T>
	void RobotDriver::turn_right_until(T& detector)
	{
		this->_commander.turn_until<T>(-1.0 * this->_angular_velocity, detector);
	}
}

#endif
