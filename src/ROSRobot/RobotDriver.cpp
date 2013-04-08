#include "RobotDriver.hpp"
#include "MovementDetector.hpp"
#include "RotationDetector.hpp"
#include "TimeDetector.hpp"
#ifndef PI
	#define PI 3.1415926
#endif

namespace ROSRobot
{
	RobotDriver::RobotDriver(ros::NodeHandle& node_handle) :
		_commander(node_handle),
		_odometry(node_handle),
		_sensors(node_handle),
		_forward_velocity(0.50f),
		_angular_velocity(PI / 4) 
	{
	}

	void RobotDriver::drive_forward(float distance)
	{
		if(this->_odometry.is_updated())
		{	
			MovementDetector movement(this->_odometry, distance);
			this->drive_forward_until<MovementDetector>(movement);
		}
		else
		{
			this->drive_forward_for(distance / this->_forward_velocity);
		}
	}

	void RobotDriver::drive_backward(float distance)
	{
		if(this->_odometry.is_updated())
		{
			MovementDetector movement(this->_odometry, distance);
			this->drive_backward_until<MovementDetector>(movement);
		}
		else
		{
			this->drive_backward_for(distance / (this->_forward_velocity));
		}
	}

	void RobotDriver::turn_left()
	{
		if(this->_odometry.is_updated())
		{
			RotationDetector rotation(this->_odometry, PI / 2.0f);
			this->turn_left_until<RotationDetector>(rotation);
		}
		else
		{
			this->turn_left_for((PI / 2.0f) / this->_angular_velocity);		
		}
	}

	void RobotDriver::turn_right()
	{	
		if(this->_odometry.is_updated())
		{
			RotationDetector rotation(this->_odometry, PI / -2.0f);
			this->turn_right_until<RotationDetector>(rotation);
		}
		else
		{
			this->turn_right_for((PI / 2.0f) / this->_angular_velocity);
		}
	}

	void RobotDriver::turn_around()
	{
		if(this->_odometry.is_updated())
		{
			RotationDetector rotation(this->_odometry, PI);
			this->turn_left_until(rotation);
		}
		else
		{
			this->turn_left_for(PI / this->_angular_velocity);
		}
	}

	void RobotDriver::drive_forward_for(double duration)
	{
		TimeDetector detector(duration);
		this->_commander.drive_until(this->_forward_velocity, detector);
	}

	void RobotDriver::drive_backward_for(double duration)
	{
		TimeDetector detector(duration);
		this->_commander.drive_until(-1.0f * this->_forward_velocity, detector);
	}

	void RobotDriver::turn_left_for(double duration)
	{
		TimeDetector detector(duration);
		this->_commander.turn_until(this->_angular_velocity, detector);
	}

	void RobotDriver::turn_right_for(double duration)
	{
		TimeDetector detector(duration);
		this->_commander.turn_until(-1 * this->_angular_velocity, detector);
	}

	RobotCommander& RobotDriver::get_commander()
	{
		return this->_commander;
	}

	RobotOdometry& RobotDriver::get_odometry()
	{
		return this->_odometry;
	}

	RobotSensors& RobotDriver::get_sensors()
	{
		return this->_sensors;
	}
		
	float RobotDriver::get_x() const
	{
		return this->_odometry.get_x();
	}

	float RobotDriver::get_y() const
	{
		return this->_odometry.get_y();
	}

	float RobotDriver::get_orientation() const
	{
		return this->_odometry.get_orientation();
	}

	float RobotDriver::get_forward_velocity() const
	{
		return this->_forward_velocity;
	}

	float RobotDriver::get_angular_velocity() const
	{
		return this->_angular_velocity;
	}

	void RobotDriver::set_forward_velocity(float value)
	{
		this->_forward_velocity = value;
	}

	void RobotDriver::set_angular_velocity(float value)
	{
		this->_angular_velocity = value;
	}
}
