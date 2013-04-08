#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#ifndef _ROBOT_COMMANDER_
#define _ROBOT_COMMANDER_

namespace ROSRobot
{
	class RobotCommander
	{
	private:
		ros::Publisher _command_publisher;

	public: /*Constructors*/
		RobotCommander(ros::NodeHandle&);

	public: /*Actuators*/
		/*Driving*/
		/*drive_for(driving speed (m/s), turning speed (rad/s)?, duration (s))*/
		void drive_for(float, float, int);
		void drive_for(float, int);

		/*drive_while(driving speed (m/s), turning speed (rad/s)?, duration (s))*/
		void drive_while(float, float, bool(*)(void));
		void drive_while(float, bool (*)(void));

		template<class T>
		void drive_while(float, float, T&);
		template<class T>
		void drive_while(float, T&);

		/*drive_until(driving speed (m/s), turning speed (rad/s)?, <predicate>)*/
		void drive_until(float, float, bool (*)(void));
		void drive_until(float, bool (*)(void));

		template<class T>
		void drive_until(float, float, T&);
		template<class T>
		void drive_until(float, T&);

		/*Turning*/
		/*turn_for(turning speed (rad/s), duration (s))*/
		void turn_for(float, int);

		/*turn_while(turning speed (rad/s), <predicate>)*/
		void turn_while(float, bool (*)(void));
		
		template<class T>
		void turn_while(float, T&);

		/*turn_until(turning speed (rad/s), <predicate>)*/
		void turn_until(float, bool (*)(void));
		
		template<class T>
		void turn_until(float, T&);
		
		/*Atomic Interaction*/
		void publish_command(float, float);
	};

	template<class Detector>
	void RobotCommander::drive_while(float forward_velocity, float angular_velocity, Detector& delegate)
	{
		while(delegate())
		{
			this->publish_command(forward_velocity, angular_velocity);
		}
	}


	template<class Detector>
	void RobotCommander::drive_while(float forward_velocity, Detector& delegate)
	{
		this->drive_while(forward_velocity, 0, delegate);
	}

	template<class Detector>
	void RobotCommander::drive_until(float forward_velocity, float angular_velocity, Detector& delegate)
	{
		while(!delegate())
		{
			this->publish_command(forward_velocity, angular_velocity);
		}
	}

	template<class Detector>
	void RobotCommander::drive_until(float forward_velocity, Detector& delegate)
	{
		this->drive_until(forward_velocity, 0.0f, delegate);
	}

	template<class Detector>
	void RobotCommander::turn_while(float angular_velocity, Detector& delegate)
	{
		this->drive_while(0.0f, angular_velocity, delegate);
	}

	template<class Detector>
	void RobotCommander::turn_until(float angular_velocity, Detector& delegate)
	{
		this->drive_until(0.0f, angular_velocity, delegate);
	}
}

#endif
