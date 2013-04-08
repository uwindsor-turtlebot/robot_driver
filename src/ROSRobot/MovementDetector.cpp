#include "MovementDetector.hpp"
#include <iostream>
#include <cmath>

namespace ROSRobot
{
	MovementDetector::MovementDetector(ROSRobot::RobotOdometry& odometry, float goal_distance) :
		_odometry(odometry),
		_starting_x(odometry.get_x()),
		_starting_y(odometry.get_y()),
		_distance_moved_goal_squared(goal_distance * goal_distance)
	{
	}
	
	float MovementDetector::get_starting_x() const
	{
		return this->_starting_x;
	}

	float MovementDetector::get_starting_y() const
	{
		return this->_starting_y;
	}

	float MovementDetector::get_x() const
	{
		return this->_odometry.get_x();
	}

	float MovementDetector::get_y() const
	{
		return this->_odometry.get_y();
	}

	float MovementDetector::get_dx() const
	{
		return this->get_x() - this->get_starting_x(); 
	}

	float MovementDetector::get_dy() const
	{
		return this->get_y() - this->get_starting_y();
	}
	
	float MovementDetector::get_distance_moved() const
	{
		return sqrt(this->get_distance_moved_squared());
	}

	float MovementDetector::get_distance_moved_squared() const
	{
		return this->get_dx() * this->get_dx() + this->get_dy() * this->get_dy();
	}

	float MovementDetector::get_distance_moved_goal() const
	{
		return sqrt(this->_distance_moved_goal_squared);
	}

	float MovementDetector::get_distance_moved_goal_squared() const
	{
		return this->_distance_moved_goal_squared;
	}

	bool MovementDetector::operator()(void) const
	{
		std::cout << '(' << this->get_starting_x() << ", " << this->get_starting_y() << ") - ";
		std::cout << '(' << this->get_x() << ", " << this->get_y() << ") = ";
		std::cout << '(' << this->get_dx() << ", " << this->get_dy() << ')' << '\n';
		std::cout << '\t' << this->get_distance_moved_squared() << this->get_distance_moved_goal_squared() << '\n';
		return this->get_distance_moved_goal_squared() < this->get_distance_moved_squared();
	}	
}
