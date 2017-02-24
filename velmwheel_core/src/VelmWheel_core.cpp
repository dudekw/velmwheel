#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include <string>

#include "VelmWheel_core.h"

const double a = 0.3775;
const double b = 0.36;
const double r = 0.2032;
double wrl_speed;
double wrr_speed;
double wfl_speed;
double wfr_speed;

VelmWheelCore::VelmWheelCore(const std::string& name) : TaskContext(name)
{

	this->addPort("input_twist",input_twist_);
	this->addPort("wrl_port",wrl_port_);
	this->addPort("wrr_port",wrr_port_);
	this->addPort("wfr_port",wfr_port_);
	this->addPort("wfl_port",wfl_port_);
}

VelmWheelCore::~VelmWheelCore() 
{

}

bool VelmWheelCore::configureHook() 
{
	geometry_msgs::Twist msg_twist_;
	double wrl_speed;
	double wfl_speed;
	double wfr_speed;
	double wrr_speed;
	const double a = 0.3775;
	const double b = 0.36;
	const double r = 0.2032;
	return true;
}

bool VelmWheelCore::startHook() 
{
	wrl_speed = 0;
	wfl_speed = 0;
	wfr_speed = 0;
	wrr_speed = 0;

	return true;
}

void VelmWheelCore::updateHook() 
{

	if (RTT::NewData == input_twist_.read(msg_twist_))
	{
		// rear left
		wrl_speed = (-msg_twist_.linear.y + msg_twist_.linear.x + msg_twist_.angular.z * (a + b)) / r;
		// front left
		wfl_speed = (msg_twist_.linear.y + msg_twist_.linear.x + msg_twist_.angular.z * (a + b)) / r;
		// front right
		wfr_speed = (msg_twist_.linear.y - msg_twist_.linear.x + msg_twist_.angular.z * (a + b)) / r;
		// rear right
		wrr_speed = (-msg_twist_.linear.y - msg_twist_.linear.x + msg_twist_.angular.z * (a + b)) / r;
		
		wrl_speed = wrl_speed * 50 * 5000 / 3.1415;
		wfl_speed = wfl_speed * 50 * 5000 / 3.1415;
		wfr_speed = wfr_speed * 50 * 5000 / 3.1415;
		wrr_speed = wrr_speed * 50 * 5000 / 3.1415;

		wrl_port_.write(wrl_speed);
		wfl_port_.write(wfl_speed);
		wfr_port_.write(wfr_speed);
		wrr_port_.write(wrr_speed);
	}

}
ORO_CREATE_COMPONENT(VelmWheelCore)