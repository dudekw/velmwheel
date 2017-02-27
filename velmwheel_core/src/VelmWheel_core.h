#ifndef VELMWHEELCORE_H_
#define VELMWHEELCORE_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <string>

class VelmWheelCore : public RTT::TaskContext
{
	public:
		explicit VelmWheelCore(const std::string& name);
		~VelmWheelCore();
		void reset();
	private:
		bool configureHook();
		void updateHook();
		bool startHook();

 		RTT::InputPort<geometry_msgs::Twist> in_twist_;
 		RTT::InputPort<double> in_wrr_enc_;
 		RTT::InputPort<double> in_wrl_enc_;
 		RTT::InputPort<double> in_wfr_enc_;
 		RTT::InputPort<double> in_wfl_enc_;
 		RTT::InputPort<sensor_msgs::Imu> in_imu_;

 		RTT::OutputPort<nav_msgs::Odometry> out_odometry_;
 		RTT::OutputPort<sensor_msgs::Imu> out_imu_;
		RTT::OutputPort<double> wrl_port_;
		RTT::OutputPort<double> wrr_port_;
		RTT::OutputPort<double> wfl_port_;
		RTT::OutputPort<double> wfr_port_;

};
#endif  // VELMWHEELCORE_H_
