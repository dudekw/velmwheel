#ifndef VELMWHEELFUSION_H_
#define VELMWHEELFUSION_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <string>

class VelmWheelFusion : public RTT::TaskContext
{
	public:
		explicit VelmWheelFusion(const std::string& name);
		~VelmWheelFusion();
		void reset();
	private:
		bool configureHook();
		void updateHook();
		bool startHook();

 		RTT::InputPort<geometry_msgs::Twist> in_twist_;
 		RTT::InputPort<sensor_msgs::Imu> in_imu_;
 		RTT::InputPort<nav_msgs::Odometry> in_odometry_;

 		RTT::OutputPort<nav_msgs::Odometry> out_odometry_;
 		Eigen::VectorXd state_ ;
const Eigen::VectorXd* state_ptr;

};
#endif  // VELMWHEELFUSION_H_
