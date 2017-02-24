#ifndef VELMWHEELCORE_H_
#define VELMWHEELCORE_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>

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

			geometry_msgs::Twist msg_twist_;

 		RTT::InputPort<geometry_msgs::Twist> input_twist_;

		RTT::OutputPort<double> wrl_port_;
		RTT::OutputPort<double> wrr_port_;
		RTT::OutputPort<double> wfl_port_;
		RTT::OutputPort<double> wfr_port_;

};
#endif  // VELMWHEELCORE_H_
