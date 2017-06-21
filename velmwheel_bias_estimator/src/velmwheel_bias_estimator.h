#ifndef VELMWHEELBIASESTIMATOR_H_
#define VELMWHEELBIASESTIMATOR_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <string>

class VelmWheelBiasEstimator : public RTT::TaskContext
{
	public:
		explicit VelmWheelBiasEstimator(const std::string& name);
		~VelmWheelBiasEstimator();
		void reset();
	private:
		bool configureHook();
		void updateHook();
		bool startHook();

 		RTT::InputPort<sensor_msgs::Imu> in_imu_;
 		RTT::InputPort<geometry_msgs::PoseWithCovarianceStamped> in_laser_;

 		RTT::OutputPort<sensor_msgs::Imu> out_imu_;
 		RTT::OutputPort<double> out_theta_;
 		double P_init_;
 		double Q_;
 		double alpha_;
};
#endif  // VELMWHEELBIASESTIMATOR_H_
