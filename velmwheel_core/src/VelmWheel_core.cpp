#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>

#include <chrono>

#include "VelmWheel_core.h"

const double a = 0.3775;
const double b = 0.36;
const double r = 0.2032;
double wrl_speed;
double wrr_speed;
double wfl_speed;
double wfr_speed;
double msg_wfl_enc_;
double msg_wfr_enc_;
double msg_wrr_enc_;
double msg_wrl_enc_;

double wrl_enc_old;
double wrr_enc_old;
double wfl_enc_old;
double wfr_enc_old;
double wrl_dist_diff, wrr_dist_diff, wfl_dist_diff, wfr_dist_diff;
double wheels_speed_enc[4];
double euler_z;
tf::Quaternion odom_quat;

nav_msgs::Odometry msg_odometry;
geometry_msgs::Twist msg_twist;
std::chrono::nanoseconds nsec;
std::chrono::nanoseconds nsec_rest;
std::chrono::nanoseconds nsec_old;
std::chrono::seconds sec;
uint32_t sequence_enc;

sensor_msgs::Imu msg_ros_imu;
tf::Quaternion imu_quat;
sensor_msgs::Imu msg_imu;
boost::array<double, 9> orientation_covariance = {0, 0, 0,
								   0, 0, 0,
								   0, 0, 0};
boost::array<double, 9> angular_velocity_covariance = {0, 0, 0,
								   0, 0, 0,
								   0, 0, 0};
boost::array<double, 9> linear_acceleration_covariance = {0, 0, 0,
								   0, 0, 0,
								   0, 0, 0};
VelmWheelCore::VelmWheelCore(const std::string& name) : TaskContext(name)
{

	this->addPort("in_twist",in_twist_);
	this->addPort("in_wrr_enc",in_wrr_enc_);
	this->addPort("in_wrl_enc",in_wrl_enc_);
	this->addPort("in_wfr_enc",in_wfr_enc_);
	this->addPort("in_wfl_enc",in_wfl_enc_);
	this->addPort("in_imu",in_imu_);


	this->addPort("wrl_port",wrl_port_);
	this->addPort("wrr_port",wrr_port_);
	this->addPort("wfr_port",wfr_port_);
	this->addPort("wfl_port",wfl_port_);

	this->addPort("out_odometry",out_odometry_);
	this->addPort("out_imu",out_imu_);

}

VelmWheelCore::~VelmWheelCore() 
{

}

bool VelmWheelCore::configureHook() 
{
	geometry_msgs::Twist msg_twist;
	nav_msgs::Odometry msg_odometry;
	tf::Quaternion odom_quat;
	double msg_wfl_enc_;
	double msg_wfr_enc_;
	double msg_wrr_enc_;
	double msg_wrl_enc_;

	double wrl_speed;
	double wfl_speed;
	double wfr_speed;
	double wrr_speed;
	const double a = 0.3775;
	const double b = 0.36;
	const double r = 0.2032;

	double wrl_enc_old;
	double wrr_enc_old;
	double wfl_enc_old;
	double wfr_enc_old;
	double wrl_dist_diff, wrr_dist_diff, wfl_dist_diff, wfr_dist_diff;
	boost::array<double, 4> wheels_speed_enc;
	double euler_z;

	std::chrono::nanoseconds nsec;
	std::chrono::nanoseconds nsec_rest;
	std::chrono::nanoseconds nsec_old;
	std::chrono::seconds sec;
	uint32_t sequence_enc;

	sensor_msgs::Imu msg_imu;
	sensor_msgs::Imu msg_ros_imu;
	tf::Quaternion imu_quat;
	boost::array<double, 9> orientation_covariance;
	boost::array<double, 9> angular_velocity_covariance;
	boost::array<double, 9> linear_acceleration_covariance;
	return true;
}

bool VelmWheelCore::startHook() 
{
	wrl_speed = 0;
	wfl_speed = 0;
	wfr_speed = 0;
	wrr_speed = 0;

	double wrl_enc_old = 0;
	double wrr_enc_old = 0;
	double wfl_enc_old = 0;
	double wfr_enc_old = 0;

	msg_odometry.child_frame_id = "base_link";
	sequence_enc = 0;
	nsec_old = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());


	return true;
}


void setHeader(std_msgs::Header &header, const std::string &frame, const uint32_t &seq_id)
{
	header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(nsec).count();
	nsec_rest = nsec - std::chrono::duration_cast<std::chrono::seconds>(nsec);
	header.stamp.nsec = nsec_rest.count();;
	header.frame_id = frame;
	header.seq = seq_id;
}

// wheels: 0 - rl, 1 - fl, 2 - fr, 3 - rr
void getWheelsSpeed()
{
	wheels_speed_enc[0] = (double)(wrl_dist_diff / (double)(nsec.count() - nsec_old.count()) ) *1000000000;
	wheels_speed_enc[1] = (double)(wfl_dist_diff / (double)(nsec.count() - nsec_old.count())) *1000000000;
	wheels_speed_enc[2] = (double)(wfr_dist_diff / (double)(nsec.count() - nsec_old.count())) *1000000000;
	wheels_speed_enc[3] = (double)(wrr_dist_diff / (double)(nsec.count() - nsec_old.count())) *1000000000;
	RTT::Logger::log() << RTT::Logger::Warning << "enc_0: "<<wheels_speed_enc[0]<<RTT::Logger::nl;
}

// wheels: 0 - rl, 1 - fl, 2 - fr, 3 - rr
void getOdom()
{
/*
	(wheels_speed_enc[0] + wheels_speed_enc[1]) * r = 2 * X + 2 * T
	(wheels_speed_enc[2] + wheels_speed_enc[3]) * r = -2 * X + 2* T 
	(wheels_speed_enc[0] + wheels_speed_enc[1]) * r + (wheels_speed_enc[2] + wheels_speed_enc[3]) * r = 4 T
*/

	// 0 r       r        0 r^2
	// ---- *  ----    = ------------
	//  4       a + b       4 (a + b)

	// wheels: 0 - rl, 1 - fl, 2 - fr, 3 - rr
	msg_odometry.twist.twist.angular.z = (wheels_speed_enc[0] + wheels_speed_enc[1] + wheels_speed_enc[2] + wheels_speed_enc[3]) * ( r*r / (4*(a + b)));
	msg_odometry.twist.twist.linear.x = (((wheels_speed_enc[0] + wheels_speed_enc[1]) * r ) / 2) - 2 * msg_odometry.twist.twist.angular.z;
	msg_odometry.twist.twist.linear.y = wheels_speed_enc[1] * r - msg_odometry.twist.twist.linear.x - msg_odometry.twist.twist.angular.z;

	euler_z =  (wrl_dist_diff + wfl_dist_diff + wfr_dist_diff + wrr_dist_diff) * ( r*r / (4*(a + b)));
	odom_quat.setRPY(0, 0, euler_z);

	msg_odometry.pose.pose.orientation.x = odom_quat.x();
	msg_odometry.pose.pose.orientation.y = odom_quat.y();
	msg_odometry.pose.pose.orientation.z = odom_quat.z();
	msg_odometry.pose.pose.orientation.w = odom_quat.w();

	msg_odometry.pose.pose.position.x =  (((wrl_dist_diff + wfl_dist_diff) * r ) / 2) - 2 * euler_z;
	msg_odometry.pose.pose.position.y =  wfl_dist_diff * r - msg_odometry.pose.pose.position.x - euler_z;
	msg_odometry.pose.pose.position.z = 0;
}


////
// UPDATE
////
void VelmWheelCore::updateHook() 
{

	if (RTT::NewData == in_twist_.read(msg_twist))
	{
		// rear left
		wrl_speed = (-msg_twist.linear.y + msg_twist.linear.x + msg_twist.angular.z * (a + b)) / r;
		// front left
		wfl_speed = (msg_twist.linear.y + msg_twist.linear.x + msg_twist.angular.z * (a + b)) / r;
		// front right
		wfr_speed = (msg_twist.linear.y - msg_twist.linear.x + msg_twist.angular.z * (a + b)) / r;
		// rear right
		wrr_speed = (-msg_twist.linear.y - msg_twist.linear.x + msg_twist.angular.z * (a + b)) / r;
		
		wrl_speed = wrl_speed * 50 * 5000 / 3.1415;
		wfl_speed = wfl_speed * 50 * 5000 / 3.1415;
		wfr_speed = wfr_speed * 50 * 5000 / 3.1415;
		wrr_speed = wrr_speed * 50 * 5000 / 3.1415;

		wrl_port_.write(wrl_speed);
		wfl_port_.write(wfl_speed);
		wfr_port_.write(wfr_speed);
		wrr_port_.write(wrr_speed);
	}

	// Read encoder data and calculate distance diff [m]  ||   Do all sensors return data at the same time???????   ||
	if (RTT::NewData == in_wrl_enc_.read(msg_wrl_enc_))
	{
		RTT::Logger::log() << RTT::Logger::Warning << "msg_wrl_enc_: "<<msg_wrl_enc_<<RTT::Logger::nl;

		nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
		sequence_enc += 1;
		wrl_dist_diff = (double) (msg_wrl_enc_ - wrl_enc_old) / (double)((50*5000 * (3.1415/180) (2 * 3.1415 * r)) / 360 ); 
		RTT::Logger::log() << RTT::Logger::Warning << "wrl_dist_diff: "<<wrl_dist_diff<<RTT::Logger::nl;
		wrl_enc_old = msg_wrl_enc_;
	}
	if (RTT::NewData == in_wrr_enc_.read(msg_wrr_enc_))
	{
		wrr_dist_diff = ( (msg_wrr_enc_ - wrr_enc_old) / (50*5000) ) * ( (2 * 3.1415 * r) / 360 );
		wrr_enc_old = msg_wrr_enc_;
	}
	if (RTT::NewData == in_wfl_enc_.read(msg_wfl_enc_))
	{
		wfl_dist_diff = ( (msg_wfl_enc_ - wfl_enc_old) / (50*5000) ) * ( (2 * 3.1415 * r) / 360 );
		wfl_enc_old = msg_wfl_enc_;
	}
	if (RTT::NewData == in_wfr_enc_.read(msg_wfr_enc_))
	{
		wfr_dist_diff = ( (msg_wfr_enc_ - wfr_enc_old) / (50*5000) ) * ( (2 * 3.1415 * r) / 360 );
		wfr_enc_old = msg_wfr_enc_;
	}
	// Read IMU data 
	if (RTT::NewData == in_imu_.read(msg_imu))
	{
		imu_quat.setRPY(0, 0, 0);
		msg_ros_imu.orientation.x = imu_quat.x();
		msg_ros_imu.orientation.y = imu_quat.y();
		msg_ros_imu.orientation.z = imu_quat.z();
		msg_ros_imu.orientation.w = imu_quat.w();

		msg_ros_imu.orientation_covariance = orientation_covariance;

		msg_ros_imu.angular_velocity.x = 0;
		msg_ros_imu.angular_velocity.y = 0;
		msg_ros_imu.angular_velocity.z = 0;
		msg_ros_imu.angular_velocity_covariance = angular_velocity_covariance;

		msg_ros_imu.linear_acceleration.x = 0;
		msg_ros_imu.linear_acceleration.y = 0;
		msg_ros_imu.linear_acceleration.z = 0;
		msg_ros_imu.linear_acceleration_covariance = linear_acceleration_covariance;
	}
	// set odometry msg header
	setHeader(msg_odometry.header, "odom", sequence_enc);
	// get speed of wheels [m/s]
	getWheelsSpeed();
	// get base pose and twist from odometry
	getOdom();
	// write odometry msg to output port
	out_odometry_.write(msg_odometry);
	// write imu msg to output port
	out_imu_.write(msg_ros_imu);


	nsec_old = nsec;

}


ORO_CREATE_COMPONENT(VelmWheelCore)