#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>

#include <chrono>

#include "VelmWheel_core.h"

// Mobile platform constants
const double a = 0.3775; // 1/2 length along Y axis
const double b = 0.36; // 1/2 length along X axis
const double r = 0.1016; // wheel radius

// gear 1/50 | 10000 encoder counts
// (50 * 10000/ 2 * pi) 
const int rad_to_encoder_scalar = 50 * 5000 / 3.1415;

double wrl_speed;
double wrr_speed;
double wfl_speed;
double wfr_speed;
double msg_wfl_enc_pos_;
double msg_wfr_enc_pos_;
double msg_wrr_enc_pos_;
double msg_wrl_enc_pos_;

double msg_wfl_enc_vel_;
double msg_wfr_enc_vel_;
double msg_wrr_enc_vel_;
double msg_wrl_enc_vel_;

double wrl_enc_old;
double wrr_enc_old;
double wfl_enc_old;
double wfr_enc_old;
double wrl_dist_diff, wrr_dist_diff, wfl_dist_diff, wfr_dist_diff;
double wheels_speed_enc[4];
double euler_z;
double euler_old;
double position_x_old;

tf::Quaternion odom_quat;

nav_msgs::Odometry msg_odometry;
geometry_msgs::Twist msg_twist;
std::chrono::nanoseconds nsec;
std::chrono::nanoseconds nsec_rest;
std::chrono::nanoseconds nsec_old;
std::chrono::seconds sec;
uint32_t sequence_enc;

const boost::array<double, 9> orientation_covariance = {0.001, 0, 0,
								   0, 0.001, 0,
								   0, 0, 0.001};
const boost::array<double, 9> angular_velocity_covariance = {0.001, 0, 0,
								   0, 0.001, 0,
								   0, 0, 0.001};
const boost::array<double, 9> linear_acceleration_covariance = {0.001, 0, 0,
								   0, 0.001, 0,
								   0, 0, 0.001};

const boost::array<double, 36> odom_pose_covariace = {0.0001, 0, 0, 0, 0, 0,
													 0, 0.0001, 0, 0, 0, 0,
													 0, 0, 0.0001, 0, 0, 0,
													 0, 0, 0, 0.0001, 0, 0,
													 0, 0, 0, 0, 0.0001, 0,
													 0, 0, 0, 0, 0, 0.0001}; 

const boost::array<double, 36> odom_twist_covariace = {0.0001, 0, 0, 0, 0, 0,
													 0, 0.0001, 0, 0, 0, 0,
													 0, 0, 0.0001, 0, 0, 0,
													 0, 0, 0, 0.0001, 0, 0,
													 0, 0, 0, 0, 0.0001, 0,
													 0, 0, 0, 0, 0, 0.0001};
VelmWheelCore::VelmWheelCore(const std::string& name) : TaskContext(name)
{

	this->addPort("in_twist",in_twist_);
	this->addPort("in_wrr_enc_pos",in_wrr_enc_pos_);
	this->addPort("in_wrl_enc_pos",in_wrl_enc_pos_);
	this->addPort("in_wfr_enc_pos",in_wfr_enc_pos_);
	this->addPort("in_wfl_enc_pos",in_wfl_enc_pos_);
	this->addPort("in_wrr_enc_vel",in_wrr_enc_vel_);
	this->addPort("in_wrl_enc_vel",in_wrl_enc_vel_);
	this->addPort("in_wfr_enc_vel",in_wfr_enc_vel_);
	this->addPort("in_wfl_enc_vel",in_wfl_enc_vel_);

	this->addPort("wrl_port",wrl_port_);
	this->addPort("wrr_port",wrr_port_);
	this->addPort("wfr_port",wfr_port_);
	this->addPort("wfl_port",wfl_port_);

	this->addPort("out_odometry",out_odometry_);
}

VelmWheelCore::~VelmWheelCore() 
{

}

bool VelmWheelCore::configureHook() 
{
	geometry_msgs::Twist msg_twist;
	nav_msgs::Odometry msg_odometry;
	tf::Quaternion odom_quat;
	uint32_t msg_wfl_enc_;
	uint32_t msg_wfr_enc_;
	uint32_t msg_wrr_enc_;
	uint32_t msg_wrl_enc_;

	double wrl_speed;
	double wfl_speed;
	double wfr_speed;
	double wrr_speed;
	const double a = 0.3775;
	const double b = 0.36;
	const double r = 0.1016;
	int rad_to_encoder_scalar = 50 * 5000 / 3.1415; 
	uint32_t wrl_enc_old;
	uint32_t wrr_enc_old;
	uint32_t wfl_enc_old;
	uint32_t wfr_enc_old;
	double wrl_dist_diff, wrr_dist_diff, wfl_dist_diff, wfr_dist_diff;
	boost::array<uint32_t, 4> wheels_speed_enc;
	double euler_z;
	double euler_old;
	double position_x_old;
	std::chrono::nanoseconds nsec;
	std::chrono::nanoseconds nsec_rest;
	std::chrono::nanoseconds nsec_old;
	std::chrono::seconds sec;
	uint32_t sequence_enc;

	const boost::array<double, 9> orientation_covariance = {0, 0, 0,
								   0, 0, 0,
								   0, 0, 0};;
	const boost::array<double, 9> angular_velocity_covariance = {0, 0, 0,
								   0, 0, 0,
								   0, 0, 0};;
	const boost::array<double, 9> linear_acceleration_covariance = {0, 0, 0,
								   0, 0, 0,
								   0, 0, 0};;

	const boost::array<double, 36> odom_pose_covariace = {0.0001, 0, 0, 0, 0, 0,
														 0, 0.0001, 0, 0, 0, 0,
														 0, 0, 0.0001, 0, 0, 0,
														 0, 0, 0, 0.0001, 0, 0,
														 0, 0, 0, 0, 0.0001, 0,
														 0, 0, 0, 0, 0, 0.0001}; 

	const boost::array<double, 36> odom_twist_covariace = {0.0001, 0, 0, 0, 0, 0,
													 0, 0.0001, 0, 0, 0, 0,
													 0, 0, 0.0001, 0, 0, 0,
													 0, 0, 0, 0.0001, 0, 0,
													 0, 0, 0, 0, 0.0001, 0,
													 0, 0, 0, 0, 0, 0.0001};

	return true;
}

bool VelmWheelCore::startHook() 
{
	wrl_speed = 0;
	wfl_speed = 0;
	wfr_speed = 0;
	wrr_speed = 0;

	wrl_enc_old = 0;
	wrr_enc_old = 0;
	wfl_enc_old = 0;
	wfr_enc_old = 0;

	wrl_dist_diff = 0;
	wrr_dist_diff = 0;
	wfl_dist_diff = 0;
	wfr_dist_diff = 0;

	msg_odometry.child_frame_id = "base_link";
	sequence_enc = 0;
	nsec_old = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
	msg_odometry.pose.covariance = odom_pose_covariace;
	msg_odometry.twist.covariance = odom_twist_covariace;
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
void getOdom()
{

/*
    \\\\
    \\   FORMULAS
    \\\\
    theta - angular velocity
	theta = (wheels_speed_enc[0] + wheels_speed_enc[1] + wheels_speed_enc[2] + wheels_speed_enc[3]) * ( r / (4*(a + b)));
    T = (a + b)
    X - velocity along X axis
	(wheels_speed_enc[0] + wheels_speed_enc[1]) * r = 2 * X + 2 * theta * T
	(wheels_speed_enc[2] + wheels_speed_enc[3]) * r = -2 * X + 2* theta * T 
	(wheels_speed_enc[0] + wheels_speed_enc[1]) * r + (wheels_speed_enc[2] + wheels_speed_enc[3]) * r = 4 T
*/
	
	// wheels: 0 - rl, 1 - fl, 2 - fr, 3 - rr
	//

	msg_odometry.twist.twist.angular.z = (wheels_speed_enc[0] + wheels_speed_enc[1] + wheels_speed_enc[2] + wheels_speed_enc[3]) * ( r / (4*(a + b)));
	msg_odometry.twist.twist.linear.x = ((wheels_speed_enc[0] + wheels_speed_enc[1]) * r  - 2 * msg_odometry.twist.twist.angular.z * (a + b)) / 2;
	msg_odometry.twist.twist.linear.y = wheels_speed_enc[1] * r - msg_odometry.twist.twist.linear.x - msg_odometry.twist.twist.angular.z * (a + b);
	euler_z =  euler_z + (wfl_dist_diff + wfr_dist_diff + wrl_dist_diff + wrr_dist_diff) * (r  / (4 * (a + b)));
	odom_quat.setRPY(0, 0, euler_z);

	msg_odometry.pose.pose.orientation.x = odom_quat.x();
	msg_odometry.pose.pose.orientation.y = odom_quat.y();
	msg_odometry.pose.pose.orientation.z = odom_quat.z();
	msg_odometry.pose.pose.orientation.w = odom_quat.w();

	msg_odometry.pose.pose.position.x =  msg_odometry.pose.pose.position.x + ((wrl_dist_diff + wfl_dist_diff) * r - 2 * (euler_z - euler_old) * (a + b)) / 2;
	msg_odometry.pose.pose.position.y =  msg_odometry.pose.pose.position.y + wfl_dist_diff * r - (msg_odometry.pose.pose.position.x - position_x_old) - (euler_z - euler_old) * (a + b);
	msg_odometry.pose.pose.position.z = 0;

	euler_old = euler_z;
	position_x_old = msg_odometry.pose.pose.position.x;

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
		

		wrl_speed = wrl_speed * rad_to_encoder_scalar;
		wfl_speed = wfl_speed * rad_to_encoder_scalar;
		wfr_speed = wfr_speed * rad_to_encoder_scalar;
		wrr_speed = wrr_speed * rad_to_encoder_scalar;

		wrl_port_.write(wrl_speed);
		wfl_port_.write(wfl_speed);
		wfr_port_.write(wfr_speed);
		wrr_port_.write(wrr_speed);
	}

	if (RTT::NewData == in_wfl_enc_pos_.read(msg_wfl_enc_pos_))
	{

		wfl_dist_diff = (msg_wfl_enc_pos_ - wfl_enc_old) / rad_to_encoder_scalar; 
		wfl_enc_old = msg_wfl_enc_pos_ ;

	}
	if (RTT::NewData == in_wrl_enc_pos_.read(msg_wrl_enc_pos_))
	{

		wrl_dist_diff = (msg_wrl_enc_pos_ - wrl_enc_old) / rad_to_encoder_scalar; 
		wrl_enc_old = msg_wrl_enc_pos_ ;

	}
	if (RTT::NewData == in_wrr_enc_pos_.read(msg_wrr_enc_pos_))
	{

		wrr_dist_diff = (msg_wrr_enc_pos_ - wrr_enc_old) / rad_to_encoder_scalar; 
		wrr_enc_old = msg_wrr_enc_pos_ ;
	}
	if (RTT::NewData == in_wfr_enc_pos_.read(msg_wfr_enc_pos_))
	{

		wfr_dist_diff = (msg_wfr_enc_pos_ - wfr_enc_old) / rad_to_encoder_scalar; 
		wfr_enc_old = msg_wfr_enc_pos_ ;

	}	
	// Read encoder data and calculate distance diff [m]
	if (RTT::NewData == in_wrl_enc_vel_.read(msg_wrl_enc_vel_))
	{
		sequence_enc += 1;

		wheels_speed_enc[0] = msg_wrl_enc_vel_/ (rad_to_encoder_scalar);
	}
	if (RTT::NewData == in_wrr_enc_vel_.read(msg_wrr_enc_vel_))
	{
		wheels_speed_enc[3] = msg_wrr_enc_vel_/ (rad_to_encoder_scalar);

	}
	if (RTT::NewData == in_wfl_enc_vel_.read(msg_wfl_enc_vel_))
	{
		wheels_speed_enc[1] = msg_wfl_enc_vel_/ (rad_to_encoder_scalar);

	}
	if (RTT::NewData == in_wfr_enc_vel_.read(msg_wfr_enc_vel_))
	{
		wheels_speed_enc[2] = msg_wfr_enc_vel_/ (rad_to_encoder_scalar);

}
	// set odometry msg header
	nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
	setHeader(msg_odometry.header, "odom", sequence_enc);
	// get base pose and twist from encoders data
	getOdom();
	// write odometry msg to output port
	out_odometry_.write(msg_odometry);

}



ORO_CREATE_COMPONENT(VelmWheelCore)