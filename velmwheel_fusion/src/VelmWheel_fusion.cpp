
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/LinearMath/Quaternion.h>
#include <chrono>
#include "tf/transform_datatypes.h"

#include "robot_localization/ros_filter_types.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "VelmWheel_fusion.h"
#include <iostream>

  uint32_t loop_time;
  std::chrono::nanoseconds nsec;
  std::chrono::nanoseconds nsec_old;
  double pose_x_from_input;
std::auto_ptr<geometry_msgs::Twist> msg_twist;
std::auto_ptr<nav_msgs::Odometry> msg_odometry;
std::auto_ptr<RobotLocalization::Ekf> filter_;
std::auto_ptr<Eigen::Vector3d> latestControl_;
std::auto_ptr<Eigen::VectorXd> last_state_;
std::auto_ptr<tf2::Quaternion> state_quat_;
std::auto_ptr<Eigen::VectorXd> curr_measurement_ptr;
std::auto_ptr<Eigen::VectorXd> curr_meas_cov_diag_ptr;
    std::auto_ptr<ros::Time> current_loop_time_ptr; 
std::auto_ptr<RobotLocalization::Measurement> new_measurement_ptr;

VelmWheelFusion::VelmWheelFusion(const std::string& name) : TaskContext(name)
{

	this->addPort("in_twist",in_twist_);
	this->addPort("in_odometry",in_odometry_);
	this->addPort("out_odometry",out_odometry_);
  //
  // FEATURE -- Dodac wczytywanie wielkosci wektorow z .yaml
  //  
  msg_twist.reset( new geometry_msgs::Twist());  
  msg_odometry.reset(new nav_msgs::Odometry());  
  filter_.reset(new RobotLocalization::Ekf());  
  latestControl_.reset(new Eigen::Vector3d());  
  last_state_.reset(new Eigen::VectorXd());  
  state_quat_.reset(new tf2::Quaternion());  
  state_ptr = &filter_->getState();  
  curr_measurement_ptr.reset(new Eigen::VectorXd(12));  
  curr_meas_cov_diag_ptr.reset(new Eigen::VectorXd(12));
  current_loop_time_ptr.reset(new ros::Time);
  new_measurement_ptr.reset(new RobotLocalization::Measurement);
}

VelmWheelFusion::~VelmWheelFusion() 
{
}

bool VelmWheelFusion::configureHook() 
{
// configure update vector from the measurement
  (*curr_measurement_ptr) << 0,0,0,0,0,0,1,1,0,0,0,1;
filter_->initialize( (*curr_measurement_ptr), 15 );

filter_->setControlParams({0,0,0,0,0,01,1,0,0,0,1,0,0,0,0,0,0}, 1, {1.3, 1.3, 1.3, 1.3, 1.3, 4.5}, {0.8, 1.3, 1.3, 1.3, 1.3, 0.9}, {1.3, 1.3, 1.3, 1.3, 1.3, 4.5}, {1.0, 1.3, 1.3, 1.3, 1.3, 1.0});
Eigen::MatrixXd process_noise_cov(15,15);
 process_noise_cov << 0.05, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                           0,    0.05, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0.06, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0.03, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0.03, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0.06, 0,     0,     0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0.025, 0,     0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,     0.025, 0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,     0,     0.04, 0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01, 0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01, 0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.02, 0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01, 0,    0,
                           0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01, 0,
                           0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.015;
          filter_->setProcessNoiseCovariance(process_noise_cov);      
          filter_->setSensorTimeout(0.1);

    (*latestControl_)(0) =0;
    (*latestControl_)(1) =0;
    (*latestControl_)(2) =0;
	return true;
}

bool VelmWheelFusion::startHook() 
{
 //std::cout << "test 3"<<std::endl;

    msg_twist->linear.x = 0.0;
    // velocity Y input to the robot model
    msg_twist->linear.y = 0.0;
    // angular velocity input to the robot model
    msg_twist->angular.z  = 0.0;
  nsec_old = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
///*       */ std::cout << "test 4"<<std::endl;

	return true;
}

////
// UPDATE
////
void VelmWheelFusion::updateHook() 
{
    nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());

    loop_time = nsec.count() - nsec_old.count();
    nsec_old = nsec;
   current_loop_time_ptr->sec = std::chrono::duration_cast<std::chrono::seconds >(nsec).count();
   current_loop_time_ptr->nsec = (nsec - std::chrono::duration_cast<std::chrono::seconds >(nsec)).count();


	if (RTT::NewData == in_twist_.read(*msg_twist))
	{
		/*
    // velocity X input to the robot model
		msg_twist->linear.x = msg_twist->linear.x * double (loop_time)/1000000000;

		// velocity Y input to the robot model
		msg_twist->linear.y = msg_twist->linear.y * double (loop_time)/1000000000;
		// angular velocity input to the robot model
		msg_twist->angular.z = msg_twist->angular.z * double (loop_time)/1000000000;
    */
//    std::cout << "input_y = "  << msg_twist->linear.y << std::endl;
    *latestControl_ << msg_twist->linear.x, msg_twist->linear.y, msg_twist->angular.z;
	}

	if (RTT::NewData == in_odometry_.read(*msg_odometry))
	{
 
		// [measurement odometry] - angular velocity
    //tf::quaternionMsgToTF(msg_odometry->pose.pose.orientation, tf_quaternion);
 //   std::cout << "measure = "  << msg_odometry->twist.twist.linear.y << std::endl;
  }
  //
  //
  //
  //
  filter_->setControl(*latestControl_, current_loop_time_ptr->toSec()- 0.001);

new_measurement_ptr->topicName_ = "odom";


(*curr_measurement_ptr) <<     0,0,0,0,0,0,msg_odometry->twist.twist.linear.x, msg_odometry->twist.twist.linear.y, 0,
                        0,0,msg_odometry->twist.twist.angular.z;
new_measurement_ptr->measurement_ = (*curr_measurement_ptr);

//new_measurement_ptr->measurement_ = Eigen::VectorXd(0,0,0,0,0,0,msg_odometry->twist.twist.linear.x, msg_odometry->twist.twist.linear.y,0,0,0,msg_odometry->twist.twist.angular.z);
(*curr_meas_cov_diag_ptr) << 1, 1, 1, 1, 1, 1, msg_odometry->twist.covariance[0], msg_odometry->twist.covariance[7], 1,1,1, msg_odometry->twist.covariance[35];
new_measurement_ptr->covariance_ = curr_meas_cov_diag_ptr->asDiagonal();
/*
size_t k = 0;
for (size_t i = 0; i< 6 ; i++){
for (size_t j = 0; j< 6 ; j++){
new_measurement_ptr->covariance_(i,j) = msg_odometry->twist.covariance[k];
k++;
}
}
*/


//std::cout << "covariance : \n"<< new_measurement_ptr->covariance_ <<std::endl;

new_measurement_ptr->updateVector_ = {0,0,0,0,0,0,1,1,0,0,0,1};
new_measurement_ptr->time_ = current_loop_time_ptr->toSec();
new_measurement_ptr->latestControl_ = *latestControl_;
//
//  TRZEBA USTAWIC mahalanobisThresh_ !!!!!!!!!!!!!!!!!!!!!!!
//
new_measurement_ptr->mahalanobisThresh_ = 2;
new_measurement_ptr->latestControlTime_ = current_loop_time_ptr->toSec() - 0.001;

filter_->processMeasurement(*new_measurement_ptr);
//const Eigen::VectorXd &pred_state = filter_->getPredictedState();
//const Eigen::VectorXd &got_control = filter_->getControl();
//std::cout << "got_control: \n"<< got_control <<std::endl;
//std::cout << "pred_state: \n"<< pred_state <<std::endl;

msg_odometry->pose.pose.position.x = (*state_ptr)(0);
msg_odometry->pose.pose.position.y = (*state_ptr)(1);
msg_odometry->pose.pose.position.z = 0;

state_quat_->setRPY((*state_ptr)(3), (*state_ptr)(4), (*state_ptr)(5));

msg_odometry->pose.pose.orientation.x = (*state_quat_).getX();
msg_odometry->pose.pose.orientation.y = (*state_quat_).getY();
msg_odometry->pose.pose.orientation.z = (*state_quat_).getZ();
msg_odometry->pose.pose.orientation.w = (*state_quat_).getW();

msg_odometry->twist.twist.linear.x = (*state_ptr)(6);
msg_odometry->twist.twist.linear.y = (*state_ptr)(7);
msg_odometry->twist.twist.linear.z = 0;

msg_odometry->twist.twist.angular.x = 0;
msg_odometry->twist.twist.angular.y = 0;
msg_odometry->twist.twist.angular.z = (*state_ptr)(11);

msg_odometry->child_frame_id = "base_link";
msg_odometry->header.stamp = (*current_loop_time_ptr);
msg_odometry->header.frame_id = "odom";

out_odometry_.write(*msg_odometry);

}

ORO_CREATE_COMPONENT(VelmWheelFusion)