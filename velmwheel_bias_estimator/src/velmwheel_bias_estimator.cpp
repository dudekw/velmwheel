#include "velmwheel_bias_estimator.h"

// Orocos
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS msgs
#include <sensor_msgs/Imu.h>

// Debug
#include <iostream>
#include <fstream>

  // Pointers to msgs
  std::auto_ptr<geometry_msgs::PoseWithCovarianceStamped> msg_laser_ptr;
  std::auto_ptr<sensor_msgs::Imu> msg_imu_ptr;
  // time variables
  uint32_t loop_time;
  std::chrono::nanoseconds nsec;
  std::chrono::nanoseconds nsec_old;
  std::auto_ptr<ros::Time> current_loop_time_ptr; 
  Eigen::Vector3d euler_laser;
  Eigen::Matrix<double, 2, 2> P;
  Eigen::Matrix<double, 2, 2> A;
  Eigen::Matrix<double, 2, 2> Q_eigen;
  double gyro_orientation_z;
  double new_bias;
  uint64_t loop_seq;
  std::chrono::microseconds usec_imu_delta(0);
  std::chrono::microseconds old_usec_imu(0);
  std::chrono::milliseconds msec_laser_delta(0);
  std::chrono::milliseconds old_msec_laser(0);
  double R;
  double S;
  Eigen::Vector2d K;
  double eBias;
  std::auto_ptr<Eigen::Quaternion<double>> eigen_quaternion_ptr;

  std::ofstream myfile;
  bool is_initialized;
VelmWheelBiasEstimator::VelmWheelBiasEstimator(const std::string& name) : TaskContext(name)
{

  this->addPort("in_laser",in_laser_);
  this->addPort("in_imu",in_imu_);
  this->addPort("out_imu",out_imu_);
  this->addPort("out_theta",out_theta_);
  this->addPort("localization_initialized",localization_initialized_);
  this->addProperty("/VELMWHEEL_OROCOS_ROBOT/VelmWheel_bias_estimator/Q",Q_);
  this->addProperty("/VELMWHEEL_OROCOS_ROBOT/VelmWheel_bias_estimator/P_init",P_init_);
  // additive bias error [deg/s] 


  msg_laser_ptr.reset(new geometry_msgs::PoseWithCovarianceStamped());
  msg_imu_ptr.reset(new sensor_msgs::Imu());
  eigen_quaternion_ptr.reset(new Eigen::Quaternion<double>(1,0,0,0));
  current_loop_time_ptr.reset(new ros::Time());

}

VelmWheelBiasEstimator::~VelmWheelBiasEstimator() 
{
    myfile.close();

}

bool VelmWheelBiasEstimator::configureHook() 
{
  std::cout << "Q: \n" << Q_.at(0) <<std::endl;

  Q_eigen << Q_.at(0) , 0       ,
                 0    , Q_.at(1);
  P << P_init_.at(0), P_init_.at(1),
       P_init_.at(2), P_init_.at(3); 
 
  std::cout << "P_init: \n" << P_init_.at(0)<<std::endl;

  std::cout << "Q_eigen: \n" << Q_eigen <<std::endl;
  std::cout << "P: \n" << P <<std::endl;
	return true;
}

bool VelmWheelBiasEstimator::startHook() 
{
  new_bias = 0;
  gyro_orientation_z = 0;
  
  loop_seq = 0;
  R = 1;
  S = 0;
  K << 0, 0;
  eBias = 0;
  is_initialized = false;
  nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch());

  nsec_old = nsec;
  loop_time = nsec.count() - nsec_old.count();
  current_loop_time_ptr->sec = std::chrono::duration_cast<std::chrono::seconds>(nsec).count();
  current_loop_time_ptr->nsec = (nsec - std::chrono::duration_cast<std::chrono::seconds>(nsec)).count();
  myfile.open ("/tmp/bias_log_data.txt");

  return true;
}

////
// UPDATE
////
void VelmWheelBiasEstimator::updateHook() 
{
    loop_seq += 1;

    nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());

    loop_time = nsec.count() - nsec_old.count();
    nsec_old = nsec;
   current_loop_time_ptr->sec = std::chrono::duration_cast<std::chrono::seconds >(nsec).count();
   current_loop_time_ptr->nsec = (nsec - std::chrono::duration_cast<std::chrono::seconds >(nsec)).count();


if (RTT::NewData == localization_initialized_.read(is_initialized))
{
}
if (is_initialized)
{
  // predict bias and calculate theta
  if (RTT::NewData == in_imu_.read(*msg_imu_ptr))
  {
    myfile << "--- NEW IMU ---"<< std::endl;
    if ( (double)old_usec_imu.count() == 0 ){
      old_usec_imu = std::chrono::duration_cast<std::chrono::microseconds>(nsec);
    }
    // get time delta in microseconds
    usec_imu_delta = std::chrono::duration_cast<std::chrono::microseconds>(nsec - old_usec_imu); 
    myfile << "usec_imu_delta: "<<usec_imu_delta.count() <<std::endl;
    myfile << "old_usec_imu: "<<old_usec_imu.count() <<std::endl;

    // predict state -> predict the bias [rad/sec] = old_bias + alpha [deg/s] * to_radians * second_fraction_passed [usec/usec]
    //new_bias = new_bias + alpha_ * (M_PI/180) * ((double)usec_imu_delta.count())/1000000;
    myfile << "new_bias: "<<new_bias <<std::endl;

    // calculate theta with gyro w/o bias
    myfile << "imu_v - bias: "<<(msg_imu_ptr->angular_velocity.z - new_bias) <<std::endl;
    myfile << "delta T: "<<((double)(usec_imu_delta.count()))/1000000  <<std::endl;
    myfile << "delta T sec: "<<std::chrono::duration_cast<std::chrono::seconds>(usec_imu_delta).count() <<std::endl;

    A << 1 , -((double)usec_imu_delta.count())/1000000,
         0 , 1;


    gyro_orientation_z = gyro_orientation_z + ((double)usec_imu_delta.count())/1000000 * (msg_imu_ptr->angular_velocity.z - new_bias);
    myfile << "gyro_orientation_z: "<<gyro_orientation_z <<std::endl;

    // save current time 
    old_usec_imu = std::chrono::duration_cast<std::chrono::microseconds>(nsec);
    // prediction varaince 
    // Q[0] = rms 
    // Q[1] = bias_stability

    myfile << "Q: \n"<<Q_eigen <<std::endl;

    P = A * P * A.inverse() + Q_eigen;

    myfile << "P: \n"<<P <<std::endl;
    myfile << "imu_vel: "<<(msg_imu_ptr->angular_velocity.z) <<std::endl;

    // substact bias from input gyro msg
    msg_imu_ptr->angular_velocity.z = (msg_imu_ptr->angular_velocity.z) - new_bias; 
    // write Imu w/o gyro bias
    myfile << "WRITING: "<<msg_imu_ptr->angular_velocity.z <<std::endl;

    out_imu_.write((*msg_imu_ptr));
  }
  // correct bias
  if (RTT::NewData == in_laser_.read(*msg_laser_ptr))
  {
    myfile << "---- NEW LASER ---- " <<std::endl;
    //  msec_laser_delta = std::chrono::duration_cast<std::chrono::milliseconds>(nsec);
    // get laser theta angle from quaternion
    eigen_quaternion_ptr->w() = msg_laser_ptr->pose.pose.orientation.w;
    eigen_quaternion_ptr->x() = msg_laser_ptr->pose.pose.orientation.x;
    eigen_quaternion_ptr->y() = msg_laser_ptr->pose.pose.orientation.y;
    eigen_quaternion_ptr->z() = msg_laser_ptr->pose.pose.orientation.z;
    euler_laser = eigen_quaternion_ptr->toRotationMatrix().eulerAngles(0, 1, 2);
    myfile << "euler_laser: "<<euler_laser <<std::endl;    
    
    if ( (double) old_msec_laser.count() == 0 )
    {
      gyro_orientation_z = euler_laser[2];
    }
    myfile << "gyro_orientation_z: "<<gyro_orientation_z <<std::endl;
    // get variance from msg covariance matrix [35] -> rZ,rZ
    R = msg_laser_ptr->pose.covariance[35];
    myfile << "R: "<<R <<std::endl;

    // get time delta in milliseconds
    msec_laser_delta = std::chrono::duration_cast<std::chrono::milliseconds>(nsec - old_msec_laser); 

    // correct -> calculate: kalman gain, residuum (eBias) and new_bias
    myfile << "P: "<<P <<std::endl;

    S = P(0,0) + R;
    myfile << "S = P + R: "<<S <<std::endl;
    //
    // P = |P[0]  0 |
    //     |0   P[1]|
    
    //
    // K = |k[0] 0| 
    //     |K[1] 0|

    K(0) = P(0,0) * 1/S ;
    K(1) = P(1,0) * 1/S ;
    myfile << "K = P * 1/S : "<<K <<std::endl;

    P = P - K * S * K.transpose();

    myfile << "P - K * S: "<<P <<std::endl;
    myfile << "delta theta: "<<(euler_laser[2] - gyro_orientation_z) <<std::endl;

    // eBias [rad/sec]
    eBias = (euler_laser[2] - gyro_orientation_z);
    myfile << "eBias: "<<eBias <<std::endl;

    //yTheta = msg_laser_ptr->orientation.z - gyro_orientation_z;
    gyro_orientation_z = gyro_orientation_z + K(0) * eBias;
    new_bias = new_bias + K(1) * eBias/ ((double)(msec_laser_delta.count())/1000);
    myfile << "new_bias: "<<new_bias <<std::endl;

    old_msec_laser = std::chrono::duration_cast<std::chrono::milliseconds>(nsec);
  }
  // wite theta
  out_theta_.write(gyro_orientation_z);
}
}

ORO_CREATE_COMPONENT(VelmWheelBiasEstimator)
