#include "velmobil_gl.h"

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

  // Pointers to msgs
  std::auto_ptr<geometry_msgs::PoseWithCovarianceStamped> msg_computed_pose_ptr;
  std::auto_ptr<sensor_msgs::Odometry> msg_odom;

  std::auto_ptr<sensor_msgs::LaserScan> msg_laser_front_ptr;
  std::auto_ptr<sensor_msgs::LaserScan> msg_laser_rear_ptr;
  // time variables
  uint32_t loop_time;
  std::chrono::nanoseconds nsec;
  std::chrono::nanoseconds nsec_old;
  std::auto_ptr<ros::Time> current_loop_time_ptr; 
  Eigen::Vector3d euler_laser;

  uint64_t loop_seq;
  std::vector<bool> new_data_vec(2, false);
  std::auto_ptr<Eigen::Matrix<float, Dynamic , Dynamic>> scan_front_data_matrix;
  std::auto_ptr<Eigen::Matrix<float, Dynamic , Dynamic>> scan_rear_data_matrix;

  std::vector<Eigen::Vector2f> markers;
  size_t marker_counter;

  std::vector<float> ranges;
  std::vector<float> angles;
  std::vector<int> intensities;
  size_t global_iterator;

class polar
{
	public:
		float r,th;
		polar(){}
		polar(int a,int b)
		{
			r=a;
			th=b;
		}
		void show()
		{
			cout<<"In polar form:\nr="<<r<<" and theta="<<th;
			
		}
};
class rectangular
{
	float x,y;
	public:
		rectangular(){}
		rectangular(polar p)
		{
			x=p.r*cos(p.th);
			y=p.r*sin(p.th);
		}
		void show()
		{
			cout<<"\nIn Rectangular form:\nx="<<x<<"and y="<<y;
			
		}
};

VelmWheelBiasEstimator::VelmWheelBiasEstimator(const std::string& name) : TaskContext(name)
{

  this->addPort("in_laser_front",in_laser_front_);
  this->addPort("in_laser_rear",in_laser_rear_);
  this->addPort("in_odom",in_odom_);
  this->addPort("out_pose",out_pose_);

  this->addProperty("/velmobil_gl/min_intensity",min_intensity_);


  msg_computed_pose_ptr.reset(new geometry_msgs::PoseWithCovarianceStamped());
  msg_laser_front_ptr.reset(new sensor_msgs::LaserScan());
  msg_laser_rear_ptr.reset(new sensor_msgs::LaserScan());


}

VelmWheelBiasEstimator::~VelmWheelBiasEstimator() 
{
}
bool VelmWheelBiasEstimator::polarLaserToCartesianBase(const std::vector<float> &ranges, const std::vector<float> &intensities, Eigen::MatrixXXf &data, const Eigen::Matrix< float, 3, 3> laser_in_base_transform ) 
{
	for (global_iterator = 0; global_iterator < ranges.size(); global_iterator++ )
	{
		data(global_iterator,0) = laser_in_base_transform(0,0) * ranges.at(global_iterator) * cos(angles.at(global_iterator)) + laser_in_base_transform(0,1) * ranges.at(global_iterator) * sin(angles.at(global_iterator)) + laser_in_base_transform(0,2);
		data(global_iterator,1) = laser_in_base_transform(1,0) * ranges.at(global_iterator) * cos(angles.at(global_iterator)) + laser_in_base_transform(1,1) * ranges.at(global_iterator) * sin(angles.at(global_iterator)) + laser_in_base_transform(1,2);
		data(global_iterator,2) = intensities.at(global_iterator);
	}

	return true;
}
bool VelmWheelBiasEstimator::configureHook() 
{


	return true;
}

bool VelmWheelBiasEstimator::startHook() 
{

  nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
  nsec_old = nsec;
  loop_seq = 0;
  new_data_vec.at(0) = false;
  new_data_vec.at(1) = false;
  //[x y intensity] * 540 (beams) * 2 (laser scanners)
  scan_front_data_matrix.resize(540,3);
  scan_rear_data_matrix.resize(540,3);

  angles.resize(540);
  for (int i = 0; i< angles.size(); i++)
  {
  	angles.at(i) = -135 + 0.5 * i;
  }
  global_iterator = 0;
  // set max markers count 
  markers.resize(10);

  return true;
}

////
// UPDATE
////
void VelmWheelBiasEstimator::updateHook() 
{

    nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());

    loop_time = nsec.count() - nsec_old.count();
    nsec_old = nsec;
   current_loop_time_ptr->sec = std::chrono::duration_cast<std::chrono::seconds >(nsec).count();
   current_loop_time_ptr->nsec = (nsec - std::chrono::duration_cast<std::chrono::seconds >(nsec)).count();
  markers.clear();
  // predict bias and calculate theta

  // 
  // // // ODOM ????
  //
  if (RTT::NewData == in_laser_front_.read(*msg_laser_front_ptr))
  {
	scan_front_data_matrix.setZero();
	polarToCartesian(msg_laser_front_ptr->ranges, msg_laser_front_ptr->intensities, scan_front_data_matrix);
  	new_data_vec.at(0) = true;
  }

  // predict bias and calculate theta
  if (RTT::NewData == in_laser_front_.read(*msg_laser_front_ptr))
  {
	scan_front_data_matrix.setZero();
	polarToCartesian(msg_laser_front_ptr->ranges, msg_laser_front_ptr->intensities, scan_front_data_matrix);
  	new_data_vec.at(0) = true;
  }
  if (RTT::NewData == in_laser_rear_.read(*msg_laser_rear_ptr))
  {
	scan_rear_data_matrix.setZero();
	polarToCartesian(msg_laser_rear_ptr->ranges, msg_laser_rear_ptr->intensities, scan_rear_data_matrix);
	new_data_vec.at(1) = true;
  }

  if (new_data_vec.at(0) && new_data_vec.at(1))
  {
  	for (global_iterator = 0; global_iterator < scan_front_data_matrix.rows(); global_iterator++ )
  	{

  		// [FRONT LASER] found rising edge
  		if (scan_front_data_matrix(global_iterator,2)  > intensity_tresh && new_rising_edge_front)
  		{
  			//calculate distance between last rising edge and cutten 
			rising_marker_iterator_front = global_iterator;
			new_rising_edge_front = false;
  		}
  		// found sloping edge
  		else if (scan_front_data_matrix(global_iterator,2) < intensity_tresh && !new_rising_edge_front)
  		{
  			//marker = {x_in_base, y_in_base}
  			markers.at(marker_counter) = {scan_front_data_matrix(global_iterator-rising_marker_iterator_front,0),scan_front_data_matrix(global_iterator-rising_marker_iterator_front,1)};
  			new_rising_edge_front = true;
  		}

   		// [REAR LASER] found rising edge
  		if (scan_rear_data_matrix(global_iterator,2)  > intensity_tresh && new_rising_edge_rear)
  		{
  			//calculate distance between last rising edge and cutten 
			rising_marker_iterator_rear = global_iterator;
			new_rising_edge_rear = false;
  		}
  		// found sloping edge
  		else if (scan_rear_data_matrix(global_iterator,2) < intensity_tresh && !new_rising_edge_rear)
  		{
  			//marker = {x_in_base, y_in_base}
  			markers.at(marker_counter) = {scan_rear_data_matrix(global_iterator-rising_marker_iterator_rear,0),scan_rear_data_matrix(global_iterator-rising_marker_iterator_rear,1)};
  			new_rising_edge_rear = true;
  		}


   	}
 	new_data_vec.at(0) = false;
 	new_data_vec.at(1) = false;
  }
  // wite theta
  out_theta_.write(gyro_orientation_z);
  loop_seq += 1;

}

ORO_CREATE_COMPONENT(VelmWheelBiasEstimator)
