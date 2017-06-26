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


  // Pointers to msgs
  std::auto_ptr<sensor_msgs::LaserScan> msg_laser_front_ptr;
  std::auto_ptr<sensor_msgs::LaserScan> msg_laser_rear_ptr;
  // time variables
  uint32_t loop_time;
  std::chrono::nanoseconds nsec;
  std::chrono::nanoseconds nsec_old;
  std::auto_ptr<ros::Time> current_loop_time_ptr; 
  Eigen::Vector3d euler_laser;

  uint64_t loop_seq;
  std::auto_ptr<std::vector<bool> > new_data_vec_ptr;

  std::auto_ptr<Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>> scan_front_data_matrix;
  std::auto_ptr<Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>> scan_rear_data_matrix;

  std::vector<Eigen::Vector2f> markers;
  std::vector<float> ranges;
  std::vector<float> angles;
  std::vector<int> intensities;
  size_t global_iterator;
  size_t marker_id;
  std::auto_ptr<visualization_msgs::Marker> msg_markers_ptr;
  size_t marker_counter;
  Eigen::Matrix<float,3,3> laser_in_base_transform;
  bool new_rising_edge_front;
  bool new_rising_edge_rear;
  size_t rising_marker_iterator_front;
  size_t rising_marker_iterator_rear;
VelmobilGlobalLocalization::VelmobilGlobalLocalization(const std::string& name) : TaskContext(name)
{

  this->addPort("in_laser_front",in_laser_front_);
  this->addPort("in_laser_rear",in_laser_rear_);
  this->addPort("out_markers",out_markers_);

  this->addProperty("/velmobil_gl/min_intensity",min_intensity_);


  msg_laser_front_ptr.reset(new sensor_msgs::LaserScan());
  msg_laser_rear_ptr.reset(new sensor_msgs::LaserScan());
  msg_markers_ptr.reset(new visualization_msgs::Marker());
  new_data_vec_ptr.reset(new std::vector<bool>(2,false));
  scan_front_data_matrix.reset(new Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>);
  scan_rear_data_matrix.reset(new Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>);
}

VelmobilGlobalLocalization::~VelmobilGlobalLocalization() 
{
}

bool VelmobilGlobalLocalization::removeMarkers(visualization_msgs::Marker &markers)
{
  //delete all objects
  markers.action = 3;
}

bool VelmobilGlobalLocalization::markersInitialization(visualization_msgs::Marker &markers, const size_t &marker_id , const std::vector<Eigen::Vector2f> &positions)
{
markers.header.frame_id = "laser_front";

markers.ns = "localization";
markers.id = 1;
// type = POINTS
markers.type = 8;
// action add object
markers.action = 0;
for (global_iterator = 0; global_iterator < marker_id+1; global_iterator++)
{
  markers.points.at(global_iterator).x = positions.at(global_iterator)(0);
  markers.points.at(global_iterator).y = positions.at(global_iterator)(1);
  markers.points.at(global_iterator).z = 0;
  markers.colors.at(global_iterator).r = 255;
  markers.colors.at(global_iterator).g = 0;
  markers.colors.at(global_iterator).b = 0;
  markers.colors.at(global_iterator).a = 1;


}
for (global_iterator = marker_id+1; global_iterator < markers.colors.size(); global_iterator++)
{
  markers.points.at(global_iterator).x = 100;
  markers.points.at(global_iterator).y = 100;
  markers.points.at(global_iterator).z = 0;
  markers.colors.at(global_iterator).r = 0;
  markers.colors.at(global_iterator).g = 0;
  markers.colors.at(global_iterator).b = 0;
  markers.colors.at(global_iterator).a = 0;
}

}

bool VelmobilGlobalLocalization::polarLaserToCartesianBase(const std::vector<float> &ranges, const std::vector<float> &intensities, Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic> &data, const Eigen::Matrix< float, 3, 3> &transform ) 
{
	for (global_iterator = 0; global_iterator < ranges.size(); global_iterator++ )
	{
		data(global_iterator,0) = transform(0,0) * ranges.at(global_iterator) * cos(angles.at(global_iterator)) + transform(0,1) * ranges.at(global_iterator) * sin(angles.at(global_iterator)) + transform(0,2);
		data(global_iterator,1) = transform(1,0) * ranges.at(global_iterator) * cos(angles.at(global_iterator)) + transform(1,1) * ranges.at(global_iterator) * sin(angles.at(global_iterator)) + transform(1,2);
		data(global_iterator,2) = intensities.at(global_iterator);
	}

	return true;
}
bool VelmobilGlobalLocalization::configureHook() 
{


	return true;
}

bool VelmobilGlobalLocalization::startHook() 
{

  nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
  nsec_old = nsec;
  loop_seq = 0;
  new_data_vec_ptr->at(0) = false;
  new_data_vec_ptr->at(1) = false;
  //[x y intensity] * 540 (beams) * 2 (laser scanners)
  scan_front_data_matrix->resize(540,3);
  scan_rear_data_matrix->resize(540,3);

  angles.resize(540);
  for (int i = 0; i< angles.size(); i++)
  {
  	angles.at(i) = -135 + 0.5 * i;
  }
  global_iterator = 0;
  // set max markers count 
  markers.resize(10);
  marker_id = 0;
  msg_markers_ptr->points.resize(10);
  msg_markers_ptr->colors.resize(10);
  new_rising_edge_front = true;
  new_rising_edge_rear = true;
  rising_marker_iterator_front = 0;
  rising_marker_iterator_rear = 0;
  return true;
}

////
// UPDATE
////
void VelmobilGlobalLocalization::updateHook() 
{

    nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());

    loop_time = nsec.count() - nsec_old.count();
    nsec_old = nsec;
   current_loop_time_ptr->sec = std::chrono::duration_cast<std::chrono::seconds >(nsec).count();
   current_loop_time_ptr->nsec = (nsec - std::chrono::duration_cast<std::chrono::seconds >(nsec)).count();
  markers.clear();
  marker_counter = 0;
  // predict bias and calculate theta

  // 
  // // // ODOM ????
  //

  // predict bias and calculate theta
  if (RTT::NewData == in_laser_front_.read(*msg_laser_front_ptr))
  {
	scan_front_data_matrix->setZero();
  laser_in_base_transform << 1, 0, 0.3,
                             0, 1, 0,
                             0, 0, 1;
	polarLaserToCartesianBase(msg_laser_front_ptr->ranges, msg_laser_front_ptr->intensities, (*scan_front_data_matrix), laser_in_base_transform);
  	new_data_vec_ptr->at(0) = true;
  }
  if (RTT::NewData == in_laser_rear_.read(*msg_laser_rear_ptr))
  {
	scan_rear_data_matrix->setZero();
  laser_in_base_transform << -1, 0, -0.3,
                             0, -1, 0,
                             0, 0, 1;
	polarLaserToCartesianBase(msg_laser_rear_ptr->ranges, msg_laser_rear_ptr->intensities, (*scan_rear_data_matrix), laser_in_base_transform);
	new_data_vec_ptr->at(1) = true;
  }

  if (new_data_vec_ptr->at(0) && new_data_vec_ptr->at(1))
  {
    removeMarkers( (*msg_markers_ptr));
    out_markers_.write((*msg_markers_ptr));

  	for (global_iterator = 0; global_iterator < scan_front_data_matrix->rows(); global_iterator++ )
  	{

  		// [FRONT LASER] found rising edge
  		if ((*scan_front_data_matrix)(global_iterator,2)  > min_intensity_ && new_rising_edge_front)
  		{
  			//calculate distance between last rising edge and cutten 
			rising_marker_iterator_front = global_iterator;
			new_rising_edge_front = false;
  		}
  		// found sloping edge
  		else if ((*scan_front_data_matrix)(global_iterator,2) < min_intensity_ && !new_rising_edge_front)
  		{
  			//marker = {x_in_base, y_in_base}
        marker_id = rising_marker_iterator_front + floor((global_iterator-rising_marker_iterator_front)/2);
  			markers.at(marker_counter) = {(*scan_front_data_matrix)(marker_id,0),(*scan_front_data_matrix)(marker_id,1)};
  			new_rising_edge_front = true;
        marker_counter += 1;
  		}
/*
   		// [REAR LASER] found rising edge
  		if ((*scan_rear_data_matrix)(global_iterator,2)  > min_intensity_ && new_rising_edge_rear)
  		{
  			//calculate distance between last rising edge and cutten 
			rising_marker_iterator_rear = global_iterator;
			new_rising_edge_rear = false;
  		}
  		// found sloping edge
  		else if ((*scan_rear_data_matrix)(global_iterator,2) < min_intensity_ && !new_rising_edge_rear)
  		{
  			//marker = {x_in_base, y_in_base}
        marker_id = rising_marker_iterator_rear + floor((global_iterator-rising_marker_iterator_rear)/2);

  			markers.at(marker_counter) = {(*scan_rear_data_matrix)(marker_id,0),(*scan_rear_data_matrix)(marker_id,1)};
  			new_rising_edge_rear = true;
        marker_counter += 1;
  		}
*/
    markersInitialization( (*msg_markers_ptr), marker_counter, markers);
    out_markers_.write((*msg_markers_ptr));

   	}
 	new_data_vec_ptr->at(0) = false;
 	new_data_vec_ptr->at(1) = false;
  }
  loop_seq += 1;

}

ORO_CREATE_COMPONENT(VelmobilGlobalLocalization)
