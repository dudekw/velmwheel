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
#include <iostream>
#include <fstream>
#include <cmath>
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

    std::ofstream myfile;

VelmobilGlobalLocalization::VelmobilGlobalLocalization(const std::string& name) : TaskContext(name)
{

  this->addPort("in_laser_front",in_laser_front_);
  this->addPort("in_laser_rear",in_laser_rear_);
  this->addPort("out_markers",out_markers_);
  this->addProperty("/VELMWHEEL_OROCOS_ROBOT/velmobil_global_localization/min_intensity",min_intensity_);


current_loop_time_ptr.reset(new ros::Time());
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

bool VelmobilGlobalLocalization::removeMarkers(visualization_msgs::Marker &vis_markers, const size_t &marker_id)
{

for (global_iterator = 0; global_iterator < marker_id; global_iterator++)
{
  markers.at(global_iterator) << 100,100;

  vis_markers.points.at(global_iterator).x = 0;
  vis_markers.points.at(global_iterator).y = 0;
  vis_markers.points.at(global_iterator).z = 0;
  vis_markers.colors.at(global_iterator).r = 0;
  vis_markers.colors.at(global_iterator).g = 1;
  vis_markers.colors.at(global_iterator).b = 0;
  vis_markers.colors.at(global_iterator).a = 0;


}
}

bool VelmobilGlobalLocalization::markersInitialization(visualization_msgs::Marker &vis_markers, const size_t &marker_id , const std::vector<Eigen::Vector2f> &positions)
{
vis_markers.header.frame_id = "base_link";

vis_markers.ns = "localization";
vis_markers.id = 1;
// type = POINTS
vis_markers.type = 8;
// action add object
vis_markers.action = 0;
vis_markers.lifetime = ros::Duration(0.5);
vis_markers.scale.x = 0.1;
vis_markers.scale.y = 0.1;
vis_markers.frame_locked = true;
for (global_iterator = 0; global_iterator < marker_id; global_iterator++)
{
  vis_markers.points.at(global_iterator).x = positions.at(global_iterator)(0);
  vis_markers.points.at(global_iterator).y = positions.at(global_iterator)(1);
  vis_markers.points.at(global_iterator).z = 0;
  vis_markers.colors.at(global_iterator).r = 0;
  vis_markers.colors.at(global_iterator).g = 1;
  vis_markers.colors.at(global_iterator).b = 0;
  vis_markers.colors.at(global_iterator).a = 1;


}
for (global_iterator = marker_id+1; global_iterator < vis_markers.colors.size(); global_iterator++)
{
  vis_markers.points.at(global_iterator).x = 100;
  vis_markers.points.at(global_iterator).y = 100;
  vis_markers.points.at(global_iterator).z = 0;
  vis_markers.colors.at(global_iterator).r = 0;
  vis_markers.colors.at(global_iterator).g = 0;
  vis_markers.colors.at(global_iterator).b = 0;
  vis_markers.colors.at(global_iterator).a = 0;
}

}

bool VelmobilGlobalLocalization::polarLaserToCartesianBase(const std::vector<float> &ranges, const std::vector<float> &intensities, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> &data, const Eigen::Matrix< float, 3, 3> &transform ) 
{
  myfile <<"polar to cartesian init\n"; 
  myfile <<"ranges: "<<ranges.size()<<"\n"; 
  myfile <<"angles: "<<angles.size()<<"\n"; 
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

std::cout<< "min_intensity_: " << min_intensity_<< "\n";


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
  scan_front_data_matrix->resize(541,3);
  scan_rear_data_matrix->resize(541,3);

  angles.resize(541);
  for (int i = 0; i< angles.size(); i++)
  {
  	angles.at(i) = M_PI/180 * (-135 + 0.5 * i);
  }
  global_iterator = 0;
  // set max markers count 
  markers.resize(20);
  marker_id = 0;
  msg_markers_ptr->points.resize(20);
  msg_markers_ptr->colors.resize(20);
  new_rising_edge_front = true;
  new_rising_edge_rear = true;
  rising_marker_iterator_front = 0;
  rising_marker_iterator_rear = 0;
std::cout<< "marker size: " << markers.size()<< "\n";
  myfile.open ("/tmp/gl_log_data.txt");

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

  removeMarkers( (*msg_markers_ptr), marker_counter);
  out_markers_.write((*msg_markers_ptr));


  marker_counter = 0;
  // predict bias and calculate theta

  // 
  // // // ODOM ????
  //

  // predict bias and calculate theta
  if (RTT::NewData == in_laser_front_.read((*msg_laser_front_ptr)))
  {

	scan_front_data_matrix->setZero();
  laser_in_base_transform << 1, 0, 0.3,
                             0, 1, 0,
                             0, 0, 1;
	polarLaserToCartesianBase(msg_laser_front_ptr->ranges, msg_laser_front_ptr->intensities, (*scan_front_data_matrix), laser_in_base_transform);
  	new_data_vec_ptr->at(0) = true;
  }

  if (RTT::NewData == in_laser_rear_.read((*msg_laser_rear_ptr)))
  {

  scan_front_data_matrix->setZero();
  laser_in_base_transform << -1, 0, -0.3,
                             0, -1, 0,
                             0, 0, 1;
  polarLaserToCartesianBase(msg_laser_rear_ptr->ranges, msg_laser_rear_ptr->intensities, (*scan_rear_data_matrix), laser_in_base_transform);
    new_data_vec_ptr->at(1) = true;
  }

  if (new_data_vec_ptr->at(0) && new_data_vec_ptr->at(1))
  {
myfile << "remove markers \n";

    // removeMarkers( (*msg_markers_ptr));
    // out_markers_.write((*msg_markers_ptr));

  	for (global_iterator = 0; global_iterator < scan_front_data_matrix->rows(); global_iterator++ )
  	{

  		// [FRONT LASER] found rising edge
  		if ((*scan_front_data_matrix)(global_iterator,2)  > min_intensity_ && new_rising_edge_front)
  		{
        myfile << "RISING\n";

  			//calculate distance between last rising edge and cutten 
			rising_marker_iterator_front = global_iterator;
			new_rising_edge_front = false;
  		}
  		// found sloping edge
  		else if ((*scan_front_data_matrix)(global_iterator,2) < min_intensity_ && !new_rising_edge_front)
  		{
        myfile << "SLOPING\n";

  			//marker = {x_in_base, y_in_base}
        marker_id = rising_marker_iterator_front + floor((global_iterator-rising_marker_iterator_front)/2);
  			markers.at(marker_counter) << (*scan_front_data_matrix)(marker_id,0),(*scan_front_data_matrix)(marker_id,1);
  			new_rising_edge_front = true;
        myfile << "marker_id: " << marker_id<<"\n";
        myfile << "marker_intensity: " << (*scan_front_data_matrix)(marker_id,2)<<"\n";
        myfile << "marker_counter: " << marker_counter<<"\n";
        myfile << "markers: \n"<< markers.at(marker_counter)<<"\n";
        myfile << "scan data: \n"<< (*scan_front_data_matrix)(marker_id,0) << "\n"<<(*scan_front_data_matrix)(marker_id,1)<<"\n";
        marker_counter += 1;

  		}

      // [REAR LASER] found rising edge
      if ((*scan_rear_data_matrix)(global_iterator,2)  > min_intensity_ && new_rising_edge_rear)
      {
        myfile << "REAR - RISING\n";

        //calculate distance between last rising edge and cutten 
      rising_marker_iterator_rear = global_iterator;
      new_rising_edge_rear = false;
      }
      // found sloping edge
      else if ((*scan_rear_data_matrix)(global_iterator,2) < min_intensity_ && !new_rising_edge_rear)
      {
        myfile << "REAR -SLOPING\n";

        //marker = {x_in_base, y_in_base}
        marker_id = rising_marker_iterator_rear + floor((global_iterator-rising_marker_iterator_rear)/2);
        markers.at(marker_counter) << (*scan_rear_data_matrix)(marker_id,0),(*scan_rear_data_matrix)(marker_id,1);
        new_rising_edge_rear = true;
        myfile << "REAR -marker_id: " << marker_id<<"\n";
        myfile << "REAR -marker_intensity: " << (*scan_rear_data_matrix)(marker_id,2)<<"\n";
        myfile << "REAR -marker_counter: " << marker_counter<<"\n";
        myfile << "REAR -markers: \n"<< markers.at(marker_counter)<<"\n";
        myfile << "REAR -scan data: \n"<< (*scan_rear_data_matrix)(marker_id,0) << "\n"<<(*scan_rear_data_matrix)(marker_id,1)<<"\n";
        marker_counter += 1;

      }


   	}

    myfile << "-- initialization ---\n";
      markersInitialization( (*msg_markers_ptr), marker_counter, markers);
    out_markers_.write((*msg_markers_ptr));
 	new_data_vec_ptr->at(0) = false;
 	new_data_vec_ptr->at(1) = false;
  }

  loop_seq += 1;

}

ORO_CREATE_COMPONENT(VelmobilGlobalLocalization)
