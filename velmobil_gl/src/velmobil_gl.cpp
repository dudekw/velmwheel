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

//XML
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

//openCV
#include "opencv2/opencv.hpp" 
  // Pointers to msgs
  std::auto_ptr<sensor_msgs::LaserScan> msg_laser_front_ptr;
  std::auto_ptr<sensor_msgs::LaserScan> msg_laser_rear_ptr;
  std::auto_ptr<tf2_msgs::TFMessage> msg_odom_transform_ptr;
  std::auto_ptr<visualization_msgs::Marker> msg_markers_ptr;
  std::auto_ptr<std::string> msg_save_map_ptr;
  std::auto_ptr<int> msg_change_mode_ptr;
  // time variables
  uint32_t loop_time;
  std::chrono::nanoseconds nsec;
  std::chrono::nanoseconds nsec_old;
  std::auto_ptr<ros::Time> current_loop_time_ptr; 
  Eigen::Vector3d euler_laser;
  Eigen::Quaternion<float> odom_quaternion;
  Eigen::Vector3f odom_translation;
  Eigen::Matrix<float,3,3> odom_transform;
  Eigen::Matrix<float,3,3> laser_in_odom_transform;

  uint64_t loop_seq;
  std::auto_ptr<std::vector<bool> > new_data_vec_ptr;

  std::auto_ptr<Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>> scan_front_data_matrix;
  std::auto_ptr<Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>> scan_rear_data_matrix;

  std::vector<Eigen::Vector2f> markers;
  std::vector<Eigen::Vector2f> map_markers;
  std::vector<float> ranges;
  std::vector<float> angles;
  std::vector<int> intensities;
  size_t global_iterator;
  size_t global_iterator_2;

  size_t marker_id;
  size_t marker_counter;
  size_t map_marker_counter;
  Eigen::Matrix<float,3,3> laser_in_base_transform;
  bool new_rising_edge_front;
  bool new_rising_edge_rear;
  size_t rising_marker_iterator_front;
  size_t rising_marker_iterator_rear;

    std::ofstream myfile;
// mode
  int my_mode;
// XML
  boost::property_tree::ptree xml_tree;
  boost::property_tree::ptree& marker_tree = xml_tree;
  std::ostringstream marker_path;

// opencv
  cv::Mat transform_cv;
  std::vector<cv::Point2f> markers_cv;
  std::vector<cv::Point2f> map_markers_cv;
VelmobilGlobalLocalization::VelmobilGlobalLocalization(const std::string& name) : TaskContext(name)
{

  this->addPort("in_odom_transform",in_odom_transform_);
  this->addPort("in_laser_front",in_laser_front_);
  this->addPort("in_laser_rear",in_laser_rear_);
  this->addPort("in_change_mode",in_change_mode_);
  this->addPort("in_save_map",in_save_map_);
  this->addPort("out_markers",out_markers_);
  min_intensity_ = -1;
  this->addProperty("/VELMWHEEL_OROCOS_ROBOT/velmobil_global_localization/min_intensity",min_intensity_);
  this->addProperty("/VELMWHEEL_OROCOS_ROBOT/velmobil_global_localization/marker_position_tresh",marker_position_tresh_);

  if (marker_position_tresh_.size() < 2)
  {
      //std::cout<< "Marker position treshold not specified. Using default: [0.1, 0.1] " << "\n";
      marker_position_tresh_.resize(2);
      marker_position_tresh_ = {0.1, 0.1};
  }

  if (min_intensity_ == -1)
  {
      //std::cout<< "Minimal intensity not specified. Using default: 1500 " << "\n";
      min_intensity_ = 1500;
  }

current_loop_time_ptr.reset(new ros::Time());
  msg_laser_front_ptr.reset(new sensor_msgs::LaserScan());
  msg_laser_rear_ptr.reset(new sensor_msgs::LaserScan());
  msg_markers_ptr.reset(new visualization_msgs::Marker());
  msg_odom_transform_ptr.reset(new tf2_msgs::TFMessage());
  msg_save_map_ptr.reset(new std::string());
  msg_change_mode_ptr.reset(new int());
  
  new_data_vec_ptr.reset(new std::vector<bool>(2,false));
  scan_front_data_matrix.reset(new Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>);
  scan_rear_data_matrix.reset(new Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>);
}

VelmobilGlobalLocalization::~VelmobilGlobalLocalization() 
{
}
bool VelmobilGlobalLocalization::localize()
{
  myfile <<"-----   LOCALIZE   ------\n"; 

  for (global_iterator = 0; global_iterator < marker_counter; global_iterator++)
  {
    markers_cv.at(global_iterator) = cv::Point2f(markers.at(global_iterator)(0), markers.at(global_iterator)(1));
  }
  for (global_iterator = 0; global_iterator < map_marker_counter; global_iterator++)
  {
    map_markers_cv.at(global_iterator) = cv::Point2f(map_markers.at(global_iterator)(0), map_markers.at(global_iterator)(1));
  }
  transform_cv = cv::findHomography(markers_cv, map_markers_cv);
  myfile <<"CV transform: \n"; 
  myfile <<transform_cv<<"\n"; 

  return true;

}
// input: markers, marker_counter, map_markers
bool VelmobilGlobalLocalization::updateMarkers()
{
    myfile <<"marker_counter: "<< marker_counter <<"\n"; 
    myfile <<"markers SIZE: "<< markers.size() <<"\n"; 
    myfile <<"map_marker_counter: "<< map_marker_counter <<"\n"; 
    myfile <<"map_markers SIZE: "<< map_markers.size() <<"\n"; 

  for (global_iterator = 0; global_iterator < marker_counter; global_iterator++)
  {
    for (global_iterator_2 = 0; global_iterator_2 < map_marker_counter; global_iterator_2++)
    {
      if (abs(markers.at(global_iterator)(0) - map_markers.at(global_iterator_2)(0)) < marker_position_tresh_[0] 
          && abs(markers.at(global_iterator)(1) - map_markers.at(global_iterator_2)(1)) < marker_position_tresh_[1])
      {
        // needed to determine if the marker was found -> global_iterator_2 greater then map_marker_counter
        myfile <<"Marker in TRESH"<<"\n"; 
        global_iterator_2 = global_iterator_2 + map_marker_counter + 1;
        break;
      }
    }
    // marker found in map_markers, change pose of known marker
    if (global_iterator_2 > map_marker_counter)
    {
      // get the true global_iterator_2 count
      global_iterator_2 = global_iterator_2 - map_marker_counter - 1;
      map_markers.at(global_iterator_2) = map_markers.at(global_iterator_2) + (markers.at(global_iterator) - map_markers.at(global_iterator_2))/2;
      myfile <<"global_iterator_2: "<< global_iterator_2 <<"\n"; 
      myfile <<"global_iterator: "<< global_iterator<<"\n"; 
      myfile <<"map_marker_counter: "<< map_marker_counter <<"\n"; 
    }
    //marker not found in map_markers -> adding this marker to map
    else
    {
      myfile <<"Adding new marker to map"<<"\n"; 
      if (map_marker_counter + 1 > map_markers.size())
            RTT::Logger::log() << RTT::Logger::Warning << "[Global localization] -- reached max global markers count ( "<<map_markers.size()<<" )!!!! \n"<< RTT::Logger::endl;
      else
      {   
        map_markers.at(map_marker_counter) = markers.at(global_iterator);
        map_marker_counter += 1;
      }
    }
  }
  return true;
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

bool VelmobilGlobalLocalization::visualizationInitialization(visualization_msgs::Marker &vis_markers, const size_t &marker_id , const std::vector<Eigen::Vector2f> &positions)
{
vis_markers.header.frame_id = "odom";

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
  myfile <<"EXIT initialization \n"; 

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
  myfile <<"EXIT polarLaserToCartesianBase \n"; 

	return true;
}
bool VelmobilGlobalLocalization::configureHook() 
{

//std::cout<< "min_intensity_: " << min_intensity_<< "\n";
//std::cout<< "marker_position_tresh: \n[ " << marker_position_tresh_[0]<< ", "<<marker_position_tresh_[1]<<"\n" ;


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
  map_markers.resize(200);
  marker_id = 0;
  map_marker_counter = 0;
  msg_markers_ptr->points.resize(20);
  msg_markers_ptr->colors.resize(20);
  new_rising_edge_front = true;
  new_rising_edge_rear = true;
  rising_marker_iterator_front = 0;
  rising_marker_iterator_rear = 0;
//std::cout<< "marker size: " << markers.size()<< "\n";
  myfile.open ("/tmp/gl_log_data.txt");
  odom_transform.setIdentity();
  odom_quaternion.setIdentity();

  xml_tree.add("map.<xmlattr>.version", "1.0");

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
  // // // ODOM 
  //
if (RTT::NewData == in_odom_transform_.read((*msg_odom_transform_ptr)))
  {
////std::cout<< "--- ODOM --- " << "\n";

    odom_quaternion.x() =  msg_odom_transform_ptr->transforms.at(0).transform.rotation.x;
    odom_quaternion.y() =  msg_odom_transform_ptr->transforms.at(0).transform.rotation.y;
    odom_quaternion.z() =  msg_odom_transform_ptr->transforms.at(0).transform.rotation.z;
    odom_quaternion.w() =  msg_odom_transform_ptr->transforms.at(0).transform.rotation.w;

    odom_translation(0) =  msg_odom_transform_ptr->transforms.at(0).transform.translation.x;
    odom_translation(1) =  msg_odom_transform_ptr->transforms.at(0).transform.translation.y;

    odom_transform.topLeftCorner(2,2) = odom_quaternion.toRotationMatrix().topLeftCorner(2,2);
    odom_transform.col(2) << odom_translation(0), 
                              odom_translation(1), 
    //                          odom_translation(2), 
                              1;


  }

  // predict bias and calculate theta
  if (RTT::NewData == in_laser_front_.read((*msg_laser_front_ptr)))
  {

	scan_front_data_matrix->setZero();
  laser_in_base_transform << 1, 0, 0.3,
                             0, 1, 0,
                             0, 0, 1;
  //std::cout<< "ODOM: \n" <<odom_transform<< "\n";
  //std::cout<< "ODOM)inv: \n" <<odom_transform.inverse() << "\n";

	laser_in_odom_transform = odom_transform * laser_in_base_transform ;
  //polarLaserToCartesianBase(msg_laser_front_ptr->ranges, msg_laser_front_ptr->intensities, (*scan_front_data_matrix), laser_in_base_transform);
  //std::cout<< "FRONT: \n" <<laser_in_odom_transform<< "\n";
  
  polarLaserToCartesianBase(msg_laser_front_ptr->ranges, msg_laser_front_ptr->intensities, (*scan_front_data_matrix), laser_in_odom_transform);
  	new_data_vec_ptr->at(0) = true;
  }

  if (RTT::NewData == in_laser_rear_.read((*msg_laser_rear_ptr)))
  {

  scan_rear_data_matrix->setZero();
  laser_in_base_transform << -1, 0, -0.3,
                             0, -1, 0,
                             0, 0, 1;
  laser_in_odom_transform = odom_transform * laser_in_base_transform;
  //std::cout<< "REAR: \n" <<laser_in_odom_transform<< "\n";

  //polarLaserToCartesianBase(msg_laser_rear_ptr->ranges, msg_laser_rear_ptr->intensities, (*scan_rear_data_matrix), laser_in_base_transform);
  polarLaserToCartesianBase(msg_laser_rear_ptr->ranges, msg_laser_rear_ptr->intensities, (*scan_rear_data_matrix), laser_in_odom_transform);
    new_data_vec_ptr->at(1) = true;
  }

  if (new_data_vec_ptr->at(0) && new_data_vec_ptr->at(1))
  {
myfile << "remove markers \n";

    //removeMarkers( (*msg_markers_ptr));
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
  		if (((*scan_front_data_matrix)(global_iterator,2) < min_intensity_ || global_iterator == (scan_front_data_matrix->rows() - 1)) && !new_rising_edge_front)
  		{
        myfile << "SLOPING\n";
        new_rising_edge_front = true;
        marker_id = rising_marker_iterator_front + floor((global_iterator-rising_marker_iterator_front)/2);

        // check if both lasers found the same marker. If so, change marker pose to average

        for (global_iterator_2 = 0; global_iterator_2 < marker_counter; global_iterator_2++)
        {
          if (abs(markers.at(global_iterator_2)(0) - (*scan_front_data_matrix)(marker_id,0)) < marker_position_tresh_[0] 
            && abs(markers.at(global_iterator_2)(1) - (*scan_front_data_matrix)(marker_id,1)) < marker_position_tresh_[1])
          {
            myfile << "Both lasers found the same marker\n";
            myfile << "saved marker:\n";
            myfile << markers.at(global_iterator_2)(0) << "\n";
            myfile << markers.at(global_iterator_2)(1) << "\n";
            myfile << "front marker:\n";
            myfile << (*scan_front_data_matrix)(marker_id,0) << "\n";
            myfile << (*scan_front_data_matrix)(marker_id,1) << "\n";

            markers.at(global_iterator_2)(0) = markers.at(global_iterator_2)(0) + ((*scan_front_data_matrix)(marker_id,0) - markers.at(global_iterator_2)(0))/2;
            markers.at(global_iterator_2)(1) = markers.at(global_iterator_2)(1) + ((*scan_front_data_matrix)(marker_id,1) - markers.at(global_iterator_2)(1))/2;
            global_iterator_2 = marker_counter + 1;
            break;
          }
        }
        // add new marker if front laser didn't find one in nearby
        if (global_iterator_2 <= marker_counter )
        {
          markers.at(marker_counter) << (*scan_front_data_matrix)(marker_id,0),(*scan_front_data_matrix)(marker_id,1);
          myfile << "FRONT -marker_id: " << marker_id<<"\n";
          myfile << "FRONT -marker_intensity: " << (*scan_front_data_matrix)(marker_id,2)<<"\n";
          myfile << "FRONT -marker_counter: " << marker_counter<<"\n";
          myfile << "FRONT -markers: \n"<< markers.at(marker_counter)<<"\n";
          marker_counter += 1;          
        }
/*
  			//marker = {x_in_base, y_in_base}

        markers.at(marker_counter) << (*scan_front_data_matrix)(marker_id,0),(*scan_front_data_matrix)(marker_id,1);
  			new_rising_edge_front = true;
        myfile << "marker_id: " << marker_id<<"\n";
        myfile << "marker_intensity: " << (*scan_front_data_matrix)(marker_id,2)<<"\n";
        myfile << "marker_counter: " << marker_counter<<"\n";
        myfile << "markers: \n"<< markers.at(marker_counter)<<"\n";
        marker_counter += 1;
*/
  		}

      // [REAR LASER] found rising edge
      if ((*scan_rear_data_matrix)(global_iterator,2)  > min_intensity_ && new_rising_edge_rear)
      {
        myfile << "REAR - RISING\n";
        myfile << "global_iterator\n";
        myfile << global_iterator<<"\n";
        myfile << "scan_front_data_matrix->rows()\n";
        myfile << scan_front_data_matrix->rows()<<"\n";

        //calculate distance between last rising edge and cutten 
      rising_marker_iterator_rear = global_iterator;
      new_rising_edge_rear = false;
      }
      // found sloping edge
      if (((*scan_rear_data_matrix)(global_iterator,2) < min_intensity_ || global_iterator == (scan_front_data_matrix->rows() - 1)) && !new_rising_edge_rear)
      {
        myfile << "REAR -SLOPING\n";

        //marker = {x_in_base, y_in_base}
        marker_id = rising_marker_iterator_rear + floor((global_iterator-rising_marker_iterator_rear)/2);
        new_rising_edge_rear = true;
        
        // check if both lasers found the same marker. If so, change marker pose to average
        for (global_iterator_2 = 0; global_iterator_2 < marker_counter; global_iterator_2++)
        {
          if (abs(markers.at(global_iterator_2)(0) - (*scan_rear_data_matrix)(marker_id,0)) < marker_position_tresh_[0] 
            && abs(markers.at(global_iterator_2)(1) - (*scan_rear_data_matrix)(marker_id,1)) < marker_position_tresh_[1])
          {
            myfile << "Both lasers found the same marker\n";
            myfile << "front marker:\n";
            myfile << markers.at(global_iterator_2)(0) << "\n";
            myfile << markers.at(global_iterator_2)(1) << "\n";
            myfile << "rear marker:\n";
            myfile << (*scan_rear_data_matrix)(marker_id,0) << "\n";
            myfile << (*scan_rear_data_matrix)(marker_id,1) << "\n";

            markers.at(global_iterator_2)(0) = markers.at(global_iterator_2)(0) + ((*scan_rear_data_matrix)(marker_id,0) - markers.at(global_iterator_2)(0))/2;
            markers.at(global_iterator_2)(1) = markers.at(global_iterator_2)(1) + ((*scan_rear_data_matrix)(marker_id,1) - markers.at(global_iterator_2)(1))/2;
            global_iterator_2 = marker_counter + 1;
          }
        }
        // add new marker if front laser didn't find one in nearby
        if (global_iterator_2 <= marker_counter )
        {
          markers.at(marker_counter) << (*scan_rear_data_matrix)(marker_id,0),(*scan_rear_data_matrix)(marker_id,1);
          myfile << "REAR -marker_id: " << marker_id<<"\n";
          myfile << "REAR -marker_intensity: " << (*scan_rear_data_matrix)(marker_id,2)<<"\n";
          myfile << "REAR -marker_counter: " << marker_counter<<"\n";
          myfile << "REAR -markers: \n"<< markers.at(marker_counter)<<"\n";
          marker_counter += 1;          
        }
        
      }
   	}

    myfile << "-- initialization ---\n";
      visualizationInitialization( (*msg_markers_ptr), marker_counter, markers);
    out_markers_.write((*msg_markers_ptr));
 	new_data_vec_ptr->at(0) = false;
 	new_data_vec_ptr->at(1) = false;
  }

  
  // check component mode
  // state: 0 - mapping
  // state: 1 - SLAM

  if (RTT::NewData == in_change_mode_.read((*msg_change_mode_ptr)))
  {
    my_mode = (*msg_change_mode_ptr);
  }

  if (my_mode == 1)
  {
    localize();
    updateMarkers();
  }
  else
  {
    updateMarkers();
  }

  if (RTT::NewData == in_save_map_.read((*msg_save_map_ptr)))
  {
      myfile <<"save map \n"; 
      //xml_tree.put("map", "");

    for (global_iterator = 0; global_iterator < map_marker_counter; global_iterator++ )
    {
        marker_path.str("");
        marker_path << "marker_" << global_iterator;

        myfile <<" \n node_name:  \n"; 
        myfile << marker_path.str() << "\n";

        //marker_tree = xml_tree.add_child(marker_path.str(), boost::property_tree::ptree{});
        marker_path.str("");
        marker_path << "map.marker_"<<global_iterator<<".x";
        xml_tree.put(marker_path.str(), map_markers.at(global_iterator)(0));
        marker_path.str("");
        marker_path << "map.marker_"<<global_iterator<<".y";
        xml_tree.put(marker_path.str(), map_markers.at(global_iterator)(1));
        myfile <<" node_data:  \n"; 
      boost::property_tree::write_xml(myfile, xml_tree);    
        myfile <<" \n"; 

    }
    boost::property_tree::write_xml(*msg_save_map_ptr, xml_tree,
        std::locale(),
        boost::property_tree::xml_writer_make_settings<std::string>('\t', 1));    
    boost::property_tree::write_xml(myfile, xml_tree);    
  }
  loop_seq += 1;
      myfile <<" <<<<  NEXT LOOP \n"; 

}

ORO_CREATE_COMPONENT(VelmobilGlobalLocalization)
