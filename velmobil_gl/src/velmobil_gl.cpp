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
#include <Eigen/SparseCore>
#include <Eigen/IterativeLinearSolvers>
#include <iostream>
#include <fstream>
#include <cmath>

//Intensity map handler
#include <intensity_map_builder/intensity_map_handler.h>

//TF
#include "tf/tf.h"
#include <tf/transform_listener.h>
  // Pointers to msgs
  std::auto_ptr<sensor_msgs::LaserScan> msg_laser_front_ptr;
  std::auto_ptr<sensor_msgs::LaserScan> msg_laser_rear_ptr;
  std::auto_ptr<tf2_msgs::TFMessage> msg_odom_transform_ptr;
  std::auto_ptr<visualization_msgs::Marker> msg_markers_ptr;
  std::auto_ptr<std::string> msg_save_map_ptr;
  std::auto_ptr<std::string> msg_load_map_ptr;
  std::auto_ptr<int> msg_change_mode_ptr;
  std::auto_ptr<tf2_msgs::TFMessage> msg_base_map_tf_ptr;
  std::auto_ptr<geometry_msgs::PoseStamped> msg_pose_stamped_ptr;
  std::auto_ptr<geometry_msgs::Pose2D> msg_pose_2d_ptr;
  intensity_map_msgs::IntensityMarker intense_marker;

  // time variables
  uint32_t loop_time;
  uint64_t loop_seq;
  std::chrono::nanoseconds nsec;
  std::chrono::nanoseconds nsec_old;
  std::auto_ptr<ros::Time> current_loop_time_ptr; 

  // convertion euler<->quaternion
  Eigen::Quaternion<float> odom_quaternion;
  Eigen::Vector3f odom_translation;
  tf2::Quaternion transf_orient_quat_;
  // odometry transformations
  Eigen::Matrix<float,3,3> odom_transform;
  Eigen::Matrix<float,3,3> old_odom_transform_inverted;
  Eigen::Matrix<float,3,3> odom_transform_diff;
  Eigen::Matrix<float,3,3> laser_in_odom_transform;

  // scan data variables
  std::auto_ptr<std::vector<bool> > new_data_vec_ptr;
  std::auto_ptr<Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>> scan_front_data_matrix;
  size_t scan_front_size;
  std::auto_ptr<Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>> scan_rear_data_matrix;
  size_t scan_rear_size;
  std::vector<float> ranges;
  std::vector<float> angles;
  std::vector<int> intensities;

  // polarLaserToCartesianBase
  Eigen::Vector2f current_dist_check_vec;
  Eigen::Vector2f last_dist_check_vec;
  Eigen::Vector2f diff_dist_check_vec;

  // markers containers
  std::vector<Eigen::Vector3f> markers;
  std::vector<Eigen::Vector3f> map_markers;
  std::vector<Eigen::Vector2f> markers_in_map;
  size_t marker_id;
  size_t marker_counter;
  size_t map_marker_counter;

  // global iterators
  size_t global_iterator;
  size_t global_iterator_2;
  size_t global_iterator_3;
  size_t global_iterator_4;
  size_t rising_marker_iterator_front;
  size_t rising_marker_iterator_rear;

  // find markers
  bool new_rising_edge_front;
  bool new_rising_edge_rear;
  Eigen::Matrix<float,3,3> laser_in_base_transform;


    std::ofstream myfile;
// mode
  int my_mode;
// Intensity Map Handler
  std::auto_ptr<IntensityMapHandler> im_map_ptr;

// opencv
  cv::Mat transform_cv;
  std::vector<cv::Point2f> markers_cv;
  std::vector<cv::Point2f> map_markers_cv;
  std::vector<cv::Point2f> map_markers_matched_cv;

  // matchMarkers
  int matched_count;
  Eigen::Vector2f markers_memory;
  Eigen::Matrix<float,Eigen::Dynamic,3> map_markers_dist;
  Eigen::Matrix<float,Eigen::Dynamic,3> markers_dist;
  float matching_eps;
  float diff_x;
  float diff_y;
  float diff;
  Eigen::Vector2f diff_old;
  Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> map_markers_matched_eigen;
  Eigen::Matrix<float,3,3> transform_eigen;
  Eigen::Matrix<float,3,3> transform_eigen_inverted;
  Eigen::Matrix<float,Eigen::Dynamic,3> markers_eigen;
  Eigen::Matrix<float,Eigen::Dynamic,3> map_markers_eigen;
  float pointCurrentWeight;
  Eigen::Matrix<float,Eigen::Dynamic,1> pointCurrentMatches;
  float pointBestWeight;
  Eigen::Matrix<float,Eigen::Dynamic,1> pointBestMatches;
  Eigen::Matrix<float,1,3>  distance_memory;
  bool calc_match_initialized;
  Eigen::Matrix<float, Eigen::Dynamic, 1> matched_map_markers;
  size_t marker_counter_check_matched_markers;
  size_t best_matched_map;
  Eigen::Matrix<float, Eigen::Dynamic, 1> partial_cost;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matched_partial_cost;
  float matched_cost_thresh;
  float matched_cost_thresh_factor;
  // matchMarkersLocally
  Eigen::Matrix<float,1,3> marker_transformed;
	float	marker_pose_dist;
	float map_marker_pose_dist;
  bool msg_localization_initialized;
  float local_cost_match_thresh;
  int low_cost_local_match;
  size_t local_localization_marker_counter;
  Eigen::Matrix<float,Eigen::Dynamic,3> local_localization_markers_eigen;
  // iterators for matchMarkers
  size_t calcMarkDist_iter = 0;
  size_t calcMarkDist_iter_2 = 0;
  size_t calcMarkDist_iter_3 = 0;
  size_t calcWeight_iterator = 0;
  size_t calcWeight_iterator_2 = 0;
  // Leasat Squares Fit (LSF) localization
  Eigen::Matrix<float,4, 1> GL_product;
  Eigen::Matrix<float,Eigen::Dynamic, 1> projection;
  Eigen::Matrix<float,Eigen::Dynamic, 4> M;
  Eigen::Matrix<float,2,4> bestMatch; 
  Eigen::Matrix<float,2,4> secBestMatch;
  bool localization_initialized;
// visualization
  Eigen::Matrix<float,4, 1> vis_color;
  std::string vis_frame;
  std::string vis_namespace;
  visualization_msgs::MarkerArray vis_marker_array;

  // TF
  tf::TransformListener tf_listener;
  tf::StampedTransform tf_transform;


VelmobilGlobalLocalization::VelmobilGlobalLocalization(const std::string& name) : TaskContext(name)
{

  this->addPort("in_odom_transform",in_odom_transform_);
  this->addPort("in_laser_front",in_laser_front_);
  this->addPort("in_laser_rear",in_laser_rear_);
  this->addPort("in_change_mode",in_change_mode_);
  this->addPort("in_save_map",in_save_map_);
  this->addPort("in_load_map",in_load_map_);
  this->addPort("in_localization_initialized",in_localization_initialized_);
  this->addPort("out_markers",out_markers_);
  this->addPort("out_transform",out_transform_);
  this->addPort("out_pose_stamped",out_pose_stamped_);
  this->addPort("out_pose_2d",out_pose_2d_);
  this->addPort("out_intense_markers",out_intense_markers_);
  min_intensity_ = -1;
  local_cost_match_thresh = 0;
  matched_cost_thresh_factor = -1;
  this->addProperty("/VELMWHEEL_OROCOS_ROBOT/velmobil_global_localization/min_intensity",min_intensity_);
  this->addProperty("/VELMWHEEL_OROCOS_ROBOT/velmobil_global_localization/marker_position_tresh",marker_position_tresh_);
  this->addProperty("/VELMWHEEL_OROCOS_ROBOT/velmobil_global_localization/local_cost_match_thresh",local_cost_match_thresh);
  this->addProperty("/VELMWHEEL_OROCOS_ROBOT/velmobil_global_localization/matched_cost_thresh_factor",matched_cost_thresh_factor);
  if (marker_position_tresh_.size() < 2)
  {
      //myfile<< "Marker position treshold not specified. Using default: [0.1, 0.1] " << "\n";
      marker_position_tresh_.resize(2);
      marker_position_tresh_ = {0.1, 0.1};
  }

  if (min_intensity_ == -1)
  {
      //myfile<< "Minimal intensity not specified. Using default: 1500 " << "\n";
      min_intensity_ = 1500;
  }
  if (local_cost_match_thresh == 0)
  {
      //myfile<< "Max cost of a locally matched marker. Default: 0.25 m " << "\n";
      local_cost_match_thresh = 0.25;
  }
  if (matched_cost_thresh_factor == -1)
  {
      matched_cost_thresh_factor = 0.15;
  }
current_loop_time_ptr.reset(new ros::Time);
  msg_laser_front_ptr.reset(new sensor_msgs::LaserScan);
  msg_laser_rear_ptr.reset(new sensor_msgs::LaserScan);
  msg_markers_ptr.reset(new visualization_msgs::Marker);
  msg_odom_transform_ptr.reset(new tf2_msgs::TFMessage);
  msg_save_map_ptr.reset(new std::string);
  msg_load_map_ptr.reset(new std::string);
  msg_change_mode_ptr.reset(new int);
  msg_base_map_tf_ptr.reset(new tf2_msgs::TFMessage);

  new_data_vec_ptr.reset(new std::vector<bool>(2,false));
  scan_front_data_matrix.reset(new Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>);
  scan_rear_data_matrix.reset(new Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic>);
  msg_pose_stamped_ptr.reset(new geometry_msgs::PoseStamped);
  msg_pose_2d_ptr.reset(new geometry_msgs::Pose2D);
  bool msg_localization_initialized;
}

VelmobilGlobalLocalization::~VelmobilGlobalLocalization() 
{
}
// bestMatch.row(0) << marker_eigen values , match_weight 
// bestMatch.row(1) << map_markers_matched_eigen values (match_weight containing match_weight)
bool VelmobilGlobalLocalization::get2BestMatchedMarkersByWeight()
{
	bestMatch(0,3) = map_markers_matched_eigen(0,3) + map_markers_matched_eigen(1,3);
	secBestMatch(0,3) = map_markers_matched_eigen(0,3) + map_markers_matched_eigen(1,3);
	for (global_iterator = 0; global_iterator < marker_counter; ++global_iterator)
	{
		if (map_markers_matched_eigen(global_iterator,3) < bestMatch(0,3))
		{
      secBestMatch.row(1) << bestMatch.row(1);
      secBestMatch.row(0) << bestMatch.row(0);

			bestMatch.row(1) << map_markers_matched_eigen.row(global_iterator);
			bestMatch.row(0) << markers_eigen.row(global_iterator), map_markers_matched_eigen(global_iterator,3);
      continue;
		}
		if (map_markers_matched_eigen(global_iterator,3) < secBestMatch(0,3))
		{
			secBestMatch.row(1) << map_markers_matched_eigen.row(global_iterator);
			secBestMatch.row(0) << markers_eigen.row(global_iterator), map_markers_matched_eigen(global_iterator,3);
		}	
	}
}
bool VelmobilGlobalLocalization::calcLSFtransform_2Best()
{
	get2BestMatchedMarkersByWeight();
  /*
	M.row(0) << bestMatch(1,0), bestMatch(1,1), 1, 0;
	M.row(1) << bestMatch(1,1), - bestMatch(1,0), 0, 1;
    projection(0) = bestMatch(0,0);
    projection(1) = bestMatch(0,1);
	M.row(2) << secBestMatch(1,0), secBestMatch(1,1), 1, 0;
	M.row(3) << secBestMatch(1,1), - secBestMatch(1,0), 0, 1;
    projection(2) = secBestMatch(0,0);
    projection(3) = secBestMatch(0,1);
        myfile <<"bestMatch: \n"; 
    myfile <<bestMatch<<"\n"; 
        myfile <<"secBestMatch: \n"; 
    myfile <<secBestMatch<<"\n"; 

    myfile <<"M: \n"; 
    myfile <<M<<"\n"; 

    myfile <<"projection: \n"; 
    myfile <<projection; 

    GL_product = M.block(0,0,4,4).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(projection.block(0,0,4,1));
	transform_eigen << GL_product(0), GL_product(1), GL_product(2),
                  -GL_product(1),  GL_product(0), GL_product(3),
                      0    ,      0   ,     1;
                      */
  
odom_quaternion = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
                    * Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf((atan2(bestMatch(1,1) - secBestMatch(1,1),bestMatch(1,0) - secBestMatch(1,0)) - atan2(bestMatch(0,1) - secBestMatch(0,1),bestMatch(0,0) - secBestMatch(0,0))), Eigen::Vector3f::UnitZ());


  transform_eigen(0,0) = cos(atan2(bestMatch(1,1) - secBestMatch(1,1),bestMatch(1,0) - secBestMatch(1,0)) - atan2(bestMatch(0,1) - secBestMatch(0,1),bestMatch(0,0) - secBestMatch(0,0)));
  transform_eigen(0,1) =  sin(atan2(bestMatch(1,1) - secBestMatch(1,1),bestMatch(1,0) - secBestMatch(1,0)) - atan2(bestMatch(0,1) - secBestMatch(0,1),bestMatch(0,0) - secBestMatch(0,0)));
  transform_eigen(0,2) = (bestMatch(1,0) - bestMatch(0,0) + secBestMatch(1,0) - secBestMatch(0,0))/2;

  transform_eigen(1,0) =  -transform_eigen(0,1);
  transform_eigen(1,1) = transform_eigen(0,0) ;
  transform_eigen(1,2) = bestMatch(1,1) - bestMatch(0,1);
  transform_eigen.row(2) << 0    ,      0   ,     1;

    myfile <<"MY transform: \n"; 
    myfile <<transform_eigen<<"\n"; 
       transform_eigen_inverted.topLeftCorner(2,2) = transform_eigen.topLeftCorner(2,2).transpose();
    transform_eigen_inverted.col(2) << -(transform_eigen_inverted(0,0)*transform_eigen(0,2) + transform_eigen_inverted(0,1)*transform_eigen(1,2)), 
                              -(transform_eigen_inverted(1,0)*transform_eigen(0,2) + transform_eigen_inverted(1,1)*transform_eigen(1,2)),
                              1;


	transform_eigen_inverted = transform_eigen.inverse();

   msg_pose_2d_ptr->x = (bestMatch(1,0) - bestMatch(0,0) + secBestMatch(1,0) - secBestMatch(0,0))/2;
  msg_pose_2d_ptr->y = (bestMatch(1,1) - bestMatch(0,1) + secBestMatch(1,1) - secBestMatch(0,1))/2;
  msg_pose_2d_ptr->theta = (atan2(transform_eigen(0,1),transform_eigen(0,0)));
  out_pose_2d_.write(*msg_pose_2d_ptr);

	return true;
}
bool VelmobilGlobalLocalization::calcConjugateGradient()
{
  global_iterator_2 = 0;
  //marker_counter = 2;
 	for (global_iterator = 0; global_iterator < marker_counter; ++global_iterator)
    {
		M.row(global_iterator_2) << map_markers_matched_eigen(global_iterator,0), map_markers_matched_eigen(global_iterator,1),1, 0;
		M.row(global_iterator_2 + 1) << map_markers_matched_eigen(global_iterator,1), -map_markers_matched_eigen(global_iterator,0), 0, 1;
	    projection(global_iterator_2) = markers_eigen(global_iterator,0);
	    projection(global_iterator_2+1) = markers_eigen(global_iterator,1);
     myfile <<"global_iterator: " << global_iterator<< "\n"; 
	    global_iterator_2+=2;
    }
     myfile <<"M: \n" << M.block(0,0,marker_counter*2,4)<< "\n"; 
     myfile <<"projection: \n" << projection.block(0,0,marker_counter*2,1)<< "\n"; 

	Eigen::ConjugateGradient<Eigen::Matrix<float, -1, -1>, Eigen::Lower|Eigen::Upper> cg;
	cg.setMaxIterations(10);
	cg.setTolerance(0.05);
  Eigen::Matrix<float, 4, 4> my_product;
  my_product = M.block(0,0,marker_counter*2,4);
	cg.compute(my_product);
  Eigen::Matrix<float, 4, 1> guess;
	guess = GL_product;
      GL_product = M.block(0,0,marker_counter*2,4).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(projection.block(0,0,marker_counter*2,1));

//  GL_product = cg.solveWithGuess(projection.block(0,0,marker_counter*2,1),guess);
  //GL_product = cg.solve(projection.block(0,0,marker_counter*2,1));
     myfile <<"my_product: \n" << GL_product<< "\n"; 

		transform_eigen << GL_product(0), GL_product(1), GL_product(2),
                  -GL_product(1),  GL_product(0), GL_product(3),
                      0    ,      0   ,     1;
  msg_pose_2d_ptr->x = GL_product(2);
  msg_pose_2d_ptr->y = GL_product(3);
  msg_pose_2d_ptr->theta = (atan2(GL_product(1),GL_product(0)));
out_pose_2d_.write(*msg_pose_2d_ptr);

    myfile <<"MY transform: \n"; 
    myfile <<transform_eigen<<"\n";
    myfile <<"Pose2D: \n"; 
    myfile <<*msg_pose_2d_ptr<<"\n";

	transform_eigen_inverted = transform_eigen.inverse();

	return true;
}
bool VelmobilGlobalLocalization::calcLSFtransform()
{
	global_iterator_2 = 0;
    myfile <<"map_markers_matched_eigen.row(global_iterator).head(2): \n"; 
    myfile <<map_markers_matched_eigen(0,0)<<", "<<map_markers_matched_eigen(0,1)<<"\n";
    for (global_iterator = 0; global_iterator < marker_counter; ++global_iterator)
    {
		M.row(global_iterator_2) << map_markers_matched_eigen(global_iterator,0), map_markers_matched_eigen(global_iterator,1),1, 0;
		M.row(global_iterator_2 + 1) << map_markers_matched_eigen(global_iterator,1), -map_markers_matched_eigen(global_iterator,0), 0, 1;
	    projection(global_iterator_2) = markers_eigen(global_iterator,0);
	    projection(global_iterator_2+1) = markers_eigen(global_iterator,1);
	//      myfile <<"global_iterator: " << global_iterator<< "\n"; 
	    global_iterator_2+=2;
    }
    myfile <<"M: \n"; 
    myfile <<M<<"\n";
//    myfile <<"M: \n"; 
//    myfile <<M; 

//    myfile <<"projection: \n"; 
//    myfile <<projection; 

    myfile <<"CALC LOCALIZATION \n"; 
    GL_product = M.block(0,0,marker_counter*2,4).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(projection.block(0,0,marker_counter*2,1));
	transform_eigen << GL_product(0), GL_product(1), GL_product(2),
                  -GL_product(1),  GL_product(0), GL_product(3),
                      0    ,      0   ,     1;
  msg_pose_2d_ptr->x = GL_product(2);
  msg_pose_2d_ptr->y = GL_product(3);
  msg_pose_2d_ptr->theta = (atan2(GL_product(1),GL_product(0)));
out_pose_2d_.write(*msg_pose_2d_ptr);

    myfile <<"MY transform: \n"; 
    myfile <<transform_eigen<<"\n";
    myfile <<"Pose2D: \n"; 
    myfile <<*msg_pose_2d_ptr<<"\n";

       transform_eigen_inverted.topLeftCorner(2,2) = transform_eigen.topLeftCorner(2,2).transpose();
    transform_eigen_inverted.col(2) << -(transform_eigen_inverted(0,0)*transform_eigen(0,2) + transform_eigen_inverted(0,1)*transform_eigen(1,2)), 
                              -(transform_eigen_inverted(1,0)*transform_eigen(0,2) + transform_eigen_inverted(1,1)*transform_eigen(1,2)),
                              1;

	//transform_eigen_inverted = transform_eigen.inverse();
	return true;
}
bool VelmobilGlobalLocalization::localizeLSF()
{
  //
  //  math explanation https://stackoverflow.com/questions/11687281/transformation-between-two-set-of-points
  // LSE recursive -> http://www.cs.tut.fi/~tabus/course/ASP/LectureNew10.pdf
  myfile <<"-----   TRY LOCALIZE   ------\n"; 
  for (global_iterator = 0; global_iterator < marker_counter; global_iterator++)
  {
    markers_eigen.row(global_iterator) << markers.at(global_iterator)(0), markers.at(global_iterator)(1), 1;  
  }
	// if GL is initialized use markers local matching using at least 2 markers and 2 map_markers
	if (marker_counter > 1 && map_marker_counter > 1)
  {
    if (localization_initialized)
  	{
  myfile << "INITIALIZED\n"; 

  		// if local matching returns false ( there are less then 2 markers mathed with cost less then local_cost_match_thresh )
  		// use matchMarkersEIGEN method - reinitialize global localization 
      // matchMarkersEigen requires at least 3 markers and 3 map_markers
  bool local_status = matchMarkersLocally();
  		if(!local_status)
      {
           myfile << "local localization failed\n";
           myfile << "marker_counter: "<<marker_counter<<"\n";
           myfile << "map_marker_counter: "<<map_marker_counter<<"\n";
        if (marker_counter > 2 && map_marker_counter > 2)
        {
           myfile << "REINITIALIZATION\n";
           std::cout << "REINITIALIZATION\n";

  	  	  matchMarkersEIGEN();
        }else 
          return false;
      }

    }
    // if GL is NOT initialized, use matchMarkersEIGEN to match markers
    else
    {
      if (marker_counter > 2 && map_marker_counter > 2)
        {
          myfile << "INITIALIZATION\n";
          matchMarkersEIGEN();
          localization_initialized = true;
        }else 
          return false;

    }
    // if there are at least 3 markers matched to the map
    // fit found markers to matched markers by least squared error minimalization 
    if (marker_counter > 2 && map_marker_counter > 2)
    {
      calcLSFtransform();	
      return true;
    }
  }
  return false;
	


/*


	if (!localization_initialized)
	{
		if (marker_counter > 2 && map_marker_counter > 2)
  		{
	    myfile <<"-----   LOCALIZATION INITIALIZATION   ------\n"; 



	    matchMarkersEIGEN();
	    myfile <<"EIGEN markers: \n" << markers_eigen << "\n"; 
	    myfile <<"EIGEN map_markers_matched_eigen: \n" << map_markers_matched_eigen<< "\n"; 
	    
	    calcLSFtransform();
	    
	    //calcLSFtransform_2Best();
	    localization_initialized = true;
	    return true;
	  }
	  else
	  {
	    return false;
	  }
	}
	// if GL is initialized use conjugened gradirnt method
	else
	{
    myfile << "LOCALIZATION using initialized transform\n";
		// we need to find 2 markers at least
		if (marker_counter > 1 && map_marker_counter > 1)
		{
			if (matchMarkersLocally())
	    	calcLSFtransform();
	    else
	    {
		    matchMarkersEIGEN();
		    calcLSFtransform();
		    localization_initialized = false;
	    }
			//calcConjugateGradient();
			return true;
		}
		// we found less then 2 markers in this loop
		else
		{
			return false;
		}
	}
  
*/
}

/*
bool VelmobilGlobalLocalization::checkMatchedMarkers()
{
  marker_counter_check_matched_markers = marker_counter;
  for (global_iterator = 0; global_iterator < count; ++global_iterator)
  {
    if (map_markers_matched_eigen.row(global_iterator).tail(1) > matched_cost_thresh)
    {
      for (global_iterator_2 = global_iterator; global_iterator_2 < marker_counter; global_iterator_2++)
      {     
        markers_eigen.row(global_iterator_2) = markers_eigen.row(global_iterator_2+1);
        matched_map_markers()
      }
      // markers list becomes shorter 
      --marker_counter;
    }
  }
  if (marker_counter_check_matched_markers != marker_counter)
    return false;
  else
    return true;
}
*/
bool VelmobilGlobalLocalization::localizeUmeyama()
{
  
    myfile <<"-----   TRY LOCALIZE   ------\n"; 

  if (marker_counter > 1 && map_marker_counter > 1)
  {

    myfile <<"-----   LOCALIZE   ------\n";
    for (global_iterator = 0; global_iterator < marker_counter; global_iterator++)
    {
      markers_eigen.row(global_iterator) << markers.at(global_iterator)(0), markers.at(global_iterator)(1), 1;  
    }
    for (global_iterator = 0; global_iterator < map_marker_counter; global_iterator++)
    {
      map_markers_eigen.row(global_iterator) << map_markers.at(global_iterator)(0), map_markers.at(global_iterator)(1), 1;
    }

    matchMarkersEIGEN();
    myfile <<"EIGEN markers: \n" << markers_eigen << "\n"; 
    myfile <<"EIGEN map_markers_matched_eigen: \n" << map_markers_matched_eigen<< "\n"; 

    transform_eigen = Eigen::umeyama (markers_eigen, map_markers_matched_eigen, false);
    myfile <<"CV transform: \n"; 
    myfile <<transform_eigen<<"\n"; 
transform_eigen_inverted = transform_eigen.inverse();
        return true;
  }
  else
  {
    return false;
  }

}
bool VelmobilGlobalLocalization::calcMatchWeight(float &matchWeight, Eigen::Matrix<float,Eigen::Dynamic,1> &match_ids)
{
  myfile <<"calcMatchWeight\n";
  myfile <<"marker_counter\n";
  myfile <<marker_counter<<"\n";
  myfile <<"map_marker_counter\n";
  myfile <<map_marker_counter<<"\n"; 
	map_markers_dist.col(2).setZero();
matchWeight = 0;
calc_match_initialized = false;
	for (calcWeight_iterator = 0; calcWeight_iterator < marker_counter; ++calcWeight_iterator)
	{
    // set initial value 
    diff_old << 10000, 0;
		for (calcWeight_iterator_2 = 0; calcWeight_iterator_2 < map_marker_counter; ++calcWeight_iterator_2)
		{
			// check if this map_markers_dist was matched before
			if (map_markers_dist(calcWeight_iterator_2,2) != 1 )
			{
				diff = fabs(map_markers_dist(calcWeight_iterator_2,0) - markers_dist(calcWeight_iterator,0));
        if (!calc_match_initialized)
        {
          diff_old << diff, calcWeight_iterator_2;  
          calc_match_initialized = true;
        }
         myfile << "diff:" <<"\n";
        myfile << diff << "\n";
				// check if current match is worse then the previous one 
				if( diff_old(0) <= diff )
				{
					// if so match the scan_point_dist to the prevoius map_markers_dist
					match_ids(markers_dist(calcWeight_iterator,1)) = map_markers_dist(diff_old(1),1);
					// add the distance diff to the match weight
					matchWeight += diff_old(0);
          partial_cost(calcWeight_iterator) =diff_old(0);
					// mark previous map_markers_dist as matched
					map_markers_dist(diff_old(1),2) = 1;
//          myfile << "BREAK" <<"\n";
        diff_old << diff, calcWeight_iterator_2;  
					break;
				}
				if (calcWeight_iterator_2 == map_marker_counter - 1)
				{
					match_ids(markers_dist(calcWeight_iterator,1)) = map_markers_dist(calcWeight_iterator_2,1);
					matchWeight += diff;
          partial_cost(calcWeight_iterator) = diff;
          myfile << "++++++LAST MARKER +++++" << "\n";
					// mark map_markers_dist as matched
					map_markers_dist(calcWeight_iterator_2,2) = 1; 
					break;
				}					
				diff_old << diff, calcWeight_iterator_2;				
//        myfile << "diff_old:" <<"\n";
//        myfile << diff_old << "\n";
			}
			if (calcWeight_iterator_2 == map_marker_counter - 1)
			{
				match_ids(markers_dist(calcWeight_iterator,1)) = map_markers_dist(diff_old(1),1);
				matchWeight += diff_old(0);
        partial_cost(calcWeight_iterator) =diff_old(0);
				// mark map_markers_dist as matched
          myfile << "***** LAST MARKER ******" << "\n";

				map_markers_dist(diff_old(1),2) = 1; 
				break;
			}				
		}
	}	
  
  myfile  << "matchWeight:" <<"\n";
  myfile  << matchWeight << "\n";
  myfile  << "partial_cost:" <<"\n";
  myfile  << partial_cost << "\n";
  
//  myfile << "MATCHES:" <<"\n";
//  myfile << match_ids << "\n";
  return true;
}
bool VelmobilGlobalLocalization::calcMarkDistEIGEN(const Eigen::Matrix<float,Eigen::Dynamic,3> &input_markers, const int &marker_size, const int &respect_marker, Eigen::Matrix<float,Eigen::Dynamic,3> &distances)
{
  myfile << "calcMarkDistEIGEN\n";
for (calcMarkDist_iter = 0; calcMarkDist_iter < marker_size; ++calcMarkDist_iter)
  {
  	// distances [ [distance, point_id, isMatched]
  	//			               ...
  	//             [distance, point_id, isMatched]	]
    distances(calcMarkDist_iter,0) = sqrt(pow(input_markers(calcMarkDist_iter,0) - input_markers(respect_marker,0), 2) + 
                                        pow(input_markers(calcMarkDist_iter,1) - input_markers(respect_marker,1), 2));
    distances(calcMarkDist_iter,1) = calcMarkDist_iter; 
    distances(calcMarkDist_iter,2) = 0;
	for (calcMarkDist_iter_2 = 0; calcMarkDist_iter_2 < calcMarkDist_iter; ++calcMarkDist_iter_2)
	{
		if (distances(calcMarkDist_iter,0) < distances(calcMarkDist_iter_2,0))
		{
			distance_memory << distances.row(calcMarkDist_iter);
			for (calcMarkDist_iter_3 = calcMarkDist_iter; calcMarkDist_iter_3 > calcMarkDist_iter_2; --calcMarkDist_iter_3)
			{			
				distances.row(calcMarkDist_iter_3) = distances.row(calcMarkDist_iter_3-1);
			}
			distances.row(calcMarkDist_iter_2) << distance_memory;
		}
	}

  }

  return true;
}
 bool VelmobilGlobalLocalization::matchMarkersLocally()
 {
  myfile << "matchMarkersLocally\n";
	
	low_cost_local_match = 0;
  local_localization_marker_counter = marker_counter;
  local_localization_markers_eigen = markers_eigen;
 	for (global_iterator = 0; global_iterator < local_localization_marker_counter; ++global_iterator)
	{
	 	// ++++ ADD +++ 
	 	// Transform markers_eigen to the map frame


		marker_transformed = transform_eigen_inverted * local_localization_markers_eigen.row(global_iterator).transpose() ;

  myfile << "marker_transformed:\n";
  myfile << transform_eigen_inverted <<"\n";
  myfile << "\n * \n";
  myfile << local_localization_markers_eigen.row(global_iterator).transpose()<<"\n = \n"<< marker_transformed << "\n";
  myfile << "transform_eigen:\n";
  myfile << transform_eigen;
		// match closest map_marker to the transformed marker
		//
		// initialize diff calculation
		marker_pose_dist = sqrt(pow(marker_transformed(0),2) + pow(marker_transformed(1),2));
		map_marker_pose_dist = sqrt(pow(map_markers_eigen(0, 0),2) + pow(map_markers_eigen(0, 1),2));
		diff = sqrt(pow(marker_transformed(0) - map_markers_eigen(0, 0),2) + pow(marker_transformed(1) - map_markers_eigen(0, 1),2));
	 	diff_old << diff,0;
	 	map_markers_matched_eigen.row(global_iterator) << map_markers_eigen.row(0), 0;
	 	// search map_markers for the closest marker to the marksers(global_iterator), the map_markers(0) is handeled in above initialization
		for (global_iterator_2 = 1; global_iterator_2 < map_marker_counter; ++global_iterator_2)
		{
			diff = sqrt(pow(marker_transformed(0) - map_markers_eigen(global_iterator_2, 0),2) + pow(marker_transformed(1) - map_markers_eigen(global_iterator_2, 1),2));
			if (diff <= diff_old(0))
			{
				map_markers_matched_eigen.row(global_iterator) << map_markers_eigen.row(global_iterator_2), diff;
				diff_old(0) = diff;
			}
		}
		
		if (diff_old(0) < local_cost_match_thresh)
    { 
       myfile << "\nCOST-OK\n";
           low_cost_local_match += 1;
    }
    // if the cost is not low enough, delete found marker from the list
    // This marker was probably not included in the map
    else
    {
       myfile << "\nDeleting found marker from memory. This marker was probably not included in the map\n";

    	for (global_iterator_2 = global_iterator; global_iterator_2 < local_localization_marker_counter; global_iterator_2++)
			{			
				local_localization_markers_eigen.row(global_iterator_2) = local_localization_markers_eigen.row(global_iterator_2+1);
			}
			// markers list becomes shorter 
			--local_localization_marker_counter;
			// we need to stay at the same global_iterator to process the next marker
			--global_iterator;
    }

	}
    myfile << "MARKERS:\n";
    myfile << local_localization_markers_eigen <<"\n";
    myfile << "MATCHED:\n";
    myfile << map_markers_matched_eigen <<"\n";

    myfile << "--- END matchMarkersLocally witch low_cost_local_match: "<<low_cost_local_match<<" ---\n";

    if (low_cost_local_match >= 2)
    {
      markers_eigen = local_localization_markers_eigen;
      marker_counter = local_localization_marker_counter;
      myfile << "returning true:\n";
			return true;
    }
		else
			return false;

 }


bool VelmobilGlobalLocalization::matchMarkersEIGEN()
{
  myfile <<"--- MATCHING ----" <<"\n"; 
  myfile  <<"markers:" <<"\n"; 
  for (global_iterator = 0; global_iterator < marker_counter; ++global_iterator)
  {
    myfile <<markers_eigen.row(global_iterator) <<"\n"; 
  }
  // 1) calculate distance between marker(0) and other markers 
  matched_map_markers = 9000 * matched_map_markers.setOnes();
  for (global_iterator = 0; global_iterator < marker_counter; ++global_iterator)
  {
    // limit for distance error for one dist to one dist is matched_cost_thresh_factor ( default 0,15 m). Thus threshold one marker_found match equals a nr of found markers "marker_counter" * matched_cost_thresh_factor
    matched_cost_thresh = marker_counter * matched_cost_thresh_factor;
    //restart when one of the found markers has high cost of the best match
    if(!matchOneMarkerEIGEN(global_iterator))
    { 
    myfile <<"+++++ REMOVE MARKER +++++" <<"\n"; 
      // replace this marker_found by the next one
      for (global_iterator_2 = global_iterator; global_iterator_2 < marker_counter; global_iterator_2++)
      {     
        markers_eigen.row(global_iterator_2) = markers_eigen.row(global_iterator_2+1);
      }
      // markers list becomes shorter 
      --marker_counter;
      // restart matching
      global_iterator = -1;
      matched_map_markers = 9000 * matched_map_markers.setOnes();

    }
  }
  myfile  <<"markers:" <<"\n"; 
  for (global_iterator = 0; global_iterator < marker_counter; ++global_iterator)
  {
    myfile <<markers_eigen.row(global_iterator) <<"\n"; 
  }
  myfile  <<"MATCHED MARKERS:" <<"\n"; 
     for (global_iterator_2 = 0; global_iterator_2 < marker_counter; ++global_iterator_2)
    {
       myfile  <<map_markers_matched_eigen.row(global_iterator_2) <<"\n"; 
    }

  return true;
}

bool VelmobilGlobalLocalization::matchOneMarkerEIGEN(size_t &marker_iterator)
{
    calcMarkDistEIGEN(markers_eigen, marker_counter, marker_iterator, markers_dist);
    myfile <<"--- NEXT MARKER MATCHING ----" <<"\n"; 
    myfile <<"markers_dist:" <<"\n"; 
    for (global_iterator_2 = 0; global_iterator_2 < marker_counter; ++global_iterator_2)
    {
      myfile <<markers_dist.row(global_iterator_2) <<"\n"; 
    }

    // 2) calculate distances between map_markers with respect to map_marker(marker_iterator)
    pointBestWeight = 10000;
    pointBestMatches.setZero();
    pointCurrentWeight = 0;
    pointCurrentMatches.setZero();
    for (global_iterator_2 = 0; global_iterator_2 < map_marker_counter; ++global_iterator_2)
    {
      myfile  <<"calcualte map_markers_dist" <<"\n"; 
      calcMarkDistEIGEN(map_markers_eigen, map_marker_counter, global_iterator_2, map_markers_dist);

      myfile <<"map_markers_dist:" <<"\n";
      for (global_iterator_3 = 0; global_iterator_3 < map_marker_counter; ++global_iterator_3)
      {
        myfile <<map_markers_dist.row(global_iterator_3) <<"\n"; 
      }

      calcMatchWeight(pointCurrentWeight, pointCurrentMatches);
      if (pointCurrentWeight < pointBestWeight)
      {
        pointBestWeight = pointCurrentWeight;
        pointBestMatches = pointCurrentMatches;
          myfile  <<"map_marker_cost: " <<matched_map_markers(global_iterator_2)<<"\n";
          myfile  <<"my_cost: " <<pointBestWeight<<"\n";
        if (pointBestWeight < matched_map_markers(global_iterator_2))
        {
          myfile  <<"---- SET MATCHED ----" <<"\n";
          map_markers_matched_eigen.row(marker_iterator) << map_markers_eigen.row(global_iterator_2), pointBestWeight;
          best_matched_map = global_iterator_2;
          matched_partial_cost.row(marker_iterator) << partial_cost.transpose();
        }
        //map_markers_matched_eigen.row(marker_iterator) << map_markers_eigen.row(pointBestMatches(marker_iterator)), pointBestWeight;
      }
    }
    // sprawdz czy dopasowanie dla badanego marker_found spelnia kryterium kosztu 
          myfile  <<"matched_cost_thresh: " <<matched_cost_thresh<<"\n";
          myfile  <<"my_cost: " <<pointBestWeight<<"\n";
    if (pointBestWeight > matched_cost_thresh)
    {
      return false;
    }       
    // jesli spelnia to ustaw nowa wartosc najlepszego kosztu dla danego markera mapy
    matched_map_markers(best_matched_map) = pointBestWeight;
    return true;
}

bool VelmobilGlobalLocalization::localizeCV()
{
  if (marker_counter > 1 && map_marker_counter > 1)
  {

    myfile <<"-----   LOCALIZE   ------\n"; 
    map_markers_matched_cv.resize(marker_counter);
    markers_cv.resize(marker_counter);
    for (global_iterator = 0; global_iterator < marker_counter; global_iterator++)
    {
      markers_cv.at(global_iterator) = cv::Point2f(markers.at(global_iterator)(0), markers.at(global_iterator)(1));  }
    for (global_iterator = 0; global_iterator < map_marker_counter; global_iterator++)
    {
      map_markers_cv.at(global_iterator) = cv::Point2f(map_markers.at(global_iterator)(0), map_markers.at(global_iterator)(1));
    }

    myfile <<"CV markers: \n" << markers_cv << "\n"; 
    //myfile <<"CV map_markers_cv: \n" << map_markers_cv; 
    matchMarkersCV();
      myfile <<"CV markers: \n" << markers_cv<< "\n"; 
    myfile <<"CV map_markers_matched_cv: \n" << map_markers_matched_cv<< "\n"; 

    transform_cv = cv::findHomography(markers_cv, map_markers_matched_cv);
    myfile <<"CV transform: \n"; 
    myfile <<transform_cv<<"\n"; 

    return true;
  }
  else
  {
    return false;
  }

}


bool VelmobilGlobalLocalization::calcMarkDistCV(const std::vector<cv::Point2f> &input_markers, const int &marker_size, const int &respect_marker, Eigen::Matrix<float,Eigen::Dynamic,3> &distances, size_t &my_iterator)
{
for (my_iterator = 0; my_iterator < marker_size; ++my_iterator)
  {
    distances(my_iterator,0) = sqrt(pow(input_markers.at(my_iterator).x - input_markers.at(respect_marker).x, 2) + 
                                        pow(input_markers.at(my_iterator).y - input_markers.at(respect_marker).y, 2)); 
  }


  return true;
}
bool VelmobilGlobalLocalization::matchMarkersCV()
{
  myfile <<"--- MATCHING ----" <<"\n"; 

  // 1) calculate distance between marker(0) and other markers 
  calcMarkDistCV(markers_cv, marker_counter, 0, markers_dist, global_iterator);
  myfile <<"markers_dist:" <<"\n"; 
  for (global_iterator_3 = 0; global_iterator_3 < marker_counter; ++global_iterator_3)
  {
    myfile <<markers_dist(global_iterator_3,0) <<"\n"; 
  }

  // 2) calculate distances between map_markers with respect to map_marker(global_iterator)
  for (global_iterator = 0; global_iterator < map_marker_counter; ++global_iterator)
  {
    myfile <<"calcualte map_markers_dist" <<"\n"; 
    calcMarkDistCV(map_markers_cv, map_marker_counter, global_iterator, map_markers_dist, global_iterator_2);
    myfile <<"map_markers_dist:" <<"\n";
    for (global_iterator_3 = 0; global_iterator_3 < map_marker_counter; ++global_iterator_3)
    {
      myfile <<map_markers_dist(global_iterator_3,0) <<"\n"; 
    }

    // 3) calculate difference between each map_marker and marker, if all markers_dist will be found in map_markers_dist,
    //    then markers[0] corresponds to map_markers[global_iterator] (both are relative values), 
    //    and map_markers_dist[global_iterator_4] corresponds markers_dist[global_iterator_3]
    //    first "marker_counter" objects of map_markers_matched_cv contains corresponding objects of markers_cv
    matched_count = 0;
    for (global_iterator_3 = 0; global_iterator_3 < marker_counter; ++global_iterator_3)
    {
      for (global_iterator_4 = 0; global_iterator_4 < map_marker_counter; ++global_iterator_4)
      {
        diff = fabs(map_markers_dist(global_iterator_4,0) - markers_dist(global_iterator_3,0));

        if(diff < matching_eps )
        {
          matched_count += 1;
          map_markers_matched_cv[global_iterator_3] = map_markers_cv[global_iterator_4];
          myfile <<"matched_count: " <<matched_count<<"\n"; 
      //    myfile <<"fabs(map_marker_dist[" <<global_iterator_4<<"] "<<" - marker_dist["<<global_iterator_3<<"]) = "<< fabs(map_markers_dist(global_iterator_4) - markers_dist(global_iterator_3)) <<" \n"; 
      //    myfile <<"fabs(map_marker_dist[" <<global_iterator_4<<"] "<<" - marker_dist["<<global_iterator_3<<"]) = "<< fabs(map_markers_dist(global_iterator_4) - markers_dist(global_iterator_3)) <<" \n";  
          myfile <<"diff: " <<diff<<"\n";
          
          myfile << "map_marker_dist[" <<global_iterator_4<<"] == " << "marker_dist["<<global_iterator_3<<"]"<<"\n"; 
          myfile << map_markers_dist(global_iterator_4,0)<<" == " << markers_dist(global_iterator_3,0)<<"\n"; 
        }
        if (matched_count == marker_counter)
          return true;
      }
    }

  }

  return true;
}

// input: markers, marker_counter, map_markers
bool VelmobilGlobalLocalization::updateMapMarkers()
{
/*
    myfile <<"marker_counter: "<< marker_counter <<"\n"; 
    myfile <<"markers SIZE: "<< markers.size() <<"\n"; 
    myfile <<"map_marker_counter: "<< map_marker_counter <<"\n"; 
    myfile <<"map_markers SIZE: "<< map_markers.size() <<"\n"; 
*/
/*  tf_listener.lookupTransform("base_link", "map", ros::Time(0), tf_transform);
 
odom_transform.col(2) << tf_transform.getOrigin().x(),tf_transform.getOrigin().y(), 0;
odom_transform(0,0) = tf_transform.getBasis()[0][0];
odom_transform(1,0) = tf_transform.getBasis()[1][0];
odom_transform(0,1) = tf_transform.getBasis()[0][1];
odom_transform(1,1) = tf_transform.getBasis()[1][1];
*/
          myfile <<"Using localization transform:"<<"\n"; 
          myfile <<odom_transform<<"\n";
  for (global_iterator = 0; global_iterator < marker_counter; ++global_iterator)
  {
    markers_in_map.at(global_iterator)(0) = (odom_transform(0,0) * markers.at(global_iterator)(0) + odom_transform(0,1) * markers.at(global_iterator)(1) + odom_transform(0,2));
    markers_in_map.at(global_iterator)(1) = (odom_transform(1,0) * markers.at(global_iterator)(0) + odom_transform(1,1) * markers.at(global_iterator)(1) + odom_transform(1,2));
  }
 /* for (global_iterator = 0; global_iterator < map_marker_counter; ++global_iterator)
  {
    map_markers.at(global_iterator) = odom_transform_diff.topLeftCorner(2,2) * map_markers.at(global_iterator).head(2) + odom_transform_diff.col(2);
   // map_markers.at(global_iterator)(1) = odom_transform_diff(1,0) * map_markers.at(global_iterator)(0) + odom_transform_diff(1,1) * map_markers.at(global_iterator)(1) + odom_transform_diff(1,2);
  }
*/
  for (global_iterator = 0; global_iterator < marker_counter; global_iterator++)
  {
    for (global_iterator_2 = 0; global_iterator_2 < map_marker_counter; global_iterator_2++)
    {
      if (fabs(markers_in_map.at(global_iterator)(0) - map_markers.at(global_iterator_2)(0)) < marker_position_tresh_[0] 
          && fabs(markers_in_map.at(global_iterator)(1) - map_markers.at(global_iterator_2)(1)) < marker_position_tresh_[1])
      {
        // needed to determine if the marker was found -> global_iterator_2 greater then map_marker_counter
        myfile <<"Marker in TRESH"<<"\n"; 
        myfile <<"ODOM:"<<"\n"; 
        myfile <<markers_in_map.at(global_iterator)<<"\n"; 
        myfile <<"MAP:"<<"\n"; 
        myfile <<map_markers.at(global_iterator_2)<<"\n"; 
      myfile <<"global_iterator_2: "<< global_iterator_2 <<"\n"; 

        global_iterator_2 = global_iterator_2 + map_marker_counter + 1;
        break;
      }
    }
    // marker found in map_markers, change pose of known marker
    if (global_iterator_2 > map_marker_counter)
    {
      // get the true global_iterator_2 count
      global_iterator_2 = global_iterator_2 - map_marker_counter - 1;
      map_markers.at(global_iterator_2).head(2) = map_markers.at(global_iterator_2).head(2) + (markers_in_map.at(global_iterator).head(2) - map_markers.at(global_iterator_2).head(2))/2;
      map_markers.at(global_iterator_2)(2) = map_markers.at(global_iterator_2)(2) + 1;
      myfile <<"new MAP:"<<"\n"; 
        myfile <<map_markers.at(global_iterator_2)<<"\n"; 

      myfile <<"global_iterator_2: "<< global_iterator_2 <<"\n"; 
      myfile <<"global_iterator: "<< global_iterator<<"\n"; 
      myfile <<"map_marker_counter: "<< map_marker_counter <<"\n"; 
    }
    //marker not found in map_markers -> adding this marker to map
    else
    {
      myfile <<"Adding new marker to map"<<"\n"; 
      if (map_marker_counter > map_markers.size())
            RTT::Logger::log() << RTT::Logger::Warning << "[Global localization] -- reached max global markers count ( "<<map_markers.size()<<" )!!!! \n"<< RTT::Logger::endl;
      else
      {   
        map_markers.at(map_marker_counter).head(2) = markers_in_map.at(global_iterator);
        map_marker_counter += 1;
      }
    }
  }
  return true;
}
bool VelmobilGlobalLocalization::removeVisMarkers(visualization_msgs::Marker &vis_markers, const size_t &marker_id)
{

  for (global_iterator = 0; global_iterator < marker_id; global_iterator++)
  {
    markers.at(global_iterator) << 100,100, 0;

    vis_markers.points.at(global_iterator).x = 0;
    vis_markers.points.at(global_iterator).y = 0;
    vis_markers.points.at(global_iterator).z = 0;
    vis_markers.colors.at(global_iterator).r = 0;
    vis_markers.colors.at(global_iterator).g = 1;
    vis_markers.colors.at(global_iterator).b = 0;
    vis_markers.colors.at(global_iterator).a = 0;


  }
}

bool VelmobilGlobalLocalization::visualizationInitialization(visualization_msgs::Marker &vis_markers, const size_t &marker_id , const std::vector<Eigen::Vector3f> &positions, const Eigen::Matrix<float, 4,1> &color, const std::string &frame , const std::string &ns)
{
vis_markers.header.frame_id = frame;

vis_markers.ns = ns;
vis_markers.id = 1;
// type = POINTS
vis_markers.type = 8;
// action add object
vis_markers.action = 0;
vis_markers.lifetime = ros::Duration(5);
vis_markers.scale.x = 0.1;
vis_markers.scale.y = 0.1;
vis_markers.frame_locked = true;

    for (global_iterator = 0; global_iterator < marker_id; global_iterator++)
  {
    vis_markers.points.at(global_iterator).x = positions.at(global_iterator)(0);
    vis_markers.points.at(global_iterator).y = positions.at(global_iterator)(1);
    vis_markers.points.at(global_iterator).z = 0;
    vis_markers.colors.at(global_iterator).r = color(0);
    vis_markers.colors.at(global_iterator).g = color(1);
    vis_markers.colors.at(global_iterator).b = color(2);
    vis_markers.colors.at(global_iterator).a = color(3);

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

bool VelmobilGlobalLocalization::polarLaserToCartesianBase(const std::vector<float> &ranges, const std::vector<float> &intensities, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> &data, size_t &data_size, const Eigen::Matrix< float, 3, 3> &transform ) 
{
  myfile <<"polar to cartesian init\n"; 
  myfile <<"ranges: "<<ranges.size()<<"\n"; 
  myfile <<"angles: "<<angles.size()<<"\n"; 
  data_size = 0;
	for (global_iterator = 0; global_iterator < ranges.size() - 1; )
	{
    last_dist_check_vec << transform(0,0) * ranges.at(global_iterator) * cos(angles.at(global_iterator)) + transform(0,1) * ranges.at(global_iterator) * sin(angles.at(global_iterator)) + transform(0,2)
                        , transform(1,0) * ranges.at(global_iterator) * cos(angles.at(global_iterator)) + transform(1,1) * ranges.at(global_iterator) * sin(angles.at(global_iterator)) + transform(1,2);
    ++global_iterator;
    current_dist_check_vec << transform(0,0) * ranges.at(global_iterator) * cos(angles.at(global_iterator)) + transform(0,1) * ranges.at(global_iterator) * sin(angles.at(global_iterator)) + transform(0,2)
                        , transform(1,0) * ranges.at(global_iterator) * cos(angles.at(global_iterator)) + transform(1,1) * ranges.at(global_iterator) * sin(angles.at(global_iterator)) + transform(1,2);

    diff_dist_check_vec = last_dist_check_vec - current_dist_check_vec;                   
/*    myfile <<"norm: "<<diff_dist_check_vec.norm()<<"\n"; 
    myfile <<"tresh: "<<marker_position_tresh_[0]<<"\n";     
*/
    if (diff_dist_check_vec.norm() < 0.05)
    {
      --global_iterator;
      data.row(data_size) << last_dist_check_vec(0), last_dist_check_vec(1), intensities.at(global_iterator);
      ++data_size;
      ++global_iterator;
      data.row(data_size) << current_dist_check_vec(0), current_dist_check_vec(1), intensities.at(global_iterator);
    }
/*  
		data(global_iterator,0) = transform(0,0) * ranges.at(global_iterator) * cos(angles.at(global_iterator)) + transform(0,1) * ranges.at(global_iterator) * sin(angles.at(global_iterator)) + transform(0,2);
		data(global_iterator,1) = transform(1,0) * ranges.at(global_iterator) * cos(angles.at(global_iterator)) + transform(1,1) * ranges.at(global_iterator) * sin(angles.at(global_iterator)) + transform(1,2);
		data(global_iterator,2) = intensities.at(global_iterator);
	*/
  }

	return true;
}
bool VelmobilGlobalLocalization::configureHook() 
{


	return true;
}

bool VelmobilGlobalLocalization::startHook() 
{
  size_t current_set_markers_size = 20;
  size_t map_set_markers_size = 200;
  nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
  nsec_old = nsec;
  loop_seq = 0;
  new_data_vec_ptr->at(0) = false;
  new_data_vec_ptr->at(1) = false;
  //[x y intensity] * 540 (beams) * 2 (laser scanners)
  scan_front_data_matrix->resize(541,3);
  scan_front_size = 0;
  scan_rear_data_matrix->resize(541,3);
  scan_rear_size = 0;
  angles.resize(541);
  for (int i = 0; i< angles.size(); i++)
  {
  	angles.at(i) = M_PI/180 * (-135 + 0.5 * i);
  }
  global_iterator = 0;

  // component initial mode -- idle
  my_mode = 0; 
  // set max markers count 
  markers.resize(current_set_markers_size);
  map_markers.resize(200);
  markers_in_map.resize(current_set_markers_size);

  marker_id = 0;
  map_marker_counter = 0;
  msg_markers_ptr->points.resize(current_set_markers_size);
  msg_markers_ptr->colors.resize(current_set_markers_size);
  new_rising_edge_front = true;
  new_rising_edge_rear = true;
  rising_marker_iterator_front = 0;
  rising_marker_iterator_rear = 0;
//myfile<< "marker size: " << markers.size()<< "\n";
  myfile.open ("/tmp/gl_log_data.txt");
  odom_transform.setIdentity();
  odom_quaternion.setIdentity();
// localizacion
  map_markers_cv.resize(map_set_markers_size);
  markers_cv.resize(map_set_markers_size);

// matching
  map_markers_dist.resize(map_set_markers_size,3);
  markers_dist.resize(current_set_markers_size,3);
  matched_count = 0;
  matching_eps = 0.1;
  pointCurrentWeight = 0;
  pointCurrentMatches.resize(current_set_markers_size,1);
  pointBestWeight = 0;
  pointBestMatches.resize(current_set_markers_size,1);
  calcMarkDist_iter = 0;
  calcMarkDist_iter_2 = 0;
  calcMarkDist_iter_3 = 0;
  calcWeight_iterator = 0;
  calcWeight_iterator_2 = 0;
  diff_old.setZero();
  distance_memory.setZero();
  // match locally
  marker_transformed.setZero();
	marker_pose_dist = 0;
	map_marker_pose_dist = 0;

// LSF localization
calc_match_initialized = false;
    projection.resize(current_set_markers_size*2,1);
    M.resize(current_set_markers_size*2,4);
    // map_markers_matched_eigen -> position vector(0,1,2) + match weight(3)
    map_markers_matched_eigen.resize(current_set_markers_size,4);
    markers_eigen.resize(current_set_markers_size,3);
    map_markers_eigen.resize(map_set_markers_size,3);
    transform_eigen.setIdentity();
    transform_eigen_inverted.setIdentity();
  geometry_msgs::TransformStamped init_transform;
 localization_initialized = false;
  msg_base_map_tf_ptr->transforms.push_back(init_transform);

  msg_base_map_tf_ptr->transforms.at(0).header.seq = 0;
  msg_base_map_tf_ptr->transforms.at(0).header.stamp = *current_loop_time_ptr;
  im_map_ptr.reset(new IntensityMapHandler(map_set_markers_size));
  vis_marker_array.markers.push_back(visualization_msgs::Marker());
  vis_marker_array.markers.push_back(visualization_msgs::Marker());
  vis_marker_array.markers.at(0).points.resize(current_set_markers_size);
  vis_marker_array.markers.at(0).colors.resize(current_set_markers_size);
  vis_marker_array.markers.at(1).points.resize(current_set_markers_size);
  vis_marker_array.markers.at(1).colors.resize(current_set_markers_size);
  msg_pose_stamped_ptr->header.frame_id = "odom";
  old_odom_transform_inverted.setIdentity();
  intense_marker.points.resize(current_set_markers_size);
  msg_localization_initialized = false;
  low_cost_local_match = 0;


  marker_counter_check_matched_markers = 0;
  best_matched_map = 0;
  partial_cost.resize(current_set_markers_size, 1);
  matched_partial_cost.resize(current_set_markers_size, current_set_markers_size);
  matched_map_markers.resize(map_set_markers_size,1);
  matched_cost_thresh = 0.75;

  return true;
}

bool VelmobilGlobalLocalization::getScanData()
{


  // predict bias and calculate theta
  if (RTT::NewData == in_laser_front_.read((*msg_laser_front_ptr)))
  {

    scan_front_data_matrix->setZero();
    laser_in_base_transform << 1, 0, 0.3,
                               0, 1, 0,
                               0, 0, 1;

    laser_in_odom_transform = laser_in_base_transform ;
    //polarLaserToCartesianBase(msg_laser_front_ptr->ranges, msg_laser_front_ptr->intensities, (*scan_front_data_matrix), laser_in_base_transform);
    
    polarLaserToCartesianBase(msg_laser_front_ptr->ranges, msg_laser_front_ptr->intensities, (*scan_front_data_matrix), scan_front_size, laser_in_odom_transform);
    new_data_vec_ptr->at(0) = true;
  }

  if (RTT::NewData == in_laser_rear_.read((*msg_laser_rear_ptr)))
  {

    scan_rear_data_matrix->setZero();
    laser_in_base_transform << -1, 0, -0.3,
                               0, -1, 0,
                               0, 0, 1;
    laser_in_odom_transform = laser_in_base_transform;

    //polarLaserToCartesianBase(msg_laser_rear_ptr->ranges, msg_laser_rear_ptr->intensities, (*scan_rear_data_matrix), laser_in_base_transform);
    polarLaserToCartesianBase(msg_laser_rear_ptr->ranges, msg_laser_rear_ptr->intensities, (*scan_rear_data_matrix), scan_rear_size, laser_in_odom_transform);
    new_data_vec_ptr->at(1) = true;
  }
}

bool VelmobilGlobalLocalization::findMarkers(Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> &scan_data_matrix, size_t &scan_data_size)
{
  for (global_iterator = 0; global_iterator < scan_data_size; global_iterator++ )
  {
    // [FRONT LASER] found rising edge
    if (scan_data_matrix(global_iterator,2)  > min_intensity_ && new_rising_edge_front)
    {
      myfile << "RISING\n";

      //calculate distance between last rising edge and cutten 
      rising_marker_iterator_front = global_iterator;
      new_rising_edge_front = false;
    }
    // found sloping edge
    if ((scan_data_matrix(global_iterator,2) < min_intensity_ || global_iterator == (scan_data_size - 1)) && !new_rising_edge_front)
    {
      myfile << "SLOPING\n";
      new_rising_edge_front = true;
      marker_id = rising_marker_iterator_front + floor((global_iterator-rising_marker_iterator_front)/2);

      // check if both lasers found the same marker. If so, change marker pose to average
      global_iterator_2 = 0;
      for (global_iterator_2 = 0; global_iterator_2 < marker_counter; global_iterator_2++)
      {
        if (fabs(markers.at(global_iterator_2)(0) - scan_data_matrix(marker_id,0)) < marker_position_tresh_[0] 
          && fabs(markers.at(global_iterator_2)(1) - scan_data_matrix(marker_id,1)) < marker_position_tresh_[1])
        {
          myfile << "Both lasers found the same marker\n";
          myfile << "rear marker:\n";
          myfile << markers.at(global_iterator_2)(0) << "\n";
          myfile << markers.at(global_iterator_2)(1) << "\n";
          myfile << "front marker:\n";
          myfile << scan_data_matrix(marker_id,0) << "\n";
          myfile << scan_data_matrix(marker_id,1) << "\n";

          markers.at(global_iterator_2)(0) = markers.at(global_iterator_2)(0) + (scan_data_matrix(marker_id,0) - markers.at(global_iterator_2)(0))/2;
          markers.at(global_iterator_2)(1) = markers.at(global_iterator_2)(1) + (scan_data_matrix(marker_id,1) - markers.at(global_iterator_2)(1))/2;
          markers.at(global_iterator_2)(2) = markers.at(global_iterator_2)(2) + 1;

          myfile << "saved marker:\n";
          myfile << markers.at(global_iterator_2)(0) << "\n";
          myfile << markers.at(global_iterator_2)(1) << "\n";

          global_iterator_2 = marker_counter + 1;
          break;
        }
      }
      // add new marker if front laser didn't find one in nearby
      if (global_iterator_2 <= marker_counter )
      {
        markers.at(marker_counter) << scan_data_matrix(marker_id,0),scan_data_matrix(marker_id,1), 1;
        myfile << "marker_id: " << marker_id<<"\n";
        myfile << "marker_intensity: " << scan_data_matrix(marker_id,2)<<"\n";
        myfile << "marker_counter: " << marker_counter<<"\n";
        myfile << "markers: \n"<< markers.at(marker_counter)<<"\n";
        marker_counter += 1;          
      }
    }

  }
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

  //removeVisMarkers( (*msg_markers_ptr), marker_counter);
  //out_markers_.write((*msg_markers_ptr));
  msg_pose_stamped_ptr->header.stamp = *current_loop_time_ptr;
  marker_counter = 0;

  ////
  // load map if requested
  ////
  if (RTT::NewData == in_load_map_.read((*msg_load_map_ptr)))
  {
      myfile <<"load map \n"; 
      im_map_ptr->load_map((*msg_load_map_ptr), map_markers, map_marker_counter);
      myfile << "-- LOADED markers: ---\n";
    for (global_iterator_2 = 0; global_iterator_2 < map_marker_counter; ++global_iterator_2)
    {
 	        myfile <<"-----   map markers eigen   ------\n"; 
   	      map_markers_eigen.row(global_iterator_2) << map_markers.at(global_iterator_2)(0), map_markers.at(global_iterator_2)(1), 1;
          myfile << map_markers_eigen(global_iterator_2)<<"\n";

    }
  }
  ////
  // check if localization set to initialized
  ////
  if (RTT::NewData == in_localization_initialized_.read(msg_localization_initialized))
  {
      myfile <<"localization initialized \n"; 
      localization_initialized = msg_localization_initialized;
  }
  ////
  // check component mode
  ////
  // state: 0 - idle
  // state: 1 - mapping
  // state: 2 - localization
  if (RTT::NewData == in_change_mode_.read((*msg_change_mode_ptr)))
  {
    myfile <<"NEW MODE \n"; 
    my_mode = (*msg_change_mode_ptr);
  }
  if (my_mode == 1)
  {
    myfile <<"USE mapping mode \n"; 
    getScanData();
    if (new_data_vec_ptr->at(0) && new_data_vec_ptr->at(1))
    {

        findMarkers((*scan_front_data_matrix), scan_front_size);
        findMarkers((*scan_rear_data_matrix), scan_rear_size);
        myfile << "-- initialization ---\n";

        new_data_vec_ptr->at(0) = false;
        new_data_vec_ptr->at(1) = false;
      

      myfile << "-- show-markers ---\n";
      for (global_iterator_2 = 0; global_iterator_2 < marker_counter; ++global_iterator_2)
      {
            myfile << markers.at(global_iterator_2)<<"\n";

            intense_marker.points.at(global_iterator_2).x = markers.at(global_iterator_2)(0);
            intense_marker.points.at(global_iterator_2).y = markers.at(global_iterator_2)(1);
            intense_marker.points.at(global_iterator_2).z = 0;
      }
      intense_marker.header.stamp = *current_loop_time_ptr;
      intense_marker.header.frame_id = "base_link";
    intense_marker.marker_count = marker_counter;
    out_intense_markers_.write(intense_marker);
    }
 /*     localizeLSF();

      updateMapMarkers();


      myfile << "-- show-map-markers ---\n";
      for (global_iterator_2 = 0; global_iterator_2 < map_marker_counter; ++global_iterator_2)
      {
            myfile << map_markers.at(global_iterator_2)<<"\n";
      }
        vis_color << 0,1,0,1;
        vis_frame = "base_link";
        vis_namespace = "localization_current";
        visualizationInitialization( vis_marker_array.markers.at(0), marker_counter, markers, vis_color, vis_frame, vis_namespace);
        //vis_marker_array.markers.at(0) = (*msg_markers_ptr);

        vis_color << 1,0,0,1;
        vis_frame = "odom";
        vis_namespace = "localization_map";
        visualizationInitialization( vis_marker_array.markers.at(1), map_marker_counter, map_markers, vis_color, vis_frame, vis_namespace);
        //vis_marker_array.markers.at(1) = (*msg_markers_ptr);

        out_markers_.write(vis_marker_array);

    }
    msg_base_map_tf_ptr->transforms.at(0).header.frame_id = "map_2";
    msg_base_map_tf_ptr->transforms.at(0).child_frame_id = "odom";

    msg_base_map_tf_ptr->transforms.at(0).header.seq = loop_seq;
    msg_base_map_tf_ptr->transforms.at(0).header.stamp = *current_loop_time_ptr;

    msg_base_map_tf_ptr->transforms.at(0).transform.translation.x = odom_transform_diff(0,2);
    msg_base_map_tf_ptr->transforms.at(0).transform.translation.y = odom_transform_diff(1,2);
    msg_base_map_tf_ptr->transforms.at(0).transform.translation.z = 0;
// acos(odom_transform_diff_inverted(0,0))
    transf_orient_quat_.setRPY(0, 0, atan2(odom_transform_diff(1,0),odom_transform_diff(0,0)));
    msg_base_map_tf_ptr->transforms.at(0).transform.rotation.x = transf_orient_quat_.getX();
    msg_base_map_tf_ptr->transforms.at(0).transform.rotation.y = transf_orient_quat_.getY();
    msg_base_map_tf_ptr->transforms.at(0).transform.rotation.z = transf_orient_quat_.getZ();
    msg_base_map_tf_ptr->transforms.at(0).transform.rotation.w = transf_orient_quat_.getW();

    out_transform_.write((*msg_base_map_tf_ptr));
  */  
  }
  else  if (my_mode == 2)
  {
    myfile <<"USE LOCALIZE mode \n"; 
    getScanData();
    if (new_data_vec_ptr->at(0) && new_data_vec_ptr->at(1))
    {
      findMarkers((*scan_front_data_matrix), scan_front_size);
      findMarkers((*scan_rear_data_matrix), scan_rear_size);
      myfile << "-- initialization ---\n";
      vis_color << 0,1,0,1;
      vis_frame = "base_link";
      vis_namespace = "localization_current";
      myfile << "vis current \n";
      visualizationInitialization( vis_marker_array.markers.at(0), marker_counter, markers, vis_color, vis_frame, vis_namespace);
      //vis_marker_array.markers.at(0) = (*msg_markers_ptr);

      vis_color << 1,0,0,1;
      vis_frame = "map";
      vis_namespace = "localization_map";
      myfile << "vis map \n";
      visualizationInitialization( vis_marker_array.markers.at(1), map_marker_counter, map_markers, vis_color, vis_frame, vis_namespace);
      //vis_marker_array.markers.at(1) = (*msg_markers_ptr);

      out_markers_.write(vis_marker_array);
      new_data_vec_ptr->at(0) = false;
      new_data_vec_ptr->at(1) = false;

    out_markers_.write(vis_marker_array);

      if (localizeLSF())
      {
            myfile << "odom:\n";
          myfile << odom_transform<<std::endl;
          myfile << "old_odom_transform_inverted:\n";
          myfile << old_odom_transform_inverted<<std::endl;
          odom_transform_diff =   transform_eigen_inverted * old_odom_transform_inverted;
          myfile << "odom_transform_diff:\n";
          myfile << odom_transform_diff<<std::endl;
      }
            myfile << "prepare transform \n";
        
            msg_base_map_tf_ptr->transforms.at(0).header.frame_id = "map";
            msg_base_map_tf_ptr->transforms.at(0).child_frame_id = "odom";
        
            msg_base_map_tf_ptr->transforms.at(0).header.seq = loop_seq;
            msg_base_map_tf_ptr->transforms.at(0).header.stamp = *current_loop_time_ptr;
        
            msg_base_map_tf_ptr->transforms.at(0).transform.translation.x = odom_transform_diff(0,2);
            msg_base_map_tf_ptr->transforms.at(0).transform.translation.y = odom_transform_diff(1,2);
            msg_base_map_tf_ptr->transforms.at(0).transform.translation.z = 0;
        // acos(odom_transform_diff_inverted(0,0))
            transf_orient_quat_.setRPY(0, 0, atan2(odom_transform_diff(1,0),odom_transform_diff(0,0)));
            msg_base_map_tf_ptr->transforms.at(0).transform.rotation.x = transf_orient_quat_.getX();
            msg_base_map_tf_ptr->transforms.at(0).transform.rotation.y = transf_orient_quat_.getY();
            msg_base_map_tf_ptr->transforms.at(0).transform.rotation.z = transf_orient_quat_.getZ();
            msg_base_map_tf_ptr->transforms.at(0).transform.rotation.w = transf_orient_quat_.getW();
              myfile << "send transform \n";
        
            out_transform_.write((*msg_base_map_tf_ptr));
              myfile << "transform sent\n";
    }

  }else
  {
    return;
  }
  ////
  // save current state of map_markers if requested
  ////
  if (RTT::NewData == in_save_map_.read((*msg_save_map_ptr)))
  {
    myfile <<"save map \n"; 
    im_map_ptr->save_map((*msg_save_map_ptr), map_markers, map_marker_counter);
  }
  // 
  // // // ODOM 
  //
  if (RTT::NewData == in_odom_transform_.read((*msg_odom_transform_ptr)))
  {
    myfile <<"UPDATING ODOM \n"; 
    old_odom_transform_inverted.topLeftCorner(2,2) = odom_transform.topLeftCorner(2,2).transpose();
    old_odom_transform_inverted.col(2) << -(old_odom_transform_inverted(0,0)*odom_translation(0) + old_odom_transform_inverted(0,1)*odom_translation(1)), 
                              -(old_odom_transform_inverted(1,0)*odom_translation(0) + old_odom_transform_inverted(1,1)*odom_translation(1)),
                              1;

    odom_quaternion.x() =  msg_odom_transform_ptr->transforms.at(0).transform.rotation.x;
    odom_quaternion.y() =  msg_odom_transform_ptr->transforms.at(0).transform.rotation.y;
    odom_quaternion.z() =  msg_odom_transform_ptr->transforms.at(0).transform.rotation.z;
    odom_quaternion.w() =  msg_odom_transform_ptr->transforms.at(0).transform.rotation.w;

    odom_translation(0) =  msg_odom_transform_ptr->transforms.at(0).transform.translation.x;
    odom_translation(1) =  msg_odom_transform_ptr->transforms.at(0).transform.translation.y;

    odom_transform.topLeftCorner(2,2) = odom_quaternion.toRotationMatrix().topLeftCorner(2,2);
    odom_transform.col(2) << odom_translation(0), 
                              odom_translation(1),
                              1;

  }

  loop_seq += 1;
  myfile <<" <<<<  NEXT LOOP \n"; 
}

ORO_CREATE_COMPONENT(VelmobilGlobalLocalization)
