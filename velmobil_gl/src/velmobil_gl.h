
#ifndef VELMOBILGLOBALLOCALIZATION_H_
#define VELMOBILGLOBALLOCALIZATION_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

//openCV
#include "opencv2/opencv.hpp" 
class VelmobilGlobalLocalization : public RTT::TaskContext
{
	public:
		explicit VelmobilGlobalLocalization(const std::string& name);
		~VelmobilGlobalLocalization();
		void reset();
	private:
		bool configureHook();
		void updateHook();
		bool startHook();
		bool removeVisMarkers(visualization_msgs::Marker &markers,const size_t &marker_id);
		bool visualizationInitialization(visualization_msgs::Marker &vis_markers, const size_t &marker_id , const std::vector<Eigen::Vector3f> &positions, const Eigen::Matrix<float, 4,1> &color, const std::string &frame, const std::string & ns);
		bool polarLaserToCartesianBase(const std::vector<float> &ranges, const std::vector<float> &intensities, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> &data, size_t &data_size, const Eigen::Matrix< float, 3, 3> &transform );
		bool updateMapMarkers();
		bool localizeEIGEN();
		bool calcMarkDistEIGEN(const Eigen::Matrix<float,Eigen::Dynamic,3> &input_markers, const int &marker_size, const int &respect_marker, Eigen::Matrix<float,Eigen::Dynamic,3> &distances);
		bool calcMatchWeight( float  &matchWeight, Eigen::Matrix<float,Eigen::Dynamic,1> &match_ids);

		bool localizeUmeyama();
		bool localizeLSF();
		bool calcMarkDistEIGEN(const Eigen::Matrix<float,Eigen::Dynamic,3> &input_markers, const int &marker_size, const int &respect_marker, std::vector<float> &distances, size_t &my_iterator);
		bool matchMarkersEIGEN(); 

		bool localizeCV();
		bool calcMarkDistCV(const std::vector<cv::Point2f> &input_markers, const int &marker_size, const int &respect_marker, Eigen::Matrix<float,Eigen::Dynamic,3> &distances, size_t &my_iterator);
		bool matchMarkersCV();

		bool getScanData();
		bool findMarkers(Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> &scan_data_matrix, size_t &scan_data_size);

 		RTT::InputPort<tf2_msgs::TFMessage> in_odom_transform_;
 		RTT::InputPort<sensor_msgs::LaserScan> in_laser_front_;
 		RTT::InputPort<sensor_msgs::LaserScan> in_laser_rear_;
 		RTT::InputPort<std::string> in_save_map_;
 		RTT::InputPort<std::string> in_load_map_;
 		RTT::InputPort<int> in_change_mode_;
 		//RTT::InputPort<geometry_msgs::PoseWithCovarianceStamped> in_laser_;
 		RTT::OutputPort<visualization_msgs::MarkerArray> out_markers_;
 		RTT::OutputPort<tf2_msgs::TFMessage> out_transform_;

  		int min_intensity_;

	  	std::vector<float> marker_position_tresh_;

};
#endif  // VELMOBILGlobalLocalization_H_
