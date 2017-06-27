
#ifndef VELMOBILGLOBALLOCALIZATION_H_
#define VELMOBILGLOBALLOCALIZATION_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

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
		bool removeMarkers(visualization_msgs::Marker &markers,const size_t &marker_id);
		bool markersInitialization(visualization_msgs::Marker &markers, const size_t &marker_id , const std::vector<Eigen::Vector2f> &positions);
		bool polarLaserToCartesianBase(const std::vector<float> &ranges, const std::vector<float> &intensities, Eigen::Matrix<float, Eigen::Dynamic , Eigen::Dynamic> &data, const Eigen::Matrix< float, 3, 3> &transform );



 		RTT::InputPort<sensor_msgs::LaserScan> in_laser_front_;
 		RTT::InputPort<sensor_msgs::LaserScan> in_laser_rear_;
 		//RTT::InputPort<geometry_msgs::PoseWithCovarianceStamped> in_laser_;
 		RTT::OutputPort<visualization_msgs::Marker> out_markers_;

  		int min_intensity_;
};
#endif  // VELMOBILGlobalLocalization_H_
