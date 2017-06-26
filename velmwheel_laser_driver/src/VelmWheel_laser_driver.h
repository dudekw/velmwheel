#ifndef VELMWHEELLASERDRIVER_H_
#define VELMWHEELLASERDRIVER_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <lms1xx/LMS1xx.h>

class VelmWheelLaserDriver : public RTT::TaskContext
{
	public:
		explicit VelmWheelLaserDriver(const std::string& name);
		~VelmWheelLaserDriver();
		void reset();
	private:
		bool configureHook();
		void updateHook();
		bool startHook();
		void configLaser(LMS1xx *laser, std::string &host);
		bool checkLaserConnection(LMS1xx *laser, std::string &host);
		void setHeader(std_msgs::Header &header, const std::string &frame, const uint32_t &seq_id);

		std::vector<std::string> merged_scan_frame_;
		std::vector<std::string> hosts_;
		std::string host_F_;
 		RTT::OutputPort<sensor_msgs::LaserScan> out_laser_;
		int laser_config_;


		std::auto_ptr<Eigen::Transform<double,2,Eigen::Affine>> laserF_to_base_ptr;

std::auto_ptr<sensor_msgs::LaserScan> msg_laserF_ptr;
std::auto_ptr<sensor_msgs::LaserScan> msg_laserOut_ptr;
 LMS1xx laser_F;
  scanCfg cfg;
  NTPcfg cfg_ntp;
  scanOutputRange outputRange;
  scanDataCfg dataCfg;
std::chrono::nanoseconds nsec;
std::chrono::nanoseconds nsec_rest;
int sequence_enc;
bool useNTP_;
bool connection_status;
    scanData data;


};
#endif  // VELMWHEELLASERDRIVER_H_
