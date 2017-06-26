#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <rtt_rosparam/rosparam.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include "VelmWheel_laser_driver.h"

#define DEG2RAD M_PI/180.0


VelmWheelLaserDriver::VelmWheelLaserDriver(const std::string& name) : TaskContext(name)
{

	this->addPort("out_laser",out_laser_);
	this->addProperty("/VELMWHEEL_OROCOS_ROBOT/VelmWheel_laser_driver/merged_scan_frame",merged_scan_frame_);
	this->addProperty("/VELMWHEEL_OROCOS_ROBOT/VelmWheel_laser_driver/hosts",hosts_);
	this->addProperty("/VELMWHEEL_OROCOS_ROBOT/VelmWheel_laser_driver/useNTP",useNTP_);
	this->addProperty("laser_config",laser_config_);

	laserF_to_base_ptr.reset(new Eigen::Transform<double,2,Eigen::Affine>);
	msg_laserF_ptr.reset(new sensor_msgs::LaserScan);

	msg_laserOut_ptr.reset(new sensor_msgs::LaserScan);
	 LMS1xx laser_F;
  scanCfg cfg;
  NTPcfg cfg_ntp;
  scanOutputRange outputRange;
  scanDataCfg dataCfg;

  std::chrono::nanoseconds nsec_rest;
int sequence_enc;
bool connection_status;
    scanData data;

}

VelmWheelLaserDriver::~VelmWheelLaserDriver() 
{
    laser_F.scanContinous(0);
    laser_F.stopMeas();
    laser_F.disconnect();
}
bool VelmWheelLaserDriver::checkLaserConnection(LMS1xx *laser, std::string &host)
{
	std::cout<< "start connect" <<std::endl;
	laser->connect(host);
	std::cout<< "after connect" <<std::endl;

	if (!laser->isConnected())
	{
		std::cout<< "not connected" <<std::endl;

	    RTT::Logger::log() << RTT::Logger::Debug << "[Laser driver] -- unable to connect: \n"<<host<< RTT::Logger::endl;
		return false;
	}
		std::cout<< " connected" <<std::endl;

	RTT::Logger::log() << RTT::Logger::Debug << "[Laser driver] -- connected to: \n"<<host<< RTT::Logger::endl;
	return true;
}

void VelmWheelLaserDriver::configLaser(LMS1xx *laser, std::string &host)
{
	laser->login();
	cfg = laser->getScanCfg();
	while(cfg.scaningFrequency != 5000)
	{
	    laser->disconnect();
	    RTT::Logger::log() << RTT::Logger::Debug << "Unable to get laser output range: \n"<<host<< RTT::Logger::endl;
		sleep(1);
		laser->login();
		cfg = laser->getScanCfg();
	}
	if (useNTP_)
	{
      RTT::Logger::log() << RTT::Logger::Debug << "Setting NTP configuration."<< RTT::Logger::endl;
      // get NTP params from ROS
      std::string IPstring;
      cfg_ntp.NTProle= 1;
      cfg_ntp.timeSyncIfc= 0;
      IPstring = "192.168.0.1";
      char * dup = strdup(IPstring.c_str());
      char * token = strtok(dup, ".");
      for(int i = 0; i < 4; i++){
        cfg_ntp.serverIP[i] = std::atoi( token );
          token = strtok(NULL, ".");
      }
      free(dup);
      cfg_ntp.timeZone =1;
      cfg_ntp.updateTime =10;
      RTT::Logger::log() << RTT::Logger::Debug << "Setting NTP configuration."<< RTT::Logger::endl;
      laser->setNTPsettings(cfg_ntp);  
    }


   	dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;
    dataCfg.timestamp = true;

    RTT::Logger::log() << RTT::Logger::Debug << "Setting scan data configuration."<< RTT::Logger::endl;
    RTT::Logger::log() << RTT::Logger::Debug << "dataCfg.remission: "<< dataCfg.remission<< RTT::Logger::endl;
    laser->setScanDataCfg(dataCfg);

    RTT::Logger::log() << RTT::Logger::Debug << "Starting measurements."<< RTT::Logger::endl;

    laser->startMeas();

    status_t stat = laser->queryStatus();
    sleep(1);
    if (stat != ready_for_measurement)
    {
      RTT::Logger::log() << RTT::Logger::Debug << "Laser not ready. Retrying initialization."<< RTT::Logger::endl;
      sleep(1);
    }

    outputRange = laser->getScanOutputRange();
	RTT::Logger::log() << RTT::Logger::Debug << "Got laser output range: \n"<<host<< RTT::Logger::endl;
	// Log laser configuration
    RTT::Logger::log() << RTT::Logger::Debug << "HOST: \n"<<host<<"\n      Laser configuration: scaningFrequency " << 
    					cfg.scaningFrequency << ", angleResolution " << cfg.angleResolution << ", startAngle " << 
    					cfg.startAngle << ", stopAngle " << cfg.stopAngle << RTT::Logger::endl;
    // Log output range Debug
    RTT::Logger::log() << RTT::Logger::Debug << "Laser output range: angleResolution " <<  outputRange.angleResolution << 
    					", startAngle " <<  outputRange.startAngle << ", stopAngle " <<  outputRange.stopAngle << RTT::Logger::endl;

    RTT::Logger::log() << RTT::Logger::Debug << "Starting device."<< RTT::Logger::endl;
    laser->startDevice(); // Log out to properly re-enable system after config

    RTT::Logger::log() << RTT::Logger::Debug << "Commanding continuous measurements."<< RTT::Logger::endl;
    laser->scanContinous(1);

	int angle_range = outputRange.stopAngle - outputRange.startAngle;
    int num_values = angle_range / outputRange.angleResolution ;
    if (angle_range % outputRange.angleResolution == 0)
    {
      // Include endpoint
      ++num_values;
    }
    RTT::Logger::log() << RTT::Logger::Debug << "num_values "<< num_values<<RTT::Logger::endl;

    msg_laserOut_ptr->ranges.resize(num_values);
        RTT::Logger::log() << RTT::Logger::Debug << "SIZE OF RANGES "<<msg_laserOut_ptr->ranges.size()<<RTT::Logger::endl;

    msg_laserOut_ptr->intensities.resize(num_values);
}
bool VelmWheelLaserDriver::configureHook() 
{
	host_F_ = hosts_.at(laser_config_);
std::cout<< "host param" << host_F_ <<std::endl;
	nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
	connection_status = false;
	while (!connection_status)
	{	
		connection_status = checkLaserConnection(&laser_F, host_F_);

	      RTT::Logger::log() << RTT::Logger::Debug << "[Laser driver] -- retying in 1 sec"<< RTT::Logger::endl;
	      sleep(1);
	}
	std::cout<< "after while" <<std::endl;


	configLaser(&laser_F, host_F_);

   	

    msg_laserOut_ptr->header.frame_id = merged_scan_frame_.at(laser_config_);
    msg_laserOut_ptr->range_min = 0.01;
    msg_laserOut_ptr->range_max = 20.0;
    msg_laserOut_ptr->scan_time = 100.0 / cfg.scaningFrequency;
    msg_laserOut_ptr->angle_increment = (double)outputRange.angleResolution / 10000.0 * DEG2RAD;
    msg_laserOut_ptr->angle_min = (double)outputRange.startAngle / 10000.0 * DEG2RAD - M_PI / 2;
    msg_laserOut_ptr->angle_max = (double)outputRange.stopAngle / 10000.0 * DEG2RAD - M_PI / 2;


	return true;
}

bool VelmWheelLaserDriver::startHook() 
{

	return true;
}


void VelmWheelLaserDriver::setHeader(std_msgs::Header &header, const std::string &frame, const uint32_t &seq_id)
{

}


////
// UPDATE
////
void VelmWheelLaserDriver::updateHook() 
{
	nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
	++sequence_enc;

	msg_laserOut_ptr->header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(nsec).count();

	nsec_rest = nsec - std::chrono::duration_cast<std::chrono::seconds>(nsec);
	msg_laserOut_ptr->header.stamp.nsec = nsec_rest.count();;
	msg_laserOut_ptr->header.seq = sequence_enc;


//	setHeader(msg_laserOut_ptr->header, msg_laserOut_ptr->header.frame_id, sequence_enc);

    RTT::Logger::log() << RTT::Logger::Debug << "Reading scan data. "<< getName() <<""<< RTT::Logger::endl;

    if (laser_F.getScanData(&data))
    {
     // RTT::Logger::log() << RTT::Logger::Debug << "dist_len1: "<< data.dist_len1<< RTT::Logger::endl;
         RTT::Logger::log() << RTT::Logger::Debug << " RANGES SIZE -- "<< getName() <<" "<< msg_laserOut_ptr->ranges.size()<<RTT::Logger::endl;
   	
        for (int i = 0; i < data.dist_len1; i++)
        {
        	          // std::cout<<"i: "<<i<<std::endl;
        	           //std::cout<<""<< getName() <<" -- data.dist1["<<i<<"]: "<< data.dist1[i]<<std::endl;
        	          // std::cout<<"ranges["<<i<<"]: "<< msg_laserOut_ptr->ranges[i]<<std::endl;

          msg_laserOut_ptr->ranges[i] = (data.dist1[i] * 0.001);
        }
     //RTT::Logger::log() << RTT::Logger::Debug <<" "<< getName() <<": rssi_len1: "<< data.rssi_len1<< RTT::Logger::endl;

        for (int i = 0; i < data.rssi_len1; i++)
        {
          msg_laserOut_ptr->intensities[i] = data.rssi1[i];
        }

        // check if useNTP flag is set true
        if (useNTP_)
        {
          RTT::Logger::log() << RTT::Logger::Debug << "Publishing NTP time stamp."<< RTT::Logger::endl;
          msg_laserOut_ptr->header.stamp.sec = data.msg_sec;
          msg_laserOut_ptr->header.stamp.nsec = data.msg_usec*1000;
        }
		out_laser_.write((*msg_laserOut_ptr));

    }
    else
    {
      RTT::Logger::log() << RTT::Logger::Debug << "Laser timed out on delivering scan"<< RTT::Logger::endl;
    }    


}



ORO_CREATE_COMPONENT(VelmWheelLaserDriver)
