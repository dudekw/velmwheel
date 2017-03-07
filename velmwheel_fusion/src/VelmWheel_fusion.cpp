#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/LinearMath/Quaternion.h>
#include <chrono>
#include "tf/transform_datatypes.h"

#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussianmobile.h"
#include "mobile_robot_wall_cts.h"

#include "VelmWheel_fusion.h"
#include <iostream>

using namespace MatrixWrapper;
using namespace BFL;

static ExtendedKalmanFilter *filter = NULL;
static AnalyticSystemModelGaussianUncertainty *sys_model = NULL;
static LinearAnalyticMeasurementModelGaussianUncertainty *meas_model=NULL;
static geometry_msgs::Twist *msg_twist = NULL;
static nav_msgs::Odometry *msg_odometry = NULL;
Pdf<ColumnVector> *posterior;
static  ColumnVector *ekf_measurement=NULL;
  tf::Quaternion tf_quaternion;
static  ColumnVector *input=NULL;
static ColumnVector *prior_Mu = NULL;

static SymmetricMatrix *prior_Cov = NULL;
static ColumnVector *meas_noise_Mu = NULL;
static SymmetricMatrix *meas_noise_Cov = NULL;
static Gaussian *measurement_Uncertainty = NULL;
static Matrix *H = NULL;
static ColumnVector *sys_noise_Mu = NULL;
static SymmetricMatrix *sys_noise_Cov = NULL;
static Gaussian *system_Uncertainty = NULL;
static NonLinearAnalyticConditionalGaussianMobile *sys_pdf = NULL;
static LinearAnalyticConditionalGaussian *meas_pdf = NULL;
static Gaussian *prior_cont = NULL;
  uint32_t loop_time;
  std::chrono::nanoseconds nsec;
  std::chrono::nanoseconds nsec_old;
  double pose_x_from_input;

VelmWheelFusion::VelmWheelFusion(const std::string& name) : TaskContext(name)
{

	this->addPort("in_twist",in_twist_);
	this->addPort("in_odometry",in_odometry_);

	this->addPort("out_odometry",out_odometry_);

  msg_twist = new geometry_msgs::Twist();
  msg_odometry = new nav_msgs::Odometry();

  /****************************
   * NonLinear system model      *
   ***************************/
  // create gaussian
  sys_noise_Mu = new ColumnVector(STATE_SIZE);
  (*sys_noise_Mu)(1) = MU_SYSTEM_NOISE_X;
  (*sys_noise_Mu)(2) = MU_SYSTEM_NOISE_Y;
  (*sys_noise_Mu)(3) = MU_SYSTEM_NOISE_THETA;
  sys_noise_Cov = new SymmetricMatrix(STATE_SIZE);
  *sys_noise_Cov = 0.0;
  (*sys_noise_Cov)(1,1) = SIGMA_SYSTEM_NOISE_X;
  (*sys_noise_Cov)(1,2) = 0.0;
  (*sys_noise_Cov)(1,3) = 0.0;
  (*sys_noise_Cov)(2,1) = 0.0;
  (*sys_noise_Cov)(2,2) = SIGMA_SYSTEM_NOISE_Y;
  (*sys_noise_Cov)(2,3) = 0.0;
  (*sys_noise_Cov)(3,1) = 0.0;
  (*sys_noise_Cov)(3,2) = 0.0;
  (*sys_noise_Cov)(3,3) = SIGMA_SYSTEM_NOISE_THETA;

  system_Uncertainty = new Gaussian(*sys_noise_Mu, *sys_noise_Cov);
  // create the model
  sys_pdf = new NonLinearAnalyticConditionalGaussianMobile(*system_Uncertainty);
  sys_model = new AnalyticSystemModelGaussianUncertainty(sys_pdf);

  /*********************************
   * Initialise measurement model *
   ********************************/

  // create matrix H for linear measurement model
  //double wall_ct = 2/(sqrt(pow(RICO_WALL,2.0) + 1));
  H = new Matrix(MEAS_SIZE,STATE_SIZE);
  *H = 0.0;
  // odom position X
  (*H)(1,1) = 1;
  (*H)(1,2) = 0.0;
  (*H)(1,3) = 0.0;
  // odom position Y
  (*H)(2,1) = 0.0;
  (*H)(2,2) = 1;
  (*H)(2,3) = 0.0;
  // odom angular position
  (*H)(3,1) = 0.0;
  (*H)(3,2) = 0.0;
  (*H)(3,3) = 1;
  // Construct the measurement noise (a scalar in this case)
meas_noise_Mu = new   ColumnVector(MEAS_SIZE);
  (*meas_noise_Mu)(1) = MU_MEAS_NOISE;
  (*meas_noise_Mu)(2) = MU_MEAS_NOISE;
  (*meas_noise_Mu)(3) = MU_MEAS_NOISE;

 meas_noise_Cov = new SymmetricMatrix(MEAS_SIZE);
  (*meas_noise_Cov)(1,1) = SIGMA_MEAS_NOISE;
  (*meas_noise_Cov)(2,2) = SIGMA_MEAS_NOISE;
  (*meas_noise_Cov)(3,3) = SIGMA_MEAS_NOISE;
   measurement_Uncertainty = new Gaussian(*meas_noise_Mu, *meas_noise_Cov);

  // create the measurement model
  meas_pdf= new LinearAnalyticConditionalGaussian(*H, *measurement_Uncertainty);
  meas_model = new LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf);
   /****************************
   * Linear prior DENSITY     *
   ***************************/
   // Continuous Gaussian prior (for Kalman filters)
  prior_Mu = new ColumnVector(STATE_SIZE);
  (*prior_Mu)(1) = PRIOR_MU_X;
  (*prior_Mu)(2) = PRIOR_MU_Y;
  (*prior_Mu)(3) = PRIOR_MU_THETA;
  prior_Cov = new SymmetricMatrix(STATE_SIZE);
  (*prior_Cov)(1,1) = PRIOR_COV_X;
  (*prior_Cov)(1,2) = 0.0;
  (*prior_Cov)(1,3) = 0.0;
  (*prior_Cov)(2,1) = 0.0;
  (*prior_Cov)(2,2) = PRIOR_COV_Y;
  (*prior_Cov)(2,3) = 0.0;
  (*prior_Cov)(3,1) = 0.0;
  (*prior_Cov)(3,2) = 0.0;
  (*prior_Cov)(3,3) = PRIOR_COV_THETA;
  prior_cont = new Gaussian(*prior_Mu, *prior_Cov);
  /******************************
   * Construction of the Filter *
   ******************************/
  filter = new ExtendedKalmanFilter(prior_cont);

Pdf<ColumnVector> *posterior;
  ekf_measurement = new ColumnVector(3);
  tf::Quaternion tf_quaternion();
  input = new ColumnVector(3);

  uint32_t loop_time;
  double pose_x_from_input;
  std::chrono::nanoseconds nsec;
  std::chrono::nanoseconds nsec_old;
}

VelmWheelFusion::~VelmWheelFusion() 
{
delete filter;
delete meas_model;
delete sys_model;
delete msg_twist;
delete msg_odometry;
}

bool VelmWheelFusion::configureHook() 
{


	return true;
}

bool VelmWheelFusion::startHook() 
{

    (*input)(1) = 0.0;
    // velocity Y input to the robot model
    (*input)(2) = 0.0;
    // angular velocity input to the robot model
    (*input)(3) = 0.0;

    (*ekf_measurement)(1) = 0;
    // [measurement odometry] - velocity Y
    (*ekf_measurement)(2) = 0;
    // [measurement odometry] - angular velocity
    (*ekf_measurement)(3) = 0;
  nsec_old = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());

	return true;
}

////
// UPDATE
////
void VelmWheelFusion::updateHook() 
{
    nsec = std::chrono::duration_cast<std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
    loop_time = nsec.count() - nsec_old.count();
    nsec_old = nsec;

	if (RTT::NewData == in_twist_.read(*msg_twist))
	{
		// velocity X input to the robot model
		(*input)(1) = msg_twist->linear.x * double (loop_time)/1000000000;

		// velocity Y input to the robot model
		(*input)(2) = msg_twist->linear.y * double (loop_time)/1000000000;
		// angular velocity input to the robot model
		(*input)(3) = msg_twist->angular.z * double (loop_time)/1000000000;

	}
//if ((*input)(1) != 0)
  pose_x_from_input +=(*input)(1) * double (loop_time)/1000000000;

   //std::cout << "czas jazdy = "  << pose_x_from_input<< std::endl;

	if (RTT::NewData == in_odometry_.read(*msg_odometry))
	{

		// [measurement odometry] - velocity X
		(*ekf_measurement)(1) = msg_odometry->pose.pose.position.x;

		// [measurement odometry] - velocity Y
		(*ekf_measurement)(2) = msg_odometry->pose.pose.position.y;
		// [measurement odometry] - angular velocity
    tf::quaternionMsgToTF(msg_odometry->pose.pose.orientation, tf_quaternion);
	  (*ekf_measurement)(3) = tf_quaternion.getAngle();
 //   std::cout << "measure = "  << (*ekf_measurement)(1) << std::endl;
  }
  //std::cout << " input = "  << (*input)(3) << std::endl;

    filter->Update(sys_model,*input,meas_model,*ekf_measurement);

    posterior = filter->PostGet();

  std::cout << "KALMAN:      INPUT:        MEASUREMENT:"  << std::endl;
  std::cout<<posterior->ExpectedValueGet()[0]<<"      "<< pose_x_from_input << "                "<<(*ekf_measurement)(1)<< std::endl;
 //      << " Covariance = " << posterior->CovarianceGet() << "" << std::endl;

}



ORO_CREATE_COMPONENT(VelmWheelFusion)