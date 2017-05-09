/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_localization/ekf.h"
#include "robot_localization/filter_common.h"

#include <xmlrpcpp/XmlRpcException.h>

#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

    // Now set up the relevant matrices
    static Eigen::VectorXd *stateSubset = NULL;//(updateSize);                              // x (in most literature)
    static Eigen::VectorXd *measurementSubset = NULL;//(updateSize);                        // z
    static Eigen::MatrixXd *measurementCovarianceSubset = NULL;//(updateSize, updateSize);  // R
    static Eigen::MatrixXd *stateToMeasurementSubset = NULL;//(updateSize, state_.rows());  // H
    static Eigen::MatrixXd *kalmanGainSubset = NULL;//(state_.rows(), updateSize);          // K
    static Eigen::VectorXd *innovationSubset = NULL;//(updateSize);                         // z - Hx
    // how many states are going to be updated in current loop
    static std::vector<size_t> *updateIndices = NULL;
    size_t updateSize;
    // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
    static Eigen::MatrixXd *pht = NULL;// = estimateErrorCovariance_ * stateToMeasurementSubset.transpose();
    static Eigen::MatrixXd *hphrInv = NULL;//  = (stateToMeasurementSubset * pht + measurementCovarianceSubset).inverse();
    // (4) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
    static Eigen::MatrixXd *gainResidual = NULL;// = identity_;

    size_t i;
    size_t j;
      //
      //    PREDICT
    double roll;// = state_(StateMemberRoll);
    double pitch;// = state_(StateMemberPitch);
    double yaw;// = state_(StateMemberYaw);
    double xVel;// = state_(StateMemberVx);
    double yVel;// = state_(StateMemberVy);
    double zVel;//= state_(StateMemberVz);
    double rollVel;// = state_(StateMemberVroll);
    double pitchVel;// = state_(StateMemberVpitch);
    double yawVel;// = state_(StateMemberVyaw);
    double xAcc;// = state_(StateMemberAx);
    double yAcc;// = state_(StateMemberAy);
    double zAcc;// = state_(StateMemberAz);

    // We'll eed these trig calculations a lot.
    double sp;// = ::sin(pitch);
    double cp;// = ::cos(pitch);
    double sr;// = ::sin(roll);
    double cr;// = ::cos(roll);
    double sy;// = ::sin(yaw);
    double cy;// = ::cos(yaw);
    double xCoeff;// = 0.0;
    double yCoeff;// = 0.0;
    double zCoeff;// = 0.0;
    double oneHalfATSquared;// = 0.5 * delta * delta;
    double dFx_dR;// = (yCoeff * yVel + zCoeff * zVel) * delta +
    double dFR_dR;// = 1 + (yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFx_dP;// = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
    double dFR_dP;// = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFx_dY;// = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
    double dFR_dY;// = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFy_dR;// = (yCoeff * yVel + zCoeff * zVel) * delta +
    double dFP_dR;// = (yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFy_dP;// = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
    double dFP_dP;// = 1 + (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFy_dY;// = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
    double dFP_dY;// = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFz_dR;// = (yCoeff * yVel + zCoeff * zVel) * delta +
    double dFY_dR;// = (yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFz_dP;// = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
    double dFY_dP;// = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

static Eigen::MatrixXd *processNoiseCovariance = NULL;// = &processNoiseCovariance_;
namespace RobotLocalization
{
  Ekf::Ekf(std::vector<double>) :
    FilterBase()  // Must initialize filter base!
  {
    size_t i;
    size_t j;
    // Now set up the relevant matrices
    stateSubset = new Eigen::VectorXd();//(updateSize);                              // x (in most literature)
    measurementSubset = new Eigen::VectorXd();//(updateSize);                        // z
    measurementCovarianceSubset = new Eigen::MatrixXd();//(updateSize, updateSize);  // R
    stateToMeasurementSubset = new Eigen::MatrixXd();//(updateSize, state_.rows());  // H
    kalmanGainSubset = new Eigen::MatrixXd();//(state_.rows(), updateSize);          // K
    innovationSubset = new Eigen::VectorXd();//(updateSize);                         // z - Hx
    // how many states are going to be updated in current loop
    updateIndices = new std::vector<size_t>;
    size_t updateSize;
    // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
    pht = new Eigen::MatrixXd();// = estimateErrorCovariance_ * stateToMeasurementSubset.transpose();
    hphrInv = new Eigen::MatrixXd();//  = (stateToMeasurementSubset * pht + measurementCovarianceSubset).inverse();
    // (4) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
    gainResidual = new Eigen::MatrixXd();// = identity_;
      //
      //    PREDICT
      //    double roll;/ = state_(StateMemberRoll);
    double roll;// = state_(StateMemberRoll);
    double pitch;// = state_(StateMemberPitch);
    double yaw;// = state_(StateMemberYaw);
    double xVel;// = state_(StateMemberVx);
    double yVel;// = state_(StateMemberVy);
    double zVel;//= state_(StateMemberVz);
    double rollVel;// = state_(StateMemberVroll);
    double pitchVel;// = state_(StateMemberVpitch);
    double yawVel;// = state_(StateMemberVyaw);
    double xAcc;// = state_(StateMemberAx);
    double yAcc;// = state_(StateMemberAy);
    double zAcc;// = state_(StateMemberAz);

    // We'll eed these trig calculations a lot.
    double sp;// = ::sin(pitch);
    double cp;// = ::cos(pitch);
    double sr;// = ::sin(roll);
    double cr;// = ::cos(roll);
    double sy;// = ::sin(yaw);
    double cy;// = ::cos(yaw);
    double xCoeff;// = 0.0;
    double yCoeff;// = 0.0;
    double zCoeff;// = 0.0;
    double oneHalfATSquared;// = 0.5 * delta * delta;
    double dFx_dR;// = (yCoeff * yVel + zCoeff * zVel) * delta +
    double dFR_dR;// = 1 + (yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFx_dP;// = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
    double dFR_dP;// = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFx_dY;// = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
    double dFR_dY;// = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFy_dR;// = (yCoeff * yVel + zCoeff * zVel) * delta +
    double dFP_dR;// = (yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFy_dP;// = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
    double dFP_dP;// = 1 + (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFy_dY;// = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
    double dFP_dY;// = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFz_dR;// = (yCoeff * yVel + zCoeff * zVel) * delta +
    double dFY_dR;// = (yCoeff * pitchVel + zCoeff * yawVel) * delta;
    double dFz_dP;// = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
    double dFY_dP;// = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

    processNoiseCovariance = new Eigen::MatrixXd();// = &processNoiseCovariance_;
  }

  Ekf::~Ekf()
  {

  }

  void Ekf::correct(const Measurement &measurement)
  {
    FB_DEBUG("---------------------- Ekf::correct ----------------------\n" <<
             "State is:\n" << state_ << "\n"
             "Topic is:\n" << measurement.topicName_ << "\n"
             "Measurement is:\n" << measurement.measurement_ << "\n"
             "Measurement topic name is:\n" << measurement.topicName_ << "\n\n"
             "Measurement covariance is:\n" << measurement.covariance_ << "\n");

    // We don't want to update everything, so we need to build matrices that only update
    // the measured parts of our state vector. Throughout prediction and correction, we
    // attempt to maximize efficiency in Eigen.

    // First, determine how many state vector values we're updating
    updateIndices->clear();
    for (i = 0; i < measurement.updateVector_.size(); ++i)
    {
      if (measurement.updateVector_[i])
      {
        // Handle nan and inf values in measurements
        if (std::isnan(measurement.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was nan. Excluding from update.\n");
        }
        else if (std::isinf(measurement.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was inf. Excluding from update.\n");
        }
        else
        {
          updateIndices->push_back(i);
        }
      }
    }

    FB_DEBUG("Update indices are:\n" << (*updateIndices) << "\n");

    updateSize = updateIndices->size();

    stateSubset->resize(updateSize);                              // x (in most literature)
    measurementSubset->resize(updateSize);                        // z
    measurementCovarianceSubset->resize(updateSize, updateSize);  // R
    stateToMeasurementSubset->resize(updateSize, state_.rows());  // H
    kalmanGainSubset->resize(state_.rows(), updateSize);          // K
    innovationSubset->resize(updateSize);                         // z - Hx

    stateSubset->setZero();
    measurementSubset->setZero();
    measurementCovarianceSubset->setZero();
    stateToMeasurementSubset->setZero();
    kalmanGainSubset->setZero();
    innovationSubset->setZero();

    // Now build the sub-matrices from the full-sized matrices
    for (i = 0; i < updateSize; ++i)
    {
      (*measurementSubset)(i) = measurement.measurement_((*updateIndices)[i]);
      (*stateSubset)(i) = state_((*updateIndices)[i]);

      for (j = 0; j < updateSize; ++j)
      {
        (*measurementCovarianceSubset)(i, j) = measurement.covariance_((*updateIndices)[i], (*updateIndices)[j]);
      }

      // Handle negative (read: bad) covariances in the measurement. Rather
      // than exclude the measurement or make up a covariance, just take
      // the absolute value.
      if ((*measurementCovarianceSubset)(i, i) < 0)
      {
        FB_DEBUG("WARNING: Negative covariance for index " << i <<
                 " of measurement (value is" << (*measurementCovarianceSubset)(i, i) <<
                 "). Using absolute value...\n");

        (*measurementCovarianceSubset)(i, i) = ::fabs((*measurementCovarianceSubset)(i, i));
      }

      // If the measurement variance for a given variable is very
      // near 0 (as in e-50 or so) and the variance for that
      // variable in the covariance matrix is also near zero, then
      // the Kalman gain computation will blow up. Really, no
      // measurement can be completely without error, so add a small
      // amount in that case.
      if ((*measurementCovarianceSubset)(i, i) < 1e-9)
      {
        FB_DEBUG("WARNING: measurement had very small error covariance for index " << (*updateIndices)[i] <<
                 ". Adding some noise to maintain filter stability.\n");

        (*measurementCovarianceSubset)(i, i) = 1e-9;
      }
    }

    // The state-to-measurement function, h, will now be a measurement_size x full_state_size
    // matrix, with ones in the (i, i) locations of the values to be updated
    for (i = 0; i < updateSize; ++i)
    {
      (*stateToMeasurementSubset)(i, (*updateIndices)[i]) = 1;
    }

    FB_DEBUG("Current state subset is:\n" << (*stateSubset) <<
             "\nMeasurement subset is:\n" << (*measurementSubset) <<
             "\nMeasurement covariance subset is:\n" << (*measurementCovarianceSubset) <<
             "\nState-to-measurement subset is:\n" << (*stateToMeasurementSubset) << "\n");

    // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
    *pht = estimateErrorCovariance_ * stateToMeasurementSubset->transpose();
    *hphrInv  = ((*stateToMeasurementSubset) * (*pht) + (*measurementCovarianceSubset)).inverse();
    kalmanGainSubset->noalias() = (*pht) * (*hphrInv);

    (*innovationSubset) = ((*measurementSubset) - (*stateSubset));

    // Wrap angles in the innovation
    for (i = 0; i < updateSize; ++i)
    {
      if ((*updateIndices)[i] == StateMemberRoll  ||
          (*updateIndices)[i] == StateMemberPitch ||
          (*updateIndices)[i] == StateMemberYaw)
      {
        while ((*innovationSubset)(i) < -PI)
        {
          (*innovationSubset)(i) += TAU;
        }

        while ((*innovationSubset)(i) > PI)
        {
          (*innovationSubset)(i) -= TAU;
        }
      }
    }
    
    // (2) Check Mahalanobis distance between mapped measurement and state.
    if (checkMahalanobisThreshold((*innovationSubset), (*hphrInv), measurement.mahalanobisThresh_))
    {
      // (3) Apply the gain to the difference between the state and measurement: x = x + K(z - Hx)
      state_.noalias() += (*kalmanGainSubset) * (*innovationSubset);
      FB_DEBUG("PHT:\n" << (*pht) <<
             "\nhphrInv:\n" << (*hphrInv) <<
             "\nestimateErrorCovariance_:\n" << estimateErrorCovariance_ <<
             "\n(*stateToMeasurementSubset) * (*pht):\n" << (*stateToMeasurementSubset) * (*pht) << "\n"<<
             "\n(*measurementCovarianceSubset):\n" << (*measurementCovarianceSubset) << "\n"<<
             "\n(*stateToMeasurementSubset) * (*pht) + (*measurementCovarianceSubset):\n" << (*stateToMeasurementSubset) * (*pht) + (*measurementCovarianceSubset) << "\n"<<
             "\nSUM RANK:\n" << ((*stateToMeasurementSubset) * (*pht) + (*measurementCovarianceSubset)).eigenvalues() << "\n");
      // (4) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
      *gainResidual = identity_;
      gainResidual->noalias() -= (*kalmanGainSubset) * (*stateToMeasurementSubset);
      estimateErrorCovariance_ = (*gainResidual) * estimateErrorCovariance_ * gainResidual->transpose();
      estimateErrorCovariance_.noalias() += (*kalmanGainSubset) *
                                            (*measurementCovarianceSubset) *
                                            kalmanGainSubset->transpose();

      // Handle wrapping of angles
      wrapStateAngles();

      FB_DEBUG("Kalman gain subset is:\n" << (*kalmanGainSubset) <<
               "\nInnovation is:\n" << (*innovationSubset) <<
               "\nCorrected full state is:\n" << state_ <<
               "\nCorrected full estimate error covariance is:\n" << estimateErrorCovariance_ <<
               "\n\n---------------------- /Ekf::correct ----------------------\n");
    }
  }

  void Ekf::predict(const double referenceTime, const double delta)
  {
    FB_DEBUG("---------------------- Ekf::predict ----------------------\n" <<
             "delta is " << delta << "\n" <<
             "state is " << state_ << "\n");

    roll = state_(StateMemberRoll);
    pitch = state_(StateMemberPitch);
    yaw = state_(StateMemberYaw);
    xVel = state_(StateMemberVx);
    yVel = state_(StateMemberVy);
    zVel = state_(StateMemberVz);
    rollVel = state_(StateMemberVroll);
    pitchVel = state_(StateMemberVpitch);
    yawVel = state_(StateMemberVyaw);
    xAcc = state_(StateMemberAx);
    yAcc = state_(StateMemberAy);
    zAcc = state_(StateMemberAz);

    // We'll need these trig calculations a lot.
    sp = ::sin(pitch);
    cp = ::cos(pitch);

    sr = ::sin(roll);
    cr = ::cos(roll);

    sy = ::sin(yaw);
    cy = ::cos(yaw);

    prepareControl(referenceTime, delta);

    // Prepare the transfer function
    transferFunction_(StateMemberX, StateMemberVx) = cy * cp * delta;
    transferFunction_(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta;
    transferFunction_(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta;
    transferFunction_(StateMemberX, StateMemberAx) = 0.5 * transferFunction_(StateMemberX, StateMemberVx) * delta;
    transferFunction_(StateMemberX, StateMemberAy) = 0.5 * transferFunction_(StateMemberX, StateMemberVy) * delta;
    transferFunction_(StateMemberX, StateMemberAz) = 0.5 * transferFunction_(StateMemberX, StateMemberVz) * delta;
    transferFunction_(StateMemberY, StateMemberVx) = sy * cp * delta;
    transferFunction_(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta;
    transferFunction_(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta;
    transferFunction_(StateMemberY, StateMemberAx) = 0.5 * transferFunction_(StateMemberY, StateMemberVx) * delta;
    transferFunction_(StateMemberY, StateMemberAy) = 0.5 * transferFunction_(StateMemberY, StateMemberVy) * delta;
    transferFunction_(StateMemberY, StateMemberAz) = 0.5 * transferFunction_(StateMemberY, StateMemberVz) * delta;
    transferFunction_(StateMemberZ, StateMemberVx) = -sp * delta;
    transferFunction_(StateMemberZ, StateMemberVy) = cp * sr * delta;
    transferFunction_(StateMemberZ, StateMemberVz) = cp * cr * delta;
    transferFunction_(StateMemberZ, StateMemberAx) = 0.5 * transferFunction_(StateMemberZ, StateMemberVx) * delta;
    transferFunction_(StateMemberZ, StateMemberAy) = 0.5 * transferFunction_(StateMemberZ, StateMemberVy) * delta;
    transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * transferFunction_(StateMemberZ, StateMemberVz) * delta;
    transferFunction_(StateMemberRoll, StateMemberVroll) = transferFunction_(StateMemberX, StateMemberVx);
    transferFunction_(StateMemberRoll, StateMemberVpitch) = transferFunction_(StateMemberX, StateMemberVy);
    transferFunction_(StateMemberRoll, StateMemberVyaw) = transferFunction_(StateMemberX, StateMemberVz);
    transferFunction_(StateMemberPitch, StateMemberVroll) = transferFunction_(StateMemberY, StateMemberVx);
    transferFunction_(StateMemberPitch, StateMemberVpitch) = transferFunction_(StateMemberY, StateMemberVy);
    transferFunction_(StateMemberPitch, StateMemberVyaw) = transferFunction_(StateMemberY, StateMemberVz);
    transferFunction_(StateMemberYaw, StateMemberVroll) = transferFunction_(StateMemberZ, StateMemberVx);
    transferFunction_(StateMemberYaw, StateMemberVpitch) = transferFunction_(StateMemberZ, StateMemberVy);
    transferFunction_(StateMemberYaw, StateMemberVyaw) = transferFunction_(StateMemberZ, StateMemberVz);
    transferFunction_(StateMemberVx, StateMemberAx) = delta;
    transferFunction_(StateMemberVy, StateMemberAy) = delta;
    transferFunction_(StateMemberVz, StateMemberAz) = delta;

    // Prepare the transfer function Jacobian. This function is analytically derived from the
    // transfer function.
    xCoeff = 0.0;
    yCoeff = 0.0;
    zCoeff = 0.0;
    oneHalfATSquared = 0.5 * delta * delta;

    yCoeff = cy * sp * cr + sy * sr;
    zCoeff = -cy * sp * sr + sy * cr;
    dFx_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    dFR_dR = 1 + (yCoeff * pitchVel + zCoeff * yawVel) * delta;

    xCoeff = -cy * sp;
    yCoeff = cy * cp * sr;
    zCoeff = cy * cp * cr;
    dFx_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    dFR_dP = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

    xCoeff = -sy * cp;
    yCoeff = -sy * sp * sr - cy * cr;
    zCoeff = -sy * sp * cr + cy * sr;
    dFx_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    dFR_dY = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

    yCoeff = sy * sp * cr - cy * sr;
    zCoeff = -sy * sp * sr - cy * cr;
    dFy_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    dFP_dR = (yCoeff * pitchVel + zCoeff * yawVel) * delta;

    xCoeff = -sy * sp;
    yCoeff = sy * cp * sr;
    zCoeff = sy * cp * cr;
    dFy_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    dFP_dP = 1 + (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

    xCoeff = cy * cp;
    yCoeff = cy * sp * sr - sy * cr;
    zCoeff = cy * sp * cr + sy * sr;
    dFy_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    dFP_dY = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

    yCoeff = cp * cr;
    zCoeff = -cp * sr;
    dFz_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    dFY_dR = (yCoeff * pitchVel + zCoeff * yawVel) * delta;

    xCoeff = -cp;
    yCoeff = -sp * sr;
    zCoeff = -sp * cr;
    dFz_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    dFY_dP = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

    // Much of the transfer function Jacobian is identical to the transfer function
    transferFunctionJacobian_ = transferFunction_;
    transferFunctionJacobian_(StateMemberX, StateMemberRoll) = dFx_dR;
    transferFunctionJacobian_(StateMemberX, StateMemberPitch) = dFx_dP;
    transferFunctionJacobian_(StateMemberX, StateMemberYaw) = dFx_dY;
    transferFunctionJacobian_(StateMemberY, StateMemberRoll) = dFy_dR;
    transferFunctionJacobian_(StateMemberY, StateMemberPitch) = dFy_dP;
    transferFunctionJacobian_(StateMemberY, StateMemberYaw) = dFy_dY;
    transferFunctionJacobian_(StateMemberZ, StateMemberRoll) = dFz_dR;
    transferFunctionJacobian_(StateMemberZ, StateMemberPitch) = dFz_dP;
    transferFunctionJacobian_(StateMemberRoll, StateMemberRoll) = dFR_dR;
    transferFunctionJacobian_(StateMemberRoll, StateMemberPitch) = dFR_dP;
    transferFunctionJacobian_(StateMemberRoll, StateMemberYaw) = dFR_dY;
    transferFunctionJacobian_(StateMemberPitch, StateMemberRoll) = dFP_dR;
    transferFunctionJacobian_(StateMemberPitch, StateMemberPitch) = dFP_dP;
    transferFunctionJacobian_(StateMemberPitch, StateMemberYaw) = dFP_dY;
    transferFunctionJacobian_(StateMemberYaw, StateMemberRoll) = dFY_dR;
    transferFunctionJacobian_(StateMemberYaw, StateMemberPitch) = dFY_dP;

    FB_DEBUG("Transfer function is:\n" << transferFunction_ <<
             "\nTransfer function Jacobian is:\n" << transferFunctionJacobian_ <<
             "\nProcess noise covariance is:\n" << processNoiseCovariance_ <<
             "\nCurrent state is:\n" << state_ << "\n");

    processNoiseCovariance = &processNoiseCovariance_;

    if (useDynamicProcessNoiseCovariance_)
    {
      computeDynamicProcessNoiseCovariance(state_, delta);
      processNoiseCovariance = &dynamicProcessNoiseCovariance_;
    }

    // (1) Apply control terms, which are actually accelerations
    state_(StateMemberVroll) += controlAcceleration_(ControlMemberVroll) * delta;
    state_(StateMemberVpitch) += controlAcceleration_(ControlMemberVpitch) * delta;
    state_(StateMemberVyaw) += controlAcceleration_(ControlMemberVyaw) * delta;

    state_(StateMemberAx) = (controlUpdateVector_[ControlMemberVx] ?
      controlAcceleration_(ControlMemberVx) : state_(StateMemberAx));
    state_(StateMemberAy) = (controlUpdateVector_[ControlMemberVy] ?
      controlAcceleration_(ControlMemberVy) : state_(StateMemberAy));
    state_(StateMemberAz) = (controlUpdateVector_[ControlMemberVz] ?
      controlAcceleration_(ControlMemberVz) : state_(StateMemberAz));

    // (2) Project the state forward: x = Ax + Bu (really, x = f(x, u))
    state_ = transferFunction_ * state_;

    // Handle wrapping
    wrapStateAngles();

    FB_DEBUG("Predicted state is:\n" << state_ <<
             "\nCurrent estimate error covariance is:\n" <<  estimateErrorCovariance_ << "\n");

    // (3) Project the error forward: P = J * P * J' + Q
    estimateErrorCovariance_ = (transferFunctionJacobian_ *
                                estimateErrorCovariance_ *
                                transferFunctionJacobian_.transpose());
    estimateErrorCovariance_.noalias() += delta * (*processNoiseCovariance);

    FB_DEBUG("Predicted estimate error covariance is:\n" << estimateErrorCovariance_ <<
             "\n\n--------------------- /Ekf::predict ----------------------\n");
  }

}  // namespace RobotLocalization
