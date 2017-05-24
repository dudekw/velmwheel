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

#ifndef ROBOT_LOCALIZATION_EKF_H
#define ROBOT_LOCALIZATION_EKF_H

#include "robot_localization/filter_base.h"
#include <memory>

#include <fstream>
#include <vector>
#include <set>
#include <queue>

namespace RobotLocalization
{

//! @brief Extended Kalman filter class
//!
//! Implementation of an extended Kalman filter (EKF). This
//! class derives from FilterBase and overrides the predict()
//! and correct() methods in keeping with the discrete time
//! EKF algorithm.
//!
class Ekf: public FilterBase
{

    private:
        // Now set up the relevant matrices
        static std::auto_ptr<Eigen::VectorXd> stateSubset;//= NULL;//(updateSize);                              // x (in most literature)
        static std::auto_ptr<Eigen::VectorXd> measurementSubset;//= NULL;//(updateSize);                        // z
        static std::auto_ptr<Eigen::MatrixXd> measurementCovarianceSubset;//= NULL;//(updateSize, updateSize);  // R
        static std::auto_ptr<Eigen::MatrixXd> stateToMeasurementSubset;//= NULL;//(updateSize, state_.rows());  // H
        static std::auto_ptr<Eigen::MatrixXd> kalmanGainSubset;//= NULL;//(state_.rows(), updateSize);          // K
        static std::auto_ptr<Eigen::VectorXd> innovationSubset;//= NULL;//(updateSize);                         // z - Hx
        // how many states are going to be updated in current loop
        static std::auto_ptr<std::vector<size_t> > updateIndices;//= NULL;
        size_t updateSize;
        // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
        static std::auto_ptr<Eigen::MatrixXd> pht;//= NULL;// = estimateErrorCovariance_ *stateToMeasurementSubset.transpose();
        static std::auto_ptr<Eigen::MatrixXd> phrInv;//= NULL;//  = (stateToMeasurementSubset *pht + measurementCovarianceSubset).inverse();
        static std::auto_ptr<Eigen::MatrixXd> hphrInv;

        // (4) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
        static std::auto_ptr<Eigen::MatrixXd> gainResidual;//= NULL;// = identity_;
        static std::auto_ptr<Eigen::MatrixXd> processNoiseCovariance_ptr;//= NULL;// = &processNoiseCovariance_;
        static std::auto_ptr<Eigen::MatrixXd> dynamicProcessNoiseCovariance_ptr;


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

  public:
    //! @brief Constructor for the Ekf class
    //!
    //! @param[in] args - Generic argument container (not used here, but
    //! needed so that the ROS filters can pass arbitrary arguments to
    //! templated filter types).
    //!
    explicit Ekf(std::vector<double> args = std::vector<double>());

    //! @brief Destructor for the Ekf class
    //!
    ~Ekf();

    //! @brief Carries out the correct step in the predict/update cycle.
    //!
    //! @param[in] measurement - The measurement to fuse with our estimate
    //!
    void correct(const Measurement &measurement);

    //! @brief Carries out the predict step in the predict/update cycle.
    //!
    //! Projects the state and error matrices forward using a model of
    //! the vehicle's motion.
    //!
    //! @param[in] referenceTime - The time at which the prediction is being made
    //! @param[in] delta - The time step over which to predict.
    //!
    void predict(const double referenceTime, const double delta);

};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_EKF_H
