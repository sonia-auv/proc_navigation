/**
 * \file	extended_kalman_filter.cc
 * \author	Olivier Lavoie <olavoie9507@gmail.com>
 * \date	20/08/2017
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "proc_navigation/ekf/extended_kalman_filter.h"

namespace proc_navigation {

//==============================================================================
//  C / D T O R S   S E C T I O N

//-----------------------------------------------------------------------------
//
    ExtendedKalmanFilter::ExtendedKalmanFilter() {

        Initialization(0.1, 1 ^ -4, 0.1);

    }

//-----------------------------------------------------------------------------
//
    ExtendedKalmanFilter::~ExtendedKalmanFilter() { }

//==============================================================================
//  M E T H O D   S E C T I O N
    void ExtendedKalmanFilter::Initialization(float pval, float qval, float rval) {

        imuPreviousNoise_.setIdentity();
        imuPostNoise_.setIdentity();
        imuJacobiansTransition_.setIdentity();
        imuJacobiansMeasurement_.setIdentity();
        imuProcessNoise_.setIdentity();
        imuMeasurementNoise_.setIdentity();

        dvlPreviousNoise_.setIdentity();
        dvlPostNoise_.setIdentity();
        dvlJacobiansTransition_.setIdentity();
        dvlJacobiansMeasurement_.setIdentity();
        dvlProcessNoise_.setIdentity();
        dvlMeasurementNoise_.setIdentity();

        imuPreviousNoise_    *= pval;
        imuProcessNoise_     *= qval;
        imuMeasurementNoise_ *= rval;

        dvlPreviousNoise_    *= pval;
        dvlProcessNoise_     *= qval;
        dvlMeasurementNoise_ *= rval;

    }


    void ExtendedKalmanFilter::UpdateDvl(Eigen::Vector3d &measurement, Eigen::Vector3d &estimation){

        DvlMatrix inverse_matrix, gain;
        DvlEstimationMatrix estimation_matrix = DvlVectorToMatrix(measurement);

        dvlPreviousNoise_ = dvlJacobiansTransition_ * dvlPostNoise_ * dvlJacobiansTransition_ + dvlProcessNoise_;

        inverse_matrix = dvlJacobiansMeasurement_ * dvlPreviousNoise_ * dvlJacobiansMeasurement_ + dvlMeasurementNoise_;

        inverse_matrix.inverse();

        gain = dvlPreviousNoise_ * dvlJacobiansMeasurement_ * inverse_matrix;

        estimation_matrix += gain * (DvlVectorToMatrix(measurement) - estimation_matrix);

        estimation = DvlMatrixToVector(estimation_matrix);

    }

    void ExtendedKalmanFilter::UpdateImu(Eigen::Quaterniond &measurement, Eigen::Quaterniond &estimation){

        ImuMatrix inverse_matrix, gain;
        ImuEstimationMatrix estimation_matrix = ImuVectorToMatrix(measurement);

        imuPreviousNoise_ = imuJacobiansTransition_ * imuPostNoise_ * imuJacobiansTransition_ + imuProcessNoise_;

        inverse_matrix = imuJacobiansMeasurement_ * imuPreviousNoise_ * imuJacobiansMeasurement_ + imuMeasurementNoise_;

        inverse_matrix.inverse();

        gain = imuPreviousNoise_ * imuJacobiansMeasurement_ * inverse_matrix;

        estimation_matrix += gain * (ImuVectorToMatrix(measurement) - estimation_matrix);

        estimation = ImuMatrixToVector(estimation_matrix);

    }


    DvlEstimationMatrix ExtendedKalmanFilter::DvlVectorToMatrix(Eigen::Vector3d measurement){

        DvlEstimationMatrix convert_tf;

        convert_tf(0,0) = measurement(0);
        convert_tf(1,0) = measurement(1);
        convert_tf(2,0) = measurement(2);

        return convert_tf;
    }

    ImuEstimationMatrix ExtendedKalmanFilter::ImuVectorToMatrix(Eigen::Quaterniond measurement){

        ImuEstimationMatrix convert_tf;

        convert_tf(0,0) = measurement.x();
        convert_tf(1,0) = measurement.y();
        convert_tf(2,0) = measurement.z();
        convert_tf(3,0) = measurement.w();

        return convert_tf;
    }

    Eigen::Quaterniond ExtendedKalmanFilter::ImuMatrixToVector(ImuEstimationMatrix estimation){

        Eigen::Quaterniond convert_eigen(estimation(0,0), estimation(1,0), estimation(2,0), estimation(3,0));

        return convert_eigen;
    }

    Eigen::Vector3d ExtendedKalmanFilter::DvlMatrixToVector(DvlEstimationMatrix estimation){

        Eigen::Vector3d convert_eigen;

        convert_eigen << estimation(0,0), estimation(1,0), estimation(2,0);

        return convert_eigen;
    }

//-----------------------------------------------------------------------------
//


}  // namespace proc_navigation
