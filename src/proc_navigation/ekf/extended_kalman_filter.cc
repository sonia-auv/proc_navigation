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
    ExtendedKalmanFilter::ExtendedKalmanFilter()
    {
        Initialization(0.1, 1 ^ -4, 0.1);
    }

//==============================================================================
//  M E T H O D   S E C T I O N
    void ExtendedKalmanFilter::Initialization(float pval, float qval, float rval)
    {
        previousNoise_.setIdentity();
        postNoise_.setIdentity();
        jacobiansTransition_.setIdentity();
        jacobiansMeasurement_.setIdentity();
        processNoise_.setIdentity();
        measurementNoise_.setIdentity();

        previousNoise_    *= pval;
        processNoise_     *= qval;
        measurementNoise_ *= rval;
    }


    void ExtendedKalmanFilter::Update(Eigen::Vector3d &measurement, Eigen::Vector3d &estimation)
    {
        Eigen::Matrix3d inverse_matrix;
        Eigen::Matrix3d gain;
        Eigen::Vector3d stateEstimation = measurement;

        previousNoise_ = jacobiansTransition_ * postNoise_ * jacobiansTransition_ + processNoise_;

        inverse_matrix = jacobiansMeasurement_ * previousNoise_ * jacobiansMeasurement_ + measurementNoise_;

        inverse_matrix.inverse();

        gain = previousNoise_ * jacobiansMeasurement_ * inverse_matrix;

        stateEstimation += gain * (measurement - stateEstimation);

        estimation = stateEstimation;
    }

//-----------------------------------------------------------------------------
//


}  // namespace proc_navigation
