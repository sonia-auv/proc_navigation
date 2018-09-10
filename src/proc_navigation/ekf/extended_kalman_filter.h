/**
 * \file	extended_kalman_filter.h
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

#ifndef Extended_Kalman_Filter_H_
#define Extended_Kalman_Filter_H_

#include <ros/ros.h>
#include <memory>
#include <eigen3/Eigen/Geometry>

typedef Eigen::Matrix<double, 3, 3> DvlMatrix;
typedef Eigen::Matrix<double, 4, 4> ImuMatrix;
typedef Eigen::Matrix<double, 4, 1> ImuEstimationMatrix;
typedef Eigen::Matrix<double, 3, 1> DvlEstimationMatrix;

namespace proc_navigation {

class ExtendedKalmanFilter {

    public:
    //==========================================================================
    //  P U B L I C   C / D T O R S
    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter();


    //==========================================================================
    //  P U B L I C   M E T H O D S
    void UpdateDvl(Eigen::Vector3d &measurement, Eigen::Vector3d &estimation);
    void UpdateImu(Eigen::Quaterniond &measurement, Eigen::Quaterniond &estimation);

    void Initialization(float pval, float qval, float rval);

    private:
    //==========================================================================
    //  P R I V A T E   M E T H O D S
    DvlEstimationMatrix DvlVectorToMatrix(Eigen::Vector3d measurement);
    ImuEstimationMatrix ImuVectorToMatrix(Eigen::Quaterniond measurement);
    Eigen::Quaterniond  ImuMatrixToVector(ImuEstimationMatrix estimation);
    Eigen::Vector3d     DvlMatrixToVector(DvlEstimationMatrix estimation);

    //==========================================================================
    //  P R I V A T E   M E M B E R S
    ImuMatrix imuPreviousNoise_;
    ImuMatrix imuPostNoise_;
    ImuMatrix imuJacobiansTransition_;
    ImuMatrix imuJacobiansMeasurement_;
    ImuMatrix imuProcessNoise_;
    ImuMatrix imuMeasurementNoise_;

    DvlMatrix dvlPreviousNoise_;
    DvlMatrix dvlPostNoise_;
    DvlMatrix dvlJacobiansTransition_;
    DvlMatrix dvlJacobiansMeasurement_;
    DvlMatrix dvlProcessNoise_;
    DvlMatrix dvlMeasurementNoise_;

};


}
#endif  // Extended_Kalman_Filter_H_
