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
    void update_dvl(Eigen::Vector3d &measurement, Eigen::Vector3d &estimation);
    void update_imu(Eigen::Quaterniond &measurement, Eigen::Quaterniond &estimation);

    void initialization(float pval, float qval, float rval);

    private:
    //==========================================================================
    //  P R I V A T E   M E T H O D S
    DvlEstimationMatrix dvl_tf_to_eigen(Eigen::Vector3d measurement);
    ImuEstimationMatrix imu_tf_to_eigen(Eigen::Quaterniond measurement);
    Eigen::Quaterniond  imu_eigen_to_tf(ImuEstimationMatrix estimation);
    Eigen::Vector3d     dvl_eigen_to_tf(DvlEstimationMatrix estimation);

    //==========================================================================
    //  P R I V A T E   M E M B E R S
    ImuMatrix imu_previous_noise_;
    ImuMatrix imu_post_noise_;
    ImuMatrix imu_jacobians_transition_;
    ImuMatrix imu_jacobians_measurement_;
    ImuMatrix imu_process_noise_;
    ImuMatrix imu_measurement_noise_;

    DvlMatrix dvl_previous_noise_;       
    DvlMatrix dvl_post_noise_;
    DvlMatrix dvl_jacobians_transition_;
    DvlMatrix dvl_jacobians_measurement_;
    DvlMatrix dvl_process_noise_;
    DvlMatrix dvl_measurement_noise_;

};


}
#endif  // Extended_Kalman_Filter_H_
