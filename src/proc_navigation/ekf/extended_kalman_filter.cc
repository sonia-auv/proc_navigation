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

        initialization();

    }

//-----------------------------------------------------------------------------
//
    ExtendedKalmanFilter::~ExtendedKalmanFilter() { }

//==============================================================================
//  M E T H O D   S E C T I O N
    void ExtendedKalmanFilter::initialization() {

        imu_previous_noise_.setIdentity();
        imu_post_noise_.setIdentity();
        imu_jacobians_transition_.setIdentity();
        imu_jacobians_measurement_.setIdentity();
        imu_process_noise_.setIdentity();
        imu_measurement_noise_.setIdentity();

        dvl_previous_noise_.setIdentity();
        dvl_post_noise_.setIdentity();
        dvl_jacobians_transition_.setIdentity();
        dvl_jacobians_measurement_.setIdentity();
        dvl_process_noise_.setIdentity();
        dvl_measurement_noise_.setIdentity();
        
    }


    void ExtendedKalmanFilter::update_dvl(tf::Vector3 measurement, tf::Vector3 &estimation){

        DvlMatrix inverse_matrix, gain;
        DvlEstimationMatrix estimation_matrix = dvl_tf_to_eigen(measurement);

        dvl_previous_noise_ = dvl_jacobians_transition_ * dvl_post_noise_ * dvl_jacobians_transition_ + dvl_process_noise_;

        inverse_matrix = dvl_jacobians_measurement_ * dvl_previous_noise_ * dvl_jacobians_measurement_ + dvl_measurement_noise_;

        inverse_matrix.inverse();

        gain = dvl_previous_noise_ * dvl_jacobians_measurement_ * inverse_matrix;

        estimation_matrix += gain * (dvl_tf_to_eigen(measurement) - estimation_matrix);

        estimation = dvl_eigen_to_tf(estimation_matrix);

    }

    void ExtendedKalmanFilter::update_imu(tf::Quaternion measurement, tf::Quaternion &estimation){

        ImuMatrix inverse_matrix, gain;
        ImuEstimationMatrix estimation_matrix = imu_tf_to_eigen(measurement);

        imu_previous_noise_ = imu_jacobians_transition_ * imu_post_noise_ * imu_jacobians_transition_ + imu_process_noise_;

        inverse_matrix = imu_jacobians_measurement_ * imu_previous_noise_ * imu_jacobians_measurement_ + imu_measurement_noise_;

        inverse_matrix.inverse();

        gain = imu_previous_noise_ * imu_jacobians_measurement_ * inverse_matrix;

        estimation_matrix += gain * (imu_tf_to_eigen(measurement) - estimation_matrix);

        estimation = imu_eigen_to_tf(estimation_matrix);

    }


    DvlEstimationMatrix ExtendedKalmanFilter::dvl_tf_to_eigen(tf::Vector3 measurement){

        DvlEstimationMatrix convert_tf;

        convert_tf(0,0) = float(measurement.x());
        convert_tf(1,0) = float(measurement.y());
        convert_tf(2,0) = float(measurement.z());

        return convert_tf;
    }

    ImuEstimationMatrix ExtendedKalmanFilter::imu_tf_to_eigen(tf::Quaternion measurement){

        ImuEstimationMatrix convert_tf;

        convert_tf(0,0) = float(measurement.x());
        convert_tf(1,0) = float(measurement.y());
        convert_tf(2,0) = float(measurement.z());
        convert_tf(3,0) = float(measurement.w());

        return convert_tf;
    }

    tf::Quaternion ExtendedKalmanFilter::imu_eigen_to_tf(ImuEstimationMatrix estimation){

        tf::Quaternion convert_eigen;

        convert_eigen.setX(estimation(0,0));
        convert_eigen.setY(estimation(1,0));
        convert_eigen.setZ(estimation(2,0));
        convert_eigen.setW(estimation(3,0));

        return convert_eigen;
    }

    tf::Vector3 ExtendedKalmanFilter::dvl_eigen_to_tf(DvlEstimationMatrix estimation){

        tf::Vector3 convert_eigen;

        convert_eigen.setX(estimation(0,0));
        convert_eigen.setY(estimation(1,0));
        convert_eigen.setZ(estimation(2,0));

        return convert_eigen;
    }

//-----------------------------------------------------------------------------
//


}  // namespace proc_navigation
