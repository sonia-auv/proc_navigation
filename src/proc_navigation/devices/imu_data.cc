/**
 * \file	imu_data.cc
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	9/14/2016
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

#include <ros/ros.h>
#include "imu_data.h"


namespace proc_navigation {

    //==============================================================================
    // C / D T O R S   S E C T I O N

    //------------------------------------------------------------------------------
    //
    IMUData::IMUData() : quaternion_(0.0, 0.0, 0.0, 0.0)
    {
        eulerAngle_         = Eigen::Vector3d::Zero();
        angularVelocity_    = Eigen::Vector3d::Zero();
        linearAcceleration_ = Eigen::Vector3d::Zero();

    }

    //------------------------------------------------------------------------------
    //
    IMUData::~IMUData() { }

    //==============================================================================
    // M E T H O D   S E C T I O N

    //-----------------------------------------------------------------------------
    //
    void IMUData::IMUMsgCallback(sensor_msgs::Imu msg)
    {
        quaternion_.x() = msg.orientation.x;
        quaternion_.y() = msg.orientation.y;
        quaternion_.z() = msg.orientation.z;
        quaternion_.w() = msg.orientation.w;

        // We use the IMU's data sheet transformation because Eigen return yaw on 0-180 basis,
        // which makes it impossible to use for us. This formulae gives yaw on 0-360 and independant from
        // roll pitch yaw
        double q0 = quaternion_.w(), q1 = quaternion_.x(), q2 = quaternion_.y(), q3 = quaternion_.z();
        double m11 = 2 * (q0*q0 - 0.5 + q1*q1);
        double m12 = 2 * (q1*q2 + q0*q3);
        double m13 = 2 * (q1*q3 - q0*q2);
        double m23 = 2 * (q2*q3 + q0*q1);
        double m33 = 2 * (q0*q0 - 0.5 + q3*q3);

        double pitch = std::asin(-m13);
        double roll = std::atan2(m23,m33);
        double yaw = std::atan2(m12, m11) + M_PI;

        yaw = std::fmod((yaw + M_PI), M_PI*2.0);

        eulerAngle_ << roll * RAD_TO_DEGREE, pitch * RAD_TO_DEGREE, yaw * RAD_TO_DEGREE;

        linearAcceleration_ << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;

        angularVelocity_ << msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z;

        SetNewDataReady();
    }

    //-----------------------------------------------------------------------------
    //
    Eigen::Quaterniond IMUData::GetQuaternion()
    {
        return quaternion_;
    }

    //-----------------------------------------------------------------------------
    //
    Eigen::Vector3d IMUData::GetOrientation()
    {
        return eulerAngle_;
    }

    //-----------------------------------------------------------------------------
    //
    Eigen::Vector3d IMUData::GetAngularVelocity()
    {
        return angularVelocity_;
    }

}
