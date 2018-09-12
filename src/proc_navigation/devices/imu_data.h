/**
 * \file	imu_data.h
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

#ifndef PROC_NAVIGATION_IMUDATA_HPP
#define PROC_NAVIGATION_IMUDATA_HPP

#include <sensor_msgs/Imu.h>
#include "navigation_device.h"
#include <eigen3/Eigen/Geometry>

namespace proc_navigation {

class IMUData: public NavigationDevice {
public:
    //==========================================================================
    // C O N S T  ,  T Y P E D E F   A N D   E N U M

    const double RAD_TO_DEGREE = 180.0f / M_PI;

    //==========================================================================
    // P U B L I C   C / D T O R S

    IMUData();
    ~IMUData();

    //==========================================================================
    // P U B L I C   M E T H O D S

    void IMUMsgCallback(sensor_msgs::Imu msg);
    Eigen::Quaterniond GetQuaternion();
    Eigen::Vector3d    GetOrientation();
    Eigen::Vector3d    GetAngularVelocity();

private:

    //==========================================================================
    // P R I V A T E   M E M B E R S

    Eigen::Quaterniond quaternion_;
    Eigen::Vector3d    eulerAngle_;
    Eigen::Vector3d    angularVelocity_;
    Eigen::Vector3d    linearAcceleration_;
};

} // namespace proc_navigation

#endif //PROC_NAVIGATION_IMUDATA_HPP
