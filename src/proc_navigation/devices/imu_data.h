//
// Created by jeremie on 9/14/16.
//

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
