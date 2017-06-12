//
// Created by jeremie on 9/14/16.
//

#ifndef PROC_NAVIGATION_IMUDATA_HPP
#define PROC_NAVIGATION_IMUDATA_HPP

#include <sensor_msgs/Imu.h>
#include "proc_navigation/navigation_device.h"
#include <eigen3/Eigen/Geometry>

namespace proc_navigation {

class IMUData: public NavigationDevice {
 public:
  //==========================================================================
  // C O N S T  ,  T Y P E D E F   A N D   E N U M

  static constexpr double RadToDegree = 180.0f/M_PI;

  //==========================================================================
  // P U B L I C   C / D T O R S

  IMUData();
  ~IMUData();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void IMUMsgCallback(sensor_msgs::Imu msg);
  Eigen::Quaterniond GetQuaternion();
  geometry_msgs::Vector3 GetOrientation();
  geometry_msgs::Vector3 GetAngularVelocity();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  double RadianToDegree(const double &radian);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  Eigen::Quaterniond quaternion_;
  geometry_msgs::Vector3 euler_angle_, angular_velocity_, linear_acceleration_;
};

inline double IMUData::RadianToDegree(const double &radian) {
  return radian * RadToDegree;
}

} // namespace proc_navigation

#endif //PROC_NAVIGATION_IMUDATA_HPP
