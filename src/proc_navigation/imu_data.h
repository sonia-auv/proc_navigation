//
// Created by jeremie on 9/14/16.
//

#ifndef PROC_NAVIGATION_IMUDATA_HPP
#define PROC_NAVIGATION_IMUDATA_HPP

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include "proc_navigation/navigation_device.h"

namespace proc_navigation {

class IMUData: public NavigationDevice {
 public:

  static constexpr double RadToDegree = 180.0f/M_PI;

  void IMUMsgCallback(sensor_msgs::Imu msg);
  geometry_msgs::Quaternion GetQuaternion();
  void GetOrientation(geometry_msgs::Vector3 &orientation_rpy_degree);
  double RadianToDegree(const double &radian);
 private:

  geometry_msgs::Quaternion quaternion_;

  geometry_msgs::Vector3 orientation_rpy_degree, linear_acceleration, angular_velocity;
};

inline void IMUData::GetOrientation(geometry_msgs::Vector3 &rpy) {
  rpy = orientation_rpy_degree;
}

inline double IMUData::RadianToDegree(const double &radian) {
  return radian * RadToDegree;
}

}
#endif //PROC_NAVIGATION_IMUDATA_HPP
