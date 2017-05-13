//
// Created by jeremie on 9/14/16.
//

#ifndef PROC_NAVIGATION_IMUDATA_HPP
#define PROC_NAVIGATION_IMUDATA_HPP

#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Eigen>
#include "proc_navigation/navigation_device.h"
namespace proc_navigation {

class IMUData: public NavigationDevice {
 public:

  static constexpr double RadToDegree = 180.0f/M_PI;

  void IMUMsgCallback(sensor_msgs::Imu msg);
  void GetQuaternion(Eigen::Quaterniond &quat);
  void GetOrientation(Eigen::Vector3d &orientation_rpy_degree);
  double RadianToDegree(const double &radian);
 private:

  Eigen::Quaterniond quaternion;

  Eigen::Vector3d orientation_rpy_degree, linear_acceleration, angular_velocity;
};


inline void IMUData::GetQuaternion(Eigen::Quaterniond &quat) {
  quat = quaternion;
}

inline void IMUData::GetOrientation(Eigen::Vector3d &rpy) {
  rpy = orientation_rpy_degree;
}

inline double IMUData::RadianToDegree(const double &radian) {
  return radian * RadToDegree;
}

}
#endif //PROC_NAVIGATION_IMUDATA_HPP
