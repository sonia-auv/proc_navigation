//
// Created by jeremie on 9/14/16.
//

#include <ros/ros.h>
#include "proc_navigation/imu_data.h"
#include <cmath>


namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
IMUData::IMUData() { }

//------------------------------------------------------------------------------
//
IMUData::~IMUData() { }

//==============================================================================
// M E T H O D   S E C T I O N

//-----------------------------------------------------------------------------
//
void IMUData::IMUMsgCallback(sensor_msgs::Imu msg) {
  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

//  quaternion_ = Eigen::Quaterniond(m);

  quaternion_ = Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);

  // We use the IMU's data sheet transformation because Eigen return yaw on 0-180 basis,
  // which makes it impossible to use for us. This formulae gives yaw on 0-360 and independant from
  // roll pitch yaw
  double q0 = quaternion_.w(), q1 = quaternion_.x(), q2 = quaternion_.y(), q3 = quaternion_.z();
  double m11 = 2 * (q0*q0 - 0.5 + q1*q1);
  double m12 = 2 * (q1*q2 + q0*q3);
  double m13 = 2 * (q1*q3 - q0*q2);

  double m23 = 2 * (q2*q3 + q0*q1);
  
  double m33 = 2 * (q0*q0 - 0.5 + q3*q3);

//  std::cout << m11 << "," << m12 << "," << m13 << "," << m21 << ","<< m22 << ","<< m23 << "," << m31 << ","<< m32 << ","<< m33 << std::endl;

  double pitch = std::asin(-m13);
  double roll = std::atan2(m23,m33);
  double yaw = std::atan2(m12, m11) + M_PI;

  yaw = std::fmod((yaw + M_PI), M_PI*2.0);

  euler_angle_.x = RadianToDegree(roll);
  euler_angle_.y = RadianToDegree(pitch);
  euler_angle_.z = RadianToDegree(yaw);

  linear_acceleration_ = msg.linear_acceleration;

  angular_velocity_ = msg.angular_velocity;

  SetNewDataReady();
}

//-----------------------------------------------------------------------------
//
Eigen::Quaterniond IMUData::GetQuaternion() {
  return quaternion_;
}

//-----------------------------------------------------------------------------
//
geometry_msgs::Vector3 IMUData::GetOrientation() {
  return euler_angle_;
}

//-----------------------------------------------------------------------------
//
geometry_msgs::Vector3 IMUData::GetAngularVelocity() {
  return angular_velocity_;
}

}
