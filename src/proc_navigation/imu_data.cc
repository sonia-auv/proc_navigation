//
// Created by jeremie on 9/14/16.
//

#include <ros/ros.h>
#include "proc_navigation/imu_data.h"
#include <cmath>

namespace proc_navigation {

//-----------------------------------------------------------------------------
//
void IMUData::IMUMsgCallback(sensor_msgs::Imu msg) {
  quaternion_.w = msg.orientation.w;
  quaternion_.x = msg.orientation.x;
  quaternion_.y = msg.orientation.y;
  quaternion_.z = msg.orientation.z;

  // We use the IMU's data sheet transformation because Eigen return yaw on 0-180 basis,
  // which makes it impossible to use for us. This formulae gives yaw on 0-360 and independant from
  // roll pitch yaw
  double q0 = quaternion_.w, q1 = quaternion_.x, q2 = quaternion_.y, q3 = quaternion_.z;
  double m11 = 2 * (q0*q0 - 0.5 + q1*q1);
  double m12 = 2 * (q1*q2 + q0*q3);
  double m13 = 2 * (q1*q3 - q0*q2);

  //double m21 = 2 * (q1*q2 - q0*q3);
  //double m22 = 2 * (q0*q0 - 0.5 + q2*q2);
  double m23 = 2 * (q2*q3 + q0*q1);

  //double m31 = 2 * (q1*q3 + q0*q2);
  //double m32 = 2 * (q2*q3 - q0*q1);
  double m33 = 2 * (q0*q0 - 0.5 + q3*q3);

  double pitch = std::asin(-m13);
  double roll = std::atan2(m23,m33);
  double yaw = std::atan2(m12, m11) + M_PI;
  yaw = std::fmod((yaw + M_PI), M_PI*2.0);
  orientation_rpy_degree_.x = RadianToDegree(roll);
  orientation_rpy_degree_.y = RadianToDegree(pitch);
  orientation_rpy_degree_.z = RadianToDegree(yaw);


  // acceleration
  linear_acceleration = msg.linear_acceleration;

  // We want our data to be [roll, pitch, yaw] but atlas returns it 
  // in yaw pitch roll, so we inverse z and x
  angular_velocity_ = msg.angular_velocity;

  SetNewDataReady();
}

//-----------------------------------------------------------------------------
//
geometry_msgs::Quaternion IMUData::GetQuaternion() {
  return quaternion_;
}

geometry_msgs::Vector3 IMUData::GetOrientation() {
  return orientation_rpy_degree_;
}

//-----------------------------------------------------------------------------
//
geometry_msgs::Vector3 IMUData::GetAngularVelocity() {
  return angular_velocity_;
}

}
