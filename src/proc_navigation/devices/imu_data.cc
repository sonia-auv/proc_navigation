//
// Created by jeremie on 9/14/16.
//

#include <ros/ros.h>
#include "imu_data.h"
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

  quaternion_.setValue(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

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

  euler_angle_.setValue(RadianToDegree(roll), RadianToDegree(pitch), RadianToDegree(yaw));

  linear_acceleration_.setValue(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

  angular_velocity_.setValue(msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z);

  SetNewDataReady();
}

//-----------------------------------------------------------------------------
//
tf::Quaternion IMUData::GetQuaternion() {
  return quaternion_;
}

//-----------------------------------------------------------------------------
//
tf::Vector3 IMUData::GetOrientation() {
  return euler_angle_;
}

//-----------------------------------------------------------------------------
//
tf::Vector3 IMUData::GetAngularVelocity() {

  return angular_velocity_;
}

}
