//
// Created by jeremie on 9/14/16.
//

#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "proc_navigation/imu_data.h"
#include <cmath>
namespace proc_navigation {

void IMUData::IMUMsgCallback(sensor_msgs::Imu msg) {
  quaternion = Eigen::Quaterniond(msg.orientation.w,
                                  msg.orientation.x,
                                  msg.orientation.y,
                                  msg.orientation.z);

  // We use the IMU's data sheet transformation because Eigen return yaw on 0-180 basis,
  // which makes it impossible to use for us. This formulae gives yaw on 0-360 and independant from
  // roll pitch yaw
  double q0 = quaternion.w(), q1 = quaternion.x(), q2 = quaternion.y(), q3 = quaternion.z();
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
  orientation_rpy_degree = Eigen::Vector3d(RadianToDegree(roll),
                                           RadianToDegree(pitch),
                                           RadianToDegree(yaw) );


  // acceleration
  linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x,
                                        msg.linear_acceleration.y,
                                        msg.linear_acceleration.z);

  // We want our data to be [roll, pitch, yaw] but atlas returns it 
  // in yaw pitch roll, so we inverse z and x
  angular_velocity = Eigen::Vector3d(msg.angular_velocity.z,
                                     msg.angular_velocity.y,
                                     msg.angular_velocity.x);


  SetNewDataReady();
}
}
