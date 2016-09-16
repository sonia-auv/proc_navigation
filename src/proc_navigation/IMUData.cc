//
// Created by jeremie on 9/14/16.
//

#include "proc_navigation/IMUData.h"
#include "lib_atlas/maths/matrix.h"

void IMUData::IMUMsgCallback(sensor_msgs::Imu msg)
{
  quaternion = Eigen::Quaterniond(msg.orientation.w,
                                     msg.orientation.x,
                                     msg.orientation.y,
                                     msg.orientation.z);

  // calculate euler angles for the orientation
  orientation_rpy_degree = atlas::QuatToEuler(quaternion);

  linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x,
                                        msg.linear_acceleration.y,
                                        msg.linear_acceleration.z);

  angular_velocity = Eigen::Vector3d(msg.angular_velocity.x,
                                     msg.angular_velocity.y,
                                     msg.angular_velocity.z);
  SetNewDataReady();
}