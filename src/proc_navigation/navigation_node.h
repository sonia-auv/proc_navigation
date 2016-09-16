/**
 * \file	navigation_node.h
 * \author	Etienne Boudreault-Pilon <etienne.b.pilon@gmail.com>
 * \date	24/01/2016
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

#ifndef PROC_NAVIGATION_NAVIGATION_NODE_H_
#define PROC_NAVIGATION_NAVIGATION_NODE_H_

#include <ros/ros.h>
#include <memory>
#include <geometry_msgs/Pose.h>
#include <sonia_msgs/PD0Packet.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "DVLData.h"
#include "IMUData.h"
#include "DepthMeterData.h"

namespace proc_navigation {

class NavNode {
 public:
  explicit NavNode(ros::NodeHandle nh);

  ~NavNode();

  void Spin();

  void PublishData();

 private:

  void FillTwistMsg(const Eigen::Vector3d &pos, const Eigen::Vector3d &euler, nav_msgs::Odometry &msg);
  void FillPoseMsg(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat, nav_msgs::Odometry &msg);
  ros::NodeHandle node_handle_;

  ros::Subscriber subscriber_dvl_;
  ros::Subscriber subscriber_imu_;
  ros::Subscriber subscriber_depth_meter_;

  ros::Publisher nav_pose_pub;

  DVLData dvl_data_;
  IMUData imu_data_;
  DepthMeterData depth_meter_data_;
};

inline void NavNode::FillTwistMsg(const Eigen::Vector3d &pos, const Eigen::Vector3d &euler, nav_msgs::Odometry &msg)
{
  auto &twist = msg.twist.twist;
  twist.linear.x = pos.x();
  twist.linear.y = pos.y();
  twist.linear.z = pos.z();
  twist.angular.x = euler.x();
  twist.angular.y = euler.y();
  twist.angular.z = euler.z();

}

inline void NavNode::FillPoseMsg(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat, nav_msgs::Odometry &msg)
{
  auto &pose = msg.pose.pose;
  pose.position.x = pos.x();
  pose.position.y = pos.y();
  pose.position.z = pos.z();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
}

}
#endif  // PROC_NAVIGATION_NAVIGATION_NODE_H_
