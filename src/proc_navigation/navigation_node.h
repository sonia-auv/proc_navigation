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
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <proc_navigation/SetDepthOffset.h>
#include <proc_navigation/SetWorldXYOffset.h>

#include "dvl_data.h"
#include "imu_data.h"

namespace proc_navigation {

class ProcNavigationNode {
 public:
  //==========================================================================
  // P U B L I C   C / D T O R S

  ProcNavigationNode(const ros::NodeHandlePtr &nh);
  ~ProcNavigationNode();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void Spin();
  void PublishData();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

//  bool SetDepthOffsetCallback(SetDepthOffset::Request &rqst, SetDepthOffset::Response &response);
//  bool SetWorldXYOffsetCallback(SetWorldXYOffset::Request &rqst, SetWorldXYOffset::Response &response);

  void FillTwistMsg(const geometry_msgs::Vector3 &vel, const geometry_msgs::Vector3 &euler, nav_msgs::Odometry &msg);
  void FillPoseMsg(const geometry_msgs::Vector3 &pos, const geometry_msgs::Vector3 &angular_velocity, nav_msgs::Odometry &msg);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;

  ros::Subscriber subscriber_dvl_twist_;
  ros::Subscriber subscriber_dvl_pressure_;
  ros::Subscriber subscriber_imu_;

  ros::Publisher nav_pose_pub;

  DvlData dvl_data_;
  IMUData imu_data_;

  geometry_msgs::Vector3 position_offset_;
  geometry_msgs::Vector3 position_;
};

//inline bool NavNode::SetDepthOffsetCallback(
//    SetDepthOffset::Request &rqst,
//    SetDepthOffset::Response &response)
//{
//  position_offset_.z = dvl_data_.GetPositionZFromPressure();
//  return true;
//}

//inline bool NavNode::SetWorldXYOffsetCallback(
//    SetWorldXYOffset::Request &rqst,
//    SetWorldXYOffset::Response &response)
//{
//  geometry_msgs::Vector3 tmp;
//  tmp = dvl_data_.GetPositionXYZ();
//  position_offset_.x = tmp.x;
//  position_offset_.y = tmp.y;
//  return true;
//}

}
#endif  // PROC_NAVIGATION_NAVIGATION_NODE_H_
