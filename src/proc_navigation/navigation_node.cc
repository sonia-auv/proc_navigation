/**
 * \file	navigation.cc
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

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include "navigation_node.h"

namespace proc_navigation {

//-----------------------------------------------------------------------------
//
NavNode::NavNode(ros::NodeHandle nh) : node_handle_(nh) {
  InitParameters();
  if (navigation_mode_ == 0) {
    subscriber_auv6_ = node_handle_.subscribe("/auv6/pose", 100, &NavNode::auvDataCallback, this);
  } else if (navigation_mode_ == 1) {
    subscriber_dvl_ = node_handle_.subscribe("/provider_dvl/pd0_packet", 100, &NavNode::dvlDataCallback, this);
    subscriber_imu_ = node_handle_.subscribe("/provider_imu/imu", 1000, &NavNode::imuDataCallback, this);
  }
  nav_pose_pub = node_handle_.advertise<geometry_msgs::Pose>("/proc_navigation/pose", 100);

}

//-----------------------------------------------------------------------------
//
NavNode::~NavNode() { }

//-----------------------------------------------------------------------------
//
void NavNode::InitParameters() {
  node_handle_.param("mode", navigation_mode_, 1);
}

//-----------------------------------------------------------------------------
//
void NavNode::Spin() {

  while (!ros::isShuttingDown()) {
    if (Start() > 0) {
      while (node_handle_.ok()) {
        if (PublishData() < 0) break;
        ros::spinOnce();
      }
    } else {
      usleep(100000);
      ros::spinOnce();
    }
  }
}

//-----------------------------------------------------------------------------
//
void NavNode::dvlDataCallback(sonia_msgs::PD0Packet msg) {
    ROS_INFO("received DVL msg");
}
//-----------------------------------------------------------------------------
//
void NavNode::imuDataCallback(sensor_msgs::Imu msg) {
  ROS_INFO("received IMU msg");
}
//-----------------------------------------------------------------------------
//
void NavNode::auvDataCallback(geometry_msgs::Pose msg){
  ROS_INFO("received AUV6 msg");
  if (navigation_mode_ == 0) {
    pose_msg_ = msg;
  }
}
//-----------------------------------------------------------------------------
// return -1 on error, TODO: Change that logic, throw exception instead
int NavNode::Start() {
  return 1;
}
//-----------------------------------------------------------------------------
//
void NavNode::Stop() {

}

//-----------------------------------------------------------------------------
// return -1 on error, TODO: Change that logic
int NavNode::PublishData() {
  nav_pose_pub.publish(pose_msg_);
  return 1;
}

} // namespace proc_navigation
