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

namespace proc_navigation {

class NavNode {
 public:
  explicit NavNode(ros::NodeHandle nh);

  ~NavNode();

  void Spin();

  int Start();
  void Stop();
  int PublishData();

 private:
  void dvlDataCallback(sonia_msgs::PD0Packet msg);
  void imuDataCallback(sensor_msgs::Imu msg);

  void InitParameters();

  ros::NodeHandle node_handle_;

  ros::Subscriber subscriber_dvl_;
  ros::Subscriber subscriber_imu_;
  ros::Subscriber subscriber_auv6_;

  ros::Publisher nav_pose_pub;

  //-- Mode is temporary so we can switch from AUV6 interface to DVL/IMU
  //interface
  //-- 0: AUV6
  //-- 1: IMU/DVL
  int navigation_mode_;
  // -- Contains both Attitude and Position
  geometry_msgs::Pose pose_msg_;
};

}

#endif  // PROC_NAVIGATION_NAVIGATION_NODE_H_
