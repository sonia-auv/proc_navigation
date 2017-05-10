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
#include "proc_navigation/navigation_node.h"

namespace proc_navigation {

//-----------------------------------------------------------------------------
//
NavNode::NavNode(ros::NodeHandle nh) :
    node_handle_(nh),
    position_offset_(0,0,0)
{
//  subscriber_dvl_ = node_handle_.subscribe("/provider_dvl/bottom_tracking", 100,
//                                           &DVLData::BottomTrackingCallback, &dvl_data_);
  subscriber_imu_ = node_handle_.subscribe("/provider_imu/imu", 1000,
                                           &IMUData::IMUMsgCallback, &imu_data_);
  subscriber_depth_meter_ = node_handle_.subscribe("/provider_can/barometer_intern_press_msgs", 1000,
                                                   &DepthMeterData::DepthMeterCallback,
                                                   &depth_meter_data_);

  RegisterService<SetDepthOffset>("/proc_navigation/set_depth_offset",
                               &NavNode::SetDepthOffsetCallback, *this);
  RegisterService<SetWorldXYOffset>("/proc_navigation/set_world_x_y_offset",
                                  &NavNode::SetWorldXYOffsetCallback, *this);

  nav_pose_pub =
      node_handle_.advertise<nav_msgs::Odometry>("/proc_navigation/odom", 100);
}

//-----------------------------------------------------------------------------
//
NavNode::~NavNode() {}

//-----------------------------------------------------------------------------
//
void NavNode::Spin()
{
  ros::Rate r(100); // 100 hz
  while(ros::ok())
  {
    PublishData();
    ros::spinOnce();
    r.sleep();
  }
}

//-----------------------------------------------------------------------------
//
void NavNode::PublishData() {
  if (dvl_data_.IsNewDataReady() ||
      imu_data_.IsNewDataReady() ||
      depth_meter_data_.IsNewDataReady()) {

    dvl_data_.SetNewDataUsed();
    imu_data_.SetNewDataUsed();
    depth_meter_data_.SetNewDataUsed();

    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.frame_id = "NED";
    odometry_msg.header.stamp = ros::Time::now();

    // Fill the position
    Eigen::Vector3d position, euler_angle;
    Eigen::Quaterniond quaternion;
    double depth;
    dvl_data_.GetPositionXYZ(position);
    imu_data_.GetQuaternion(quaternion);
    imu_data_.GetOrientation(euler_angle);
    depth = depth_meter_data_.GetDepth();

    // We use the depth from the depth meter.
    position.z() = depth ;
    position -= position_offset_;

    FillPoseMsg(position, quaternion, odometry_msg);
    FillTwistMsg(position, euler_angle, odometry_msg);

    nav_pose_pub.publish(odometry_msg);
  }
}

}  // namespace proc_navigation
