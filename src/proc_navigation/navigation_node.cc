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

#include "proc_navigation/navigation_node.h"

namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

//-----------------------------------------------------------------------------
//
ProcNavigationNode::ProcNavigationNode(const ros::NodeHandlePtr &nh) : nh_(nh) {
  subscriber_dvl_twist_ = nh_->subscribe("/provider_dvl/dvl_twist", 100,
                                           &DvlData::DvlTwistCallback, &dvl_data_);
  subscriber_dvl_pressure_ = nh_->subscribe("/provider_dvl/dvl_pressure", 100,
                                           &DvlData::DvlPressureCallback, &dvl_data_);
  subscriber_imu_ = nh_->subscribe("/provider_imu/imu", 1000,
                                           &IMUData::IMUMsgCallback, &imu_data_);

//  RegisterService<SetDepthOffset>("/proc_navigation/set_depth_offset",
//                               &NavNode::SetDepthOffsetCallback, *this);
//  RegisterService<SetWorldXYOffset>("/proc_navigation/set_world_x_y_offset",
//                                  &NavNode::SetWorldXYOffsetCallback, *this);

  nav_pose_pub = nh_->advertise<nav_msgs::Odometry>("/proc_navigation/odom", 100);
}

//-----------------------------------------------------------------------------
//
ProcNavigationNode::~ProcNavigationNode() {}

//==============================================================================
// M E T H O D   S E C T I O N

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::Spin()
{
  ros::Rate r(100); // 100 hz
  while(ros::ok()) {
    ros::spinOnce();
    PublishData();
    r.sleep();
  }
}

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::PublishData() {
  if (dvl_data_.IsNewDataReady() ||
      imu_data_.IsNewDataReady()) {

    dvl_data_.SetNewDataUsed();
    imu_data_.SetNewDataUsed();

    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.frame_id = "NED";
    odometry_msg.header.stamp = ros::Time::now();

    // Fill the position
    geometry_msgs::Vector3 position;
    Eigen::Vector3d euler_angle;
    Eigen::Quaterniond quaternion;
    position = dvl_data_.GetPositionXYZ();
    imu_data_.GetQuaternion(quaternion);
    imu_data_.GetOrientation(euler_angle);

    // We use the depth from the depth meter.
//    position.z = depth;
    position.z -= position_offset_.z;

    FillPoseMsg(position, quaternion, odometry_msg);
    FillTwistMsg(position, euler_angle, odometry_msg);

    nav_pose_pub.publish(odometry_msg);
  }
}

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::FillTwistMsg(const geometry_msgs::Vector3 &pos,
                                      const Eigen::Vector3d &euler,
                                      nav_msgs::Odometry &msg) {

  msg.twist.twist.linear.x = pos.x;
  msg.twist.twist.linear.y = pos.y;
  msg.twist.twist.linear.z = pos.z;
  msg.twist.twist.angular.x = euler.x();
  msg.twist.twist.angular.y = euler.y();
  msg.twist.twist.angular.z = euler.z();
}

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::FillPoseMsg(const geometry_msgs::Vector3 &pos,
                                     const Eigen::Quaterniond &quat,
                                     nav_msgs::Odometry &msg) {
  msg.pose.pose.position.x = pos.x;
  msg.pose.pose.position.y = pos.y;
  msg.pose.pose.position.z = pos.z;
  msg.pose.pose.orientation.x = quat.x();
  msg.pose.pose.orientation.y = quat.y();
  msg.pose.pose.orientation.z = quat.z();
  msg.pose.pose.orientation.w = quat.w();
}

}  // namespace proc_navigation
