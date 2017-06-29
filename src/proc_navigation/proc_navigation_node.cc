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

#include "proc_navigation/proc_navigation_node.h"

namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

//-----------------------------------------------------------------------------
//
ProcNavigationNode::ProcNavigationNode(const ros::NodeHandlePtr &nh) : nh_(nh) {
  dvl_twist_subscriber_ = nh_->subscribe("/provider_dvl/dvl_twist", 100,
                                           &DvlData::DvlTwistCallback, &dvl_data_);
  dvl_pressure_subscriber_ = nh_->subscribe("/provider_dvl/dvl_pressure", 100,
                                           &DvlData::DvlPressureCallback, &dvl_data_);
  imu_subscriber_ = nh_->subscribe("/provider_imu/imu", 100,
                                           &IMUData::IMUMsgCallback, &imu_data_);

  navigation_depth_offset_server_ = nh_->advertiseService("/proc_navigation/set_depth_offset",
                               &ProcNavigationNode::SetDepthOffsetCallback, this);
  navigation_xy_offset_server_ = nh_->advertiseService("/proc_navigation/set_world_x_y_offset",
                                  &ProcNavigationNode::SetWorldXYOffsetCallback, this);

  position_.x() = 0.0;
  position_.y() = 0.0;
  position_.z() = 0.0;

  navigation_odom_publisher_ = nh_->advertise<nav_msgs::Odometry>("/proc_navigation/odom", 100);
}

//-----------------------------------------------------------------------------
//
ProcNavigationNode::~ProcNavigationNode() { }

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

bool ProcNavigationNode::SetDepthOffsetCallback(
    SetDepthOffset::Request &rqst,
    SetDepthOffset::Response &response)
{
  z_offset_ = dvl_data_.GetPositionZFromPressure();

  imu_data_.SetNewDataReady();

  return true;
}

bool ProcNavigationNode::SetWorldXYOffsetCallback(
    SetWorldXYOffset::Request &rqst,
    SetWorldXYOffset::Response &response)
{
  position_.x() = 0.0f;
  position_.y() = 0.0f;

  imu_data_.SetNewDataReady();

  return true;
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

    geometry_msgs::Vector3 position = dvl_data_.GetPositionXYZ();
    double position_from_depth = dvl_data_.GetPositionZFromPressure();
    geometry_msgs::Vector3 velocity = dvl_data_.GetVelocityXYZ();
    geometry_msgs::Vector3 angular_velocity = imu_data_.GetAngularVelocity();
    geometry_msgs::Vector3 euler_angle = imu_data_.GetOrientation();
    Eigen::Quaterniond quaternion = imu_data_.GetQuaternion();

    Eigen::Affine3d transform;
    Eigen::Vector3d incrementPose(position.x, position.y, position.z);

    transform = quaternion;

    position_ += transform * incrementPose;

    position_.z() = position_from_depth - z_offset_;

//    if (fabs(position_from_depth - position_.z) > 0.1) {
//      position_.z = (position_from_depth + position_.z)/2;
//    }

    FillPoseMsg(position_, euler_angle, odometry_msg);
    FillTwistMsg(velocity, angular_velocity, odometry_msg);

    odometry_msg.pose.covariance = {-1, 0, 0, 0, 0, 0,
                                    0, -1, 0, 0, 0, 0,
                                    0, 0, -1, 0, 0, 0,
                                    0, 0, 0, 99999, 0, 0,
                                    0, 0, 0, 0, 99999, 0,
                                    0, 0, 0, 0, 0, 99999};

    odometry_msg.twist.covariance = {-1, 0, 0, 0, 0, 0,
                                    0, -1, 0, 0, 0, 0,
                                    0, 0, -1, 0, 0, 0,
                                    0, 0, 0, 99999, 0, 0,
                                    0, 0, 0, 0, 99999, 0,
                                    0, 0, 0, 0, 0, 99999};

    navigation_odom_publisher_.publish(odometry_msg);
  }
}

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::FillPoseMsg(Eigen::Vector3d position,
                                     geometry_msgs::Vector3 angle,
                                     nav_msgs::Odometry &msg) {
  msg.pose.pose.position.x = position.x();
  msg.pose.pose.position.y = position.y();
  msg.pose.pose.position.z = position.z();
  msg.pose.pose.orientation.x = angle.x;
  msg.pose.pose.orientation.y = angle.y;
  msg.pose.pose.orientation.z = angle.z;
}

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::FillTwistMsg(geometry_msgs::Vector3 linear_velocity,
                                      geometry_msgs::Vector3 angular_velocity,
                                      nav_msgs::Odometry &msg) {
  msg.twist.twist.linear.x = linear_velocity.x;
  msg.twist.twist.linear.y = linear_velocity.y;
  msg.twist.twist.linear.z = linear_velocity.z;
  msg.twist.twist.angular.x = angular_velocity.x;
  msg.twist.twist.angular.y = angular_velocity.y;
  msg.twist.twist.angular.z = angular_velocity.z;
}


Eigen::Matrix3d ProcNavigationNode::EulerToRot(const Eigen::Vector3d &vec) {
  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd(vec.x(), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(vec.y(), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(vec.z(), Eigen::Vector3d::UnitX());
  return m;
}

}  // namespace proc_navigation
