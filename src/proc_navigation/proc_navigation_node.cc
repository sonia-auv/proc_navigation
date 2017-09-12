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


  navigation_odom_publisher_ = nh_->advertise<nav_msgs::Odometry>("/proc_navigation/odom", 100);

  position_.x() = 0.0;
  position_.y() = 0.0;
  position_.z() = 0.0;


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

    tf::Vector3 position = dvl_data_.GetPositionXYZ();
    tf::Vector3 velocity = dvl_data_.GetVelocityXYZ();
    tf::Vector3 angular_velocity = imu_data_.GetAngularVelocity();
    tf::Vector3 euler_angle = imu_data_.GetOrientation();
    tf::Quaternion quaternion = imu_data_.GetQuaternion();

    position.setZ(dvl_data_.GetPositionZFromPressure() - z_offset_);

    dvl_filter_.update_dvl(position, postion_estimation_);
    imu_filter_.update_imu(quaternion, orientation_estimation_);

    Eigen::Quaterniond quaterniond = Eigen::Quaterniond(quaternion.w(), quaternion.x(), quaternion.y(),
                                                        quaternion.z());

    Eigen::Affine3d transform;
    Eigen::Vector3d incrementPose(position.x(), position.y(), position.z());

    transform = quaterniond;

    position_ += transform * incrementPose;

    FillPoseMsg(position_, euler_angle, odometry_msg);
    FillTwistMsg(velocity, angular_velocity, odometry_msg);

    navigation_odom_publisher_.publish(odometry_msg);
  }
}

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::FillPoseMsg(Eigen::Vector3d position,
                                     tf::Vector3 angle,
                                     nav_msgs::Odometry &msg) {
  msg.pose.pose.position.x = position.x();
  msg.pose.pose.position.y = position.y();
  msg.pose.pose.position.z = position.z();
  msg.pose.pose.orientation.x = angle.x();
  msg.pose.pose.orientation.y = angle.y();
  msg.pose.pose.orientation.z = angle.z();
}

//-----------------------------------------------------------------------------
//
void ProcNavigationNode::FillTwistMsg(tf::Vector3 linear_velocity,
                                      tf::Vector3 angular_velocity,
                                      nav_msgs::Odometry &msg) {
  msg.twist.twist.linear.x = linear_velocity.x();
  msg.twist.twist.linear.y = linear_velocity.y();
  msg.twist.twist.linear.z = linear_velocity.z();
  msg.twist.twist.angular.x = angular_velocity.x();
  msg.twist.twist.angular.y = angular_velocity.y();
  msg.twist.twist.angular.z = angular_velocity.z();
}

tf::Vector3 ProcNavigationNode::PoseMsgToTf(const geometry_msgs::Vector3 position) {

    tf::Vector3 pose; pose.setValue(position.x,position.y,position.z);

    return pose;

}

tf::Quaternion ProcNavigationNode::QuaMsgToTf(const Eigen::Quaterniond quaternion) {

    tf::Quaternion qua;

    qua.setValue(quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());

    return qua;
}

}  // namespace proc_navigation
