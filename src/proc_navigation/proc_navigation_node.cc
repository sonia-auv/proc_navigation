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
	ProcNavigationNode::ProcNavigationNode(const ros::NodeHandlePtr &nh) : nh_(nh), quaternion_(0.0, 0.0, 0.0, 0.0)
    {
		dvlTwistSubscriber_ = nh_->subscribe("/provider_dvl/dvl_twist", 100,
												 &DvlData::DvlTwistCallback, &dvlData_);

		dvlPressureSubscriber_ = nh_->subscribe("/provider_dvl/dvl_pressure", 100,
												 &DvlData::DvlPressureCallback, &dvlData_);

		imuSubscriber_ = nh_->subscribe("/provider_imu/imu", 100,
												 &IMUData::IMUMsgCallback, &imuData_);

		navigationDepthOffsetServer_ = nh_->advertiseService("/proc_navigation/set_depth_offset",
									 &ProcNavigationNode::SetDepthOffsetCallback, this);

		navigationXYOffsetServer_ = nh_->advertiseService("/proc_navigation/set_world_x_y_offset",
										&ProcNavigationNode::SetWorldXYOffsetCallback, this);


		navigationOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("/proc_navigation/odom", 100);

	    position_          = Eigen::Vector3d::Zero();
	    incrementPosition_ = Eigen::Vector3d::Zero();
	    velocity_          = Eigen::Vector3d::Zero();
	    angularVelocity_   = Eigen::Vector3d::Zero();
	    eulerAngel_        = Eigen::Vector3d::Zero();

	}

	//-----------------------------------------------------------------------------
	//
	ProcNavigationNode::~ProcNavigationNode()
    {
        dvlTwistSubscriber_.shutdown();
        dvlPressureSubscriber_.shutdown();
        imuSubscriber_.shutdown();
        navigationDepthOffsetServer_.shutdown();
        navigationXYOffsetServer_.shutdown();
    }

	//==============================================================================
	// M E T H O D   S E C T I O N

	//-----------------------------------------------------------------------------
	//
	void ProcNavigationNode::Spin()
	{
		ros::Rate r(100); // 100 hz
		while(ros::ok())
        {
			ros::spinOnce();
            ProcessCartesianPose();
			r.sleep();
		}
	}

	bool ProcNavigationNode::SetDepthOffsetCallback(SetDepthOffset::Request &request,
													SetDepthOffset::Response &response)
	{
		zOffset_ = dvlData_.GetPositionZFromPressure();
		imuData_.SetNewDataReady();
		return true;
	}

	bool ProcNavigationNode::SetWorldXYOffsetCallback(SetWorldXYOffset::Request &request,
													  SetWorldXYOffset::Response &response)
	{
		position_.x() = 0.0f;
		position_.y() = 0.0f;
		dvlData_.SetNewDataReady();
		return true;
	}

	//-----------------------------------------------------------------------------
	//
	void ProcNavigationNode::ProcessCartesianPose()
	{
		if (dvlData_.IsNewDataReady() || imuData_.IsNewDataReady())
		{
		    dvlData_.SetNewDataUsed();
		    imuData_.SetNewDataUsed();

		    incrementPosition_ = dvlData_.GetPositionXYZ();
		    positionFromDepth_ = dvlData_.GetPositionZFromPressure();
		    velocity_          = dvlData_.GetVelocityXYZ();
		    angularVelocity_   = imuData_.GetAngularVelocity();
		    eulerAngel_        = imuData_.GetOrientation();
		    quaternion_        = imuData_.GetQuaternion();

		    position_ += quaternion_.toRotationMatrix() * incrementPosition_;

		    position_.z() = positionFromDepth_ - zOffset_;

            dvlFilter_.Update(position_, poseEstimation_);

            PublishData();
		}
	}
    //-----------------------------------------------------------------------------
    //
    void ProcNavigationNode::PublishData()
    {
        nav_msgs::Odometry odometry_msg;
        odometry_msg.header.frame_id = "NED";
        odometry_msg.header.stamp = ros::Time::now();

        FillPoseMsg(poseEstimation_, eulerAngel_, odometry_msg);
        FillTwistMsg(velocity_, angularVelocity_, odometry_msg);

        navigationOdomPublisher_.publish(odometry_msg);
    }

	//-----------------------------------------------------------------------------
	//
	void ProcNavigationNode::FillPoseMsg(Eigen::Vector3d &position, Eigen::Vector3d &angle, nav_msgs::Odometry &msg)
	{
		msg.pose.pose.position.x    = position.x();
		msg.pose.pose.position.y    = position.y();
		msg.pose.pose.position.z    = position.z();
		msg.pose.pose.orientation.x = angle.x();
		msg.pose.pose.orientation.y = angle.y();
		msg.pose.pose.orientation.z = angle.z();
	}

	//-----------------------------------------------------------------------------
	//
	void ProcNavigationNode::FillTwistMsg(Eigen::Vector3d &linear_velocity, Eigen::Vector3d &angular_velocity, nav_msgs::Odometry &msg)
	{
		msg.twist.twist.linear.x  = linear_velocity.x();
		msg.twist.twist.linear.y  = linear_velocity.y();
		msg.twist.twist.linear.z  = linear_velocity.z();
		msg.twist.twist.angular.x = angular_velocity.x();
		msg.twist.twist.angular.y = angular_velocity.y();
		msg.twist.twist.angular.z = angular_velocity.z();
	}

}  // namespace proc_navigation
