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
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <proc_navigation/SetDepthOffset.h>
#include <proc_navigation/SetWorldXYOffset.h>

#include "proc_navigation/devices/dvl_data.h"
#include "proc_navigation/devices/imu_data.h"
#include "proc_navigation/ekf/extended_kalman_filter.h"

namespace proc_navigation {

    class ProcNavigationNode {
    public:

        static constexpr double DegreeToRad = M_PI / 180.0f;
        //==========================================================================
        // P U B L I C   C / D T O R S

        ProcNavigationNode(const ros::NodeHandlePtr &nh);
        ~ProcNavigationNode();

        //==========================================================================
        // P U B L I C   M E T H O D S

        void Spin();
        void ProcessCartesianPose();
        void PublishData();

    private:
        //==========================================================================
        // P R I V A T E   M E T H O D S

        bool SetDepthOffsetCallback(SetDepthOffset::Request &rqst, SetDepthOffset::Response &response);
        bool SetWorldXYOffsetCallback(SetWorldXYOffset::Request &rqst, SetWorldXYOffset::Response &response);

        void FillPoseMsg(Eigen::Vector3d &position, Eigen::Vector3d &angle, nav_msgs::Odometry &msg);
        void FillTwistMsg(Eigen::Vector3d &linear_velocity, Eigen::Vector3d &angular_velocity, nav_msgs::Odometry &msg);

        //==========================================================================
        // P R I V A T E   M E M B E R S

        ros::NodeHandlePtr nh_;

        ros::Subscriber dvlTwistSubscriber_;
        ros::Subscriber dvlPressureSubscriber_;
        ros::Subscriber imuSubscriber_;

        ros::Publisher  navigationOdomPublisher_;

        ros::ServiceServer navigationDepthOffsetServer_;
        ros::ServiceServer navigationXYOffsetServer_;



        DvlData dvlData_;
        IMUData imuData_;

        double zOffset_;
        double positionFromDepth_;

        Eigen::Quaterniond quaternion_;
        Eigen::Vector3d    position_;
        Eigen::Vector3d    incrementPosition_;
        Eigen::Vector3d    velocity_;
        Eigen::Vector3d    angularVelocity_;
        Eigen::Vector3d    eulerAngel_;

    };

}
#endif  // PROC_NAVIGATION_NAVIGATION_NODE_H_
