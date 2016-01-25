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

#include "navigation_node.h"

namespace proc_navigation {

//-----------------------------------------------------------------------------
//
NavNode::NavNode(ros::NodeHandle nh) : node_handle_() {
  InitParameters();
  if (navigation_mode_ == 0) {
    // Subscribe to AUV6 node, position topic
  } else if (navigation_mode_ == 1) {
    subscriber_dvl_ = node_handle_.subscribe("data", 1000, &NavNode::dvlDataCallback, this);
    // publisher_ = node_handle_.advertise<provider_dvl::DVL>("data", 1000);
    // Subscribe to IMU and DVL
  }
}

//-----------------------------------------------------------------------------
//
NavNode::~NavNode() { }

//-----------------------------------------------------------------------------
//
void NavNode::InitParameters() {
  node_handle_.param("mode", navigation_mode_, 0);
}

//-----------------------------------------------------------------------------
//
void NavNode::Spin() { }

//-----------------------------------------------------------------------------
//
void NavNode::dvlDataCallback(const sonia_msgs::PD0Packet msg) {
  ROS_INFO("received dvl msg");  // m
}

} // namespace proc_navigation
