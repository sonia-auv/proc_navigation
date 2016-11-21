//
// Created by jeremie on 9/14/16.
//

#include <ros/ros.h>
#include <cmath>
#include "proc_navigation/dvl_data.h"

namespace proc_navigation {


DVLData::DVLData() :
    timestamp_us(-1),
    last_timestamp_us(-1),
    velocity_east(DBL_MAX),
    velocity_north(DBL_MAX),
    velocity_up(DBL_MAX) {
  position_xyz_m.fill(0.0f);
}


void DVLData::BottomTrackingCallback(sonia_msgs::BottomTracking msg) {
  // If no NAN, set the values
  if (!VerifyNAN(msg.velocity)) {
    // See p.195, search in page for: BT Velocity
    // Receive data as ENU
    velocity_north = msg.velocity[0];
    velocity_east = msg.velocity[1];
    velocity_up = msg.velocity[2];
    // For integration of speed.
    if (last_timestamp_us != -1) {
      last_timestamp_us = timestamp_us;
      timestamp_us = msg.time;
    } else {
      last_timestamp_us = msg.time;
      timestamp_us = msg.time;
    }

    // speed is in mm/second
    // convert time to second
    double time_delta_s = (timestamp_us - last_timestamp_us) / 1000000.0f;

    // Convert velocities to meter on the fly
    // Convert from ENU to NED
    position_xyz_m[0] += (velocity_north) * time_delta_s;
    position_xyz_m[1] += (velocity_east) * time_delta_s;
    position_xyz_m[2] += (-velocity_up) * time_delta_s;
  } else {
//    ROS_INFO_STREAM(
//        " DVL - Bottom tracking - Received NAN on velocity.\n" << msg);
  }

  SetNewDataReady();
}

}
