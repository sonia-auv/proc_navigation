//
// Created by jeremie on 9/14/16.
//

#include <ros/ros.h>
#include "proc_navigation/dvl_data.h"

namespace proc_navigation {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
DvlData::DvlData() {
  last_timestamp = ros::Time::now();
}

//------------------------------------------------------------------------------
//
DvlData::~DvlData() { }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void DvlData::DvlTwistCallback(geometry_msgs::TwistStamped msg) {
  dvl_twist_ = msg;
  SetNewDataReady();
}

//------------------------------------------------------------------------------
//
void DvlData::DvlPressureCallback(sensor_msgs::FluidPressure msg) {
  dvl_pressure_ = msg;
  SetNewDataReady();
}

//------------------------------------------------------------------------------
//
geometry_msgs::Vector3 DvlData::GetPositionXYZ() {
  geometry_msgs::Vector3 position;
  ros::Duration dt;
  dt = dvl_twist_.header.stamp - last_timestamp;

  position.x = dvl_twist_.twist.linear.x * dt.sec;
  position.y = dvl_twist_.twist.linear.y * dt.sec;
  position.z = dvl_twist_.twist.linear.z * dt.sec;

  last_timestamp = dvl_twist_.header.stamp;

  return position;
}

//------------------------------------------------------------------------------
//
sensor_msgs::FluidPressure DvlData::GetPressure() {
  return dvl_pressure_;
}

//------------------------------------------------------------------------------
//
double DvlData::GetPositionZFromPressure() {
  return dvl_pressure_.fluid_pressure * barToMeterOfWater;
}

}
