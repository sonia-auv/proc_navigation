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
  last_timestamp_ = ros::Time::now();
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
  ros::Duration dt = ros::Time::now() - last_timestamp_;
  double dt_sec = dt.toSec();

  position.x = dvl_twist_.twist.linear.x * dt_sec;
  position.y = dvl_twist_.twist.linear.y * dt_sec;
  position.z = dvl_twist_.twist.linear.z * dt_sec;

  last_timestamp_ = ros::Time::now();

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
