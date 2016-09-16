//
// Created by jeremie on 9/14/16.
//

#include "DepthMeterData.h"

DepthMeterData::DepthMeterData() : depth_m_(false)
{
}

void DepthMeterData::DepthMeterCallback(sensor_msgs::FluidPressure msg)
{
  // Depth from message is in mm
  depth_m_ = msg.fluid_pressure * 1000;
  SetNewDataReady();
}
