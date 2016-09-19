//
// Created by jeremie on 9/14/16.
//

#include "depth_meter_data.h"
namespace proc_navigation {

DepthMeterData::DepthMeterData() : depth_m_(false) {
}

void DepthMeterData::DepthMeterCallback(sensor_msgs::FluidPressure msg) {
  // Depth from message is in mm
  double surface_pressure = 101325;
  double ge = 9.80;
  double rho_water = 1000;
//  doublepressure.fluid_pressure = teleop.position[2] * (rho_water * ge) + surface_pressure
  depth_m_ = (msg.fluid_pressure - surface_pressure)/(rho_water * ge) / 1000.0f;
  SetNewDataReady();
}
}