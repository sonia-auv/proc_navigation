//
// Created by jeremie on 9/14/16.
//

#ifndef PROC_NAVIGATION_DEPTHMETERDATA_HPP
#define PROC_NAVIGATION_DEPTHMETERDATA_HPP

#include <sensor_msgs/FluidPressure.h>
#include <proc_navigation/NavigationDevice.h>

class DepthMeterData : public NavigationDevice {
 public:
  DepthMeterData();

  void DepthMeterCallback(sensor_msgs::FluidPressure msg);
  void GetDepth(double &depth);
 private:
  double depth_m_;

};

inline void DepthMeterData::GetDepth(double &depth)
{
  depth = depth_m_;
}

#endif //PROC_NAVIGATION_DEPTHMETERDATA_HPP
