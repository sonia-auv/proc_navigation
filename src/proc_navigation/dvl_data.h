//
// Created by jeremie on 9/14/16.
//

#ifndef DVLDATA_H
#define DVLDATA_H

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include "proc_navigation/navigation_device.h"

namespace proc_navigation {

class DvlData: public NavigationDevice {
 public:
  DvlData();
  ~DvlData();

  static constexpr double barToMeterOfWater = 10.1972;

  void DvlTwistCallback(geometry_msgs::TwistStamped msg);
  void DvlPressureCallback(sensor_msgs::FluidPressure msg);

  geometry_msgs::Vector3 GetPositionXYZ();
  double GetPositionZFromPressure();
  sensor_msgs::FluidPressure GetPressure();

 private:
  ros::Time last_timestamp_;
  geometry_msgs::TwistStamped dvl_twist_;
  sensor_msgs::FluidPressure dvl_pressure_;
};

}
#endif // DVLDATA_H
