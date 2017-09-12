//
// Created by jeremie on 9/14/16.
//

#ifndef DVLDATA_H
#define DVLDATA_H

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <tf/tf.h>
#include "navigation_device.h"

namespace proc_navigation {

class DvlData: public NavigationDevice {
 public:
  //==========================================================================
  // C O N S T  ,  T Y P E D E F   A N D   E N U M

  static constexpr double barToMeterOfWater = 10.1972;

  //==========================================================================
  // P U B L I C   C / D T O R S

  DvlData();
  ~DvlData();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void DvlTwistCallback(geometry_msgs::TwistStamped msg);
  void DvlPressureCallback(sensor_msgs::FluidPressure msg);

  tf::Vector3 GetPositionXYZ();
  tf::Vector3 GetVelocityXYZ();
  double GetPositionZFromPressure();
  sensor_msgs::FluidPressure GetPressure();

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::Time last_timestamp_;
  geometry_msgs::TwistStamped dvl_twist_;
  sensor_msgs::FluidPressure dvl_pressure_;
};

}
#endif // DVLDATA_H
