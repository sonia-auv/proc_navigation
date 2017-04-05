//
// Created by jeremie on 9/14/16.
//

#ifndef PROC_NAVIGATION_DVLDATA_HPP
#define PROC_NAVIGATION_DVLDATA_HPP

#include <provider_dvl/BottomTracking.h>
#include <lib_atlas/maths/matrix.h>
#include "proc_navigation/navigation_device.h"
namespace proc_navigation {

class DVLData: public NavigationDevice {
 public:
  DVLData();
  void BottomTrackingCallback(provider_dvl::BottomTracking msg);

  void GetPositionXYZ(Eigen::Vector3d &pos);
 private:

  /*!
   * Returns true if the array has a NAN value in it.
   */
  bool VerifyNAN(const boost::array<double, 4> &data);

  double timestamp_us, last_timestamp_us;
  double velocity_east, velocity_north, velocity_up;
  // X, Y, Z where we use it as NED coordinate:
  //  X is North
  //  Y is East
  //  Z is Down
  Eigen::Vector3d position_xyz_m;
};


inline bool DVLData::VerifyNAN(const boost::array<double, 4> &data) {
  bool has_NAN = false;
  // Check for NAN values
  for (const auto &d : data) {
    // Using vel != vel should work, but compiler might optimise out if
    // the compiler option -ffast-math is on. For safety will use C++ standard
    // http://stackoverflow.com/questions/570669/checking-if-a-double-or-float-is-nan-in-c
    if (std::isnan(d)) {
      has_NAN = true;
    }
  }
  return has_NAN;
}

inline void DVLData::GetPositionXYZ(Eigen::Vector3d &pos) {
  pos = position_xyz_m;
}
}
#endif //PROC_NAVIGATION_DVLDATA_HPP
