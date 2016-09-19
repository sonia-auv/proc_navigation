//
// Created by jeremie on 9/14/16.
//

#ifndef PROC_NAVIGATION_NAVIGATIONDEVICE_H
#define PROC_NAVIGATION_NAVIGATIONDEVICE_H

#include <mutex>
namespace proc_navigation {

class NavigationDevice {

 public:
  NavigationDevice() : new_data_ready_(false) { };
  void SetNewDataReady();
  bool IsNewDataReady();
  void SetNewDataUsed();

 private:
  bool new_data_ready_;
  std::mutex data_ready_mutex_;
};

inline void NavigationDevice::SetNewDataReady() {
  std::lock_guard<std::mutex> guard(data_ready_mutex_);
  new_data_ready_ = true;
};
inline bool NavigationDevice::IsNewDataReady() {
  std::lock_guard<std::mutex> guard(data_ready_mutex_);
  return new_data_ready_;
};
inline void NavigationDevice::SetNewDataUsed() {
  std::lock_guard<std::mutex> guard(data_ready_mutex_);
  new_data_ready_ = false;
};

}
#endif //PROC_NAVIGATION_NAVIGATIONDEVICE_H
