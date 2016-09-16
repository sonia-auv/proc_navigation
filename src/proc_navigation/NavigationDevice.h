//
// Created by jeremie on 9/14/16.
//

#ifndef PROC_NAVIGATION_NAVIGATIONDEVICE_H
#define PROC_NAVIGATION_NAVIGATIONDEVICE_H


class NavigationDevice {

 public:
  NavigationDevice():new_data_ready_(false){};
  void SetNewDataReady(){new_data_ready_ = true;};
  bool IsNewDataReady(){return new_data_ready_;};
  void SetNewDataUsed(){new_data_ready_ = false;};

 private:
  bool new_data_ready_;
};


#endif //PROC_NAVIGATION_NAVIGATIONDEVICE_H
