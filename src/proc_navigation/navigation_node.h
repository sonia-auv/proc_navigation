//
// Created by etienne on 06/01/16.
//

#ifndef PROJECT_NAVIGATION_NODE_H
#define PROJECT_NAVIGATION_NODE_H

#include <ros/ros.h>
#include <memory>
#include <provider_dvl/PD0Packet.h>

namespace proc_navigation{

class NavNode{
public:
    explicit NavNode(ros::NodeHandle nh);
    ~NavNode();

    void Spin();

private:
    void InitParameters();
    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_dvl_;
    void dvlDataCallback(const provider_dvl::PD0Packet msg);

    //-- Mode is temporary so we can switch from AUV6 interface to DVL/IMU interface
    //-- 0: AUV6
    //-- 1: IMU/DVL
    int navigation_mode_;
};

}

#endif //PROJECT_NAVIGATION_NODE_H
