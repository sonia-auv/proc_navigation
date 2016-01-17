//
// Created by etienne on 07/01/16.
//

#include "navigation_node.h"

namespace proc_navigation{

    NavNode::NavNode(ros::NodeHandle nh):
        node_handle_(){
        InitParameters();
        if (navigation_mode_ == 0){
            // Subscribe to AUV6 node, position topic
        }
        else if(navigation_mode_ == 1){
            subscriber_dvl_ = node_handle_.subscribe("data", 1000, dvlDataCallback);
           //publisher_ = node_handle_.advertise<provider_dvl::DVL>("data", 1000);
            // Subscribe to IMU and DVL
        }
    }
    NavNode::~NavNode() {}
    void NavNode::InitParameters() {
        node_handle_.param("mode", navigation_mode_, 0);
    }
    void NavNode::Spin() { }
    void NavNode::dvlDataCallback(const provider_dvl::DVL msg) {
        ROS_INFO("received dvl msg");//m
    }
}