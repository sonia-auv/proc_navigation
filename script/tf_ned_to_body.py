#!/usr/bin/python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('proc_navigation')

import rospy
import tf
from nav_msgs.msg import Odometry


class NedToBody:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.tf_send_divider = 10
	self.tf_send_count = 0
	self.sub_odom = rospy.Subscriber("/proc_navigation/odom", Odometry,
                                         self.odom_callback)
	
    def odom_callback(self, msg):
	self.tf_send_count = self.tf_send_count + 1
	# Enable dividing by divider to reduce load on CPU since it is 
	# only for visualisation.
	if( self.tf_send_divider != self.tf_send_count ):
	    return
	else:
	    self.tf_send_count = 0
        p = (msg.pose.pose.position.x,
             msg.pose.pose.position.y,
             msg.pose.pose.position.z)

        q = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)
	
        self.br.sendTransform(p, q, rospy.Time.now(), "BODY", "NED")


if __name__ == '__main__':
    rospy.init_node('tf_ned_to_body')
    try:
        ned_to_body = NedToBody()
    	rospy.spin()
    except rospy.ROSInterruptException:
        pass
