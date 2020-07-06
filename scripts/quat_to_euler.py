#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'proc_navigation'
import roslib

roslib.load_manifest(PKG)
import rospy
from math import atan2, asin, sqrt

# ROS messages.
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sonia_msgs.msg import Eulers


class QuatToEuler:
    """ Class that subscribe to different ROS topics
        and output euler angles messages
    """

    def __init__(self):
        # Create subscribers and publishers.
        self.sub_imu = rospy.Subscriber("/provider_imu/imu", Imu,
                                        self.imu_callback)
        self.sub_odom = rospy.Subscriber("/proc_navigation/odom", Odometry,
                                         self.odom_callback)
        self.pub_euler_odom = rospy.Publisher("/proc_navigation/euler_odom",
                                              Eulers, queue_size=100)
        self.pub_euler_imu = rospy.Publisher("/proc_navigation/euler_imu",
                                             Eulers, queue_size=100)

        # Main while loop.
        while not rospy.is_shutdown():
            continue

    # Odometry callback function.
    def odom_callback(self, msg):
        b = [msg.pose.pose.orientation.w,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z]

        e = self.quat_to_euler(b)
        euler_msg = self.quat_to_euler_msg(msg, e[0], e[1], e[2])

        self.pub_euler_imu.publish(euler_msg)

    # IMU callback function.
    def imu_callback(self, msg):
        b = [msg.orientation.w, msg.orientation.x, msg.orientation.y,
             msg.orientation.z]

        e = self.quat_to_euler(b)
        euler_msg = self.quat_to_euler_msg(msg, e[0], e[1], e[2])

        self.pub_euler_imu.publish(euler_msg)

    # Fill in Euler angle message.
    @staticmethod
    def quat_to_euler_msg(msg, r, p, y):
        euler_msg = Eulers()
        euler_msg.header.stamp = msg.header.stamp
        euler_msg.roll = r
        euler_msg.pitch = p
        euler_msg.yaw = y
        return euler_msg

    def normalize_quat(self, b):
        n = b[0] * b[0] + b[1] * b[1] + b[2] * b[2] + b[3] * b[3]
        if n == 1:
            return b
        normalized_b = b
        n = 1 / sqrt(n)
        normalized_b[0] = b[0] * n
        normalized_b[1] = b[1] * n
        normalized_b[2] = b[2] * n
        normalized_b[3] = b[3] * n
	return normalized_b

    def quat_to_euler(self, b):
        b = self.normalize_quat(b)
        e = [0] * 3
        asin_input = (-2 * (b[1] * b[3] - b[0] * b[2]))

        if asin_input > 1:
            asin_input = 1

        e[0] = atan2(2 * (b[2] * b[3] + b[0] * b[1]),
                     b[0] * b[0] - b[1] * b[1] -
                     b[2] * b[2] + b[3] * b[3])
        e[1] = asin(asin_input)
        e[2] = atan2(2 * (b[1] * b[2] + b[0] * b[3]),
                     b[0] * b[0] + b[1] * b[1] -
                     b[2] * b[2] - b[3] * b[3])
        return e


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException:
        pass
