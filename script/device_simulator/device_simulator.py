#!/usr/bin/env python

import numpy
import roslib
import math

roslib.load_manifest('proc_navigation')
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, FluidPressure
from sonia_msgs.msg import BottomTracking

import sys, select, termios, tty

from F130_keypad import *
import euler_math


# maximum speed 2000 mm/s
MAX_SPEED_LINEAR = 1000.0
# one 360 / 10 second
# rad/s
MAX_SPEED_ANGULAR =  2 * numpy.pi /10.0

# refresh rate 100 Hz
REFRESH_RATE_SECOND = 1.0/100.0


class Teleop:
    def __init__(self):
        self.keypad = F130_Keypad()
        self.position = [0,0,0]
        self.orientation = [0,0,0]

    def Update(self):
        # those are mutually exclusive... If you are not on right, you are on left,
        x_speed = float(self.keypad.states['LJ/Up'] + self.keypad.states['LJ/Down'])
        y_speed = float(self.keypad.states['LJ/Right'] + self.keypad.states['LJ/Left'])
        z_speed = float(self.keypad.states['RT'] - self.keypad.states['LT'])
        yaw_speed = float(self.keypad.states['RJ/Right'] + self.keypad.states['RJ/Left'])
        # In ENU
        self.position[1] = ((x_speed / 127.0) * MAX_SPEED_LINEAR)
        self.position[0] = ((y_speed / 127.0) * MAX_SPEED_LINEAR)
        self.position[2] += ((z_speed / 254.0) * MAX_SPEED_LINEAR) * REFRESH_RATE_SECOND

        self.orientation[2] += ((yaw_speed / 127.0) * MAX_SPEED_ANGULAR) * REFRESH_RATE_SECOND

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    dvl_pub = rospy.Publisher('/provider_dvl/bottom_tracking', BottomTracking, queue_size=100)
    imu_pub = rospy.Publisher('/provider_imu/imu', Imu, queue_size=100)
    baro_pub = rospy.Publisher('/provider_can/barometer_fluidpress_msgs', FluidPressure, queue_size=100)
    rospy.init_node('teleop_keyboard')
    teleop = Teleop()
    time_since_up = 0
    while True:
        time.sleep(REFRESH_RATE_SECOND)
        teleop.Update()
        time_since_up += REFRESH_RATE_SECOND

        bottom_tracking = BottomTracking()
        bottom_tracking.velocity[0] = teleop.position[0]
        bottom_tracking.velocity[1] = teleop.position[1]
        bottom_tracking.velocity[2] = 0
        bottom_tracking.time = time_since_up * 1000 *1000

        pressure = FluidPressure()
        # Saunder-Fofonoff equation
        surface_pressure = 101325
        ge = 9.80
        rho_water = 1000
        pressure.fluid_pressure = teleop.position[2] * (rho_water * ge) + surface_pressure

        imu = Imu()
        quaternion = euler_math.euler_to_quat(0, 0, teleop.orientation[2])
        imu.orientation.x = quaternion[0]
        imu.orientation.y = quaternion[1]
        imu.orientation.z = quaternion[2]
        imu.orientation.w = quaternion[3]

        imu_pub.publish(imu)
        dvl_pub.publish(bottom_tracking)
        baro_pub.publish(pressure)

        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        if key == '\x1b' or key == '\x03':
            exit(0)