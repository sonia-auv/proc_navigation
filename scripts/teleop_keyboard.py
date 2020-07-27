#!/usr/bin/env python

import numpy
import roslib
import math

roslib.load_manifest("proc_navigation")
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, FluidPressure
from sonia_common.msg import BottomTracking

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w
   a    s    d

Rotation:
---------------------------
CCW: q
CQ:  e

Depth:
---------------------------
r : up (+z)
f : down (-z)

CTRL-C to quit
"""

move_bindings = {
    "w": (0.5, 0, 0, 0),
    "s": (-0.5, 0, 0, 0),
    "a": (0, 0.5, 0, 0),
    "d": (0, -0.5, 0, 0),
    "r": (0, 0, -0.5, 0),
    "f": (0, 0, 0.5, 0),
    "q": (0, 0, 0, 0.5),
    "e": (0, 0, 0, -0.5),
}


def euler_to_quat(ai, aj, ak):
    """Return quaternion from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> q = euler_to_quat(1, 2, 3)
    >>> numpy.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
    True

    """
    firstaxis, parity, repetition, frame = (0, 0, 0, 0)
    _NEXT_AXIS = [1, 2, 0, 1]

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    quaternion = numpy.empty((4,), dtype=numpy.float64)
    if repetition:
        quaternion[i] = cj * (cs + sc)
        quaternion[j] = sj * (cc + ss)
        quaternion[k] = sj * (cs - sc)
        quaternion[3] = cj * (cc - ss)
    else:
        quaternion[i] = cj * sc - sj * cs
        quaternion[j] = cj * ss + sj * cc
        quaternion[k] = cj * cs - sj * sc
        quaternion[3] = cj * cc + sj * ss
    if parity:
        quaternion[j] *= -1

    return quaternion


def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    if key == "\x1b" or key == "\x03":
        exit(0)
    return key


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def get_odom_from_key():
    global th, status, x, y, z
    key = get_key()
    if key in move_bindings.keys():
        x += move_bindings[key][0]
        y += move_bindings[key][1]
        z += move_bindings[key][2]
        z = max(z, 0)

        th += move_bindings[key][3]
        th = clamp(th, -math.pi, math.pi)
    else:
        x = 0
        y = 0
        z = 0
        th = 0


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    dvl_pub = rospy.Publisher(
        "/provider_dvl/bottom_tracking", BottomTracking, queue_size=100
    )
    imu_pub = rospy.Publisher("/provider_imu/imu", Imu, queue_size=100)
    baro_pub = rospy.Publisher(
        "/provider_can/barometer_fluidpress_msgs", FluidPressure, queue_size=100
    )
    rospy.init_node("teleop_keyboard")

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    current_odometry = Odometry()
    current_key = None

    print msg
    while True:
        get_odom_from_key()

        bottom_tracking = BottomTracking()
        bottom_tracking.velocity[0] = x
        bottom_tracking.velocity[1] = y
        bottom_tracking.velocity[2] = 0

        pressure = FluidPressure()
        # Saunder-Fofonoff equation
        surface_pressure = 101325
        ge = 9.80
        rho_water = 1000
        pressure.fluid_pressure = z * (rho_water * ge) + surface_pressure

        imu = Imu()
        quaternion = euler_to_quat(0, 0, th)
        imu.orientation.x = quaternion[0]
        imu.orientation.y = quaternion[1]
        imu.orientation.z = quaternion[2]
        imu.orientation.w = quaternion[3]

        imu_pub.publish(imu)
        dvl_pub.publish(bottom_tracking)
        baro_pub.publish(pressure)
