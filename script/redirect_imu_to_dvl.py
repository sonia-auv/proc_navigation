#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'proc_navigation'
import roslib

roslib.load_manifest(PKG)
import rospy
from math import atan2, asin, sqrt
import os
import serial
import socket
import sys
import threading
import time
from optparse import (OptionParser)

# ROS messages.
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sonia_msgs.msg import Eulers


class ImuToDvl:
    """ Class that subscribe to different ROS topics
        and output euler angles messages
    """

    def __init__(self, serial):
        # Create subscribers and publishers.
        self.sub_imu = rospy.Subscriber("/provider_imu/imu", Imu,
                                        self.imu_callback)
        self.serial = serial

        # Main while loop.
        while not rospy.is_shutdown():
            continue

    # IMU callback function.
    def imu_callback(self, msg):
        b = [msg.orientation.w, msg.orientation.x, msg.orientation.y,
             msg.orientation.z]

        e = self.quat_to_euler(b)
        euler_msg = self.quat_to_euler_msg(msg, e[0], e[1], e[2])
        serial_msg = parse_and_send_to_serial(euler_msg)

        self.serial.write(serial_msg)

    def parse_and_send_to_serial(self, euler_msg):

        resultString = '$PTNTHPR,'
        resultString += '{:03.1f}'.format(euler_msg.yaw)
        resultString += ',N,'
        resultString += '{:03.1f}'.format(euler_msg.pitch)
        resultString += ',N,'
        resultString += '{:03.1f}'.format(euler_msg.roll)
        resultString += ',N*'
        resultString += getChecksum(resultString[1:resultString.len() - 1])
        resultString += '\r\n'
        return resultString

    @staticmethod
    def getChecksum(str):
        checksum = 0
        for c in str:
            checksum += checksum ^ c
        return '{:02X}'.format(checksum & 0xFF)

    # Fill in Euler angle message.
    @staticmethod
    def quat_to_euler_msg(msg, r, p, y):
        euler_msg = Eulers()
        euler_msg.header.stamp = msg.header.stamp

        if r > 180.0:
            r -= 360.0
        euler_msg.roll = r

        if p > 180.0:
            p -= 360.0
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


def initialize_options():
    global parser, options, args
    parser = PassThroughOptionParser(usage="""\
        %prog [options] [port [baudrate]]
        Simple Serial to Network (TCP/IP) redirector.

        Note: no security measures are implemeted. Anyone can remotely connect
        to this service over the network.
        Only one connection at once is supported. When the connection is terminated
        it waits for the next connect.
        """)
    parser.add_option("-p", "--port", dest="port",
                      help="port, a number (default 0) or a device name (deprecated option)",
                      default=None)
    parser.add_option("-b", "--baud", dest="baudrate", action="store",
                      type='int',
                      help="set baudrate, default 9600", default=9600)
    parser.add_option("", "--parity", dest="parity", action="store",
                      help="set parity, one of [N, E, O], default=N",
                      default='N')
    parser.add_option("", "--rtscts", dest="rtscts", action="store_true",
                      help="enable RTS/CTS flow control (default off)",
                      default=False)
    parser.add_option("", "--xonxoff", dest="xonxoff", action="store_true",
                      help="enable software flow control (default off)",
                      default=False)
    parser.add_option("", "--cr", dest="cr", action="store_true",
                      help="do not send CR+LF, send CR only", default=False)
    parser.add_option("", "--lf", dest="lf", action="store_true",
                      help="do not send CR+LF, send LF only", default=False)
    parser.add_option("", "--rts", dest="rts_state", action="store", type='int',
                      help="set initial RTS line state (possible values: 0, 1)",
                      default=None)
    parser.add_option("", "--dtr", dest="dtr_state", action="store", type='int',
                      help="set initial DTR line state (possible values: 0, 1)",
                      default=None)
    parser.add_option("-q", "--quiet", dest="quiet", action="store_true",
                      help="suppress non error messages", default=False)
    parser.add_option("-s", "--sniff", dest="sniff", action="store_true",
                      help="output read data to stdout", default=False)
    parser.add_option("-P", "--localport", dest="local_port", action="store",
                      type='int',
                      help="local TCP port", default=7777)
    parser.add_option("", "--file",
                      help="fake input by reading lines from FILE",
                      metavar="FILE", action="store",
                      type="string", dest="filename")
    parser.add_option("", "--freq",
                      help="frequency of line reading when --file is used",
                      action="store", type="float", dest="freq")
    # Parsing only known arguments
    (options, args) = parser.parse_args()


def assert_options_are_valid():
    global baudrate, port
    if args:
        if options.port is not None:
            parser.error(
                "no arguments are allowed, options only when --port is given")
        args.pop(0)
        if args:
            try:
                baudrate = int(args[0])
            except ValueError:
                parser.error("baudrate must be a number, not %r" % args[0])
            args.pop(0)
        if args:
            parser.error("too many arguments")
    else:
        if port is None:
            port = 0
    if options.cr and options.lf:
        parser.error("ony one of --cr or --lf can be specified")
    if options.filename or options.freq:
        if (not (options.filename)) or (not (options.freq)):
            parser.error("--file and --freq must always be used together")
        else:
            if options.freq <= 0.0:
                parser.error("Freq must be > 0 Hz")

            if not os.path.exists(options.filename):
                parser.error("File '%s' does not exist" % options.filename)


def init_serial_interface():
    try:
        ser.open()
    except serial.SerialException, e:
        print "Could not open serial port %s: %s" % (ser.portstr, e)
        sys.exit(1)
    if options.rts_state is not None:
        ser.setRTS(options.rts_state)
    if options.dtr_state is not None:
        ser.setDTR(options.dtr_state)


# Main function.
if __name__ == '__main__':

    # Init socket connection
    initialize_options()

    port = options.port
    baudrate = options.baudrate

    assert_options_are_valid()

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.rtscts = options.rtscts
    ser.xonxoff = options.xonxoff
    ser.timeout = 1  # required so that the reader thread can exit

    if not options.quiet:
        print "--- Serial redirector --- type Ctrl-C / BREAK to quit"
        print "--- %s %s,%s,%s,%s ---" % (
            ser.portstr, ser.baudrate, 8, ser.parity, 1)
        if options.filename:
            print "--- reading from %s at %.2f Hz ---" % (
                options.filename, options.freq)


    # Initialize the node and name it.
    rospy.init_node('redirect_imu_to_dvl')
    while not rospy.is_shutdown():
        try:
            init_serial_interface()
            sys.stdout.flush()
            quat_to_euler = ImuToDvl(ser)

            ser.close()

        except rospy.ROSInterruptException:
            break