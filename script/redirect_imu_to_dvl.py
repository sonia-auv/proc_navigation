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
from transforms3d import euler
from transforms3d import quaternions

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

        self.HMR3000_QUERY_HPR = "$PTNT,HPR*78"
        self.ACK_CMD = "#!0000*21\r\n"
        # Create subscribers and publishers.
        self.sub_imu = rospy.Subscriber("/provider_imu/imu", Imu,
                                        self.imu_callback)
        self.serial = serial

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def _readline(self):
        eol = b'\n'
        leneol = len(eol)
        line = bytearray()
        while True:
            c = self.serial.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return line

    def loop(self):
        readed_str = self._readline()
        send_byte_array = bytearray("")

        # Check if DVL has request a data, sends it
        if self.HMR3000_QUERY_HPR in readed_str:
	    send_byte_array = self.convert_rpy_to_hmr3000_format()
        else:
            # Else acknowledge everything
            send_byte_array = bytearray(self.ACK_CMD)

        self.serial.write(send_byte_array )

    def getChecksum(self, str_in):
	checksum = ord('\0')
        for c in str_in:
            checksum = checksum ^ ord(c)
	return '{:02X}'.format(checksum & 0xFF)

    def convert_rpy_to_hmr3000_format(self):
        resultString = '$PTNTHPR,'
        resultString += ("{:.1f}").format(self.yaw)
        resultString += ',N,'
        resultString += ("{:.1f}").format(self.pitch)
        resultString += ',N,'
        resultString += ("{:.1f}").format(self.roll)
        resultString += ',N*'
	resultString += self.getChecksum(resultString[1:len(resultString) - 1])
        resultString += '\r\n'
        #Debug
	#print resultString
	return bytearray(resultString)

    # IMU callback function.
    def imu_callback(self, msg):
	q = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
	euler_vec = euler.quat2euler(q, 'sxyz')
	
        self.roll = euler_vec[0] * 57.2958
        self.pitch = euler_vec[1] * 57.2958
        self.yaw = euler_vec[2] * 57.2958
	
	# Debug
	# print "RPY: " + str(self.roll) + " " + str(self.pitch) + " " + str(self.yaw)


class PassThroughOptionParser(OptionParser):
    def error(self, msg):
        pass


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
    try:
        init_serial_interface()
        print "Serial interface inited"
        sys.stdout.flush()
        redirector = ImuToDvl(ser)
        print "Redirector is up"
        while not rospy.is_shutdown():
            redirector.loop()
        print "Closing serial redirector"
	ser.close()

    except rospy.ROSInterruptException:
	print "ROS execption"
