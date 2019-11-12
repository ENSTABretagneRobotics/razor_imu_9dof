#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import serial
import string
import math
import sys
from time import sleep
from lib.serial_commands import *
import yaml
import time

#from time import time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
    #if imu_yaw_calibration != config('yaw_calibration'):
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config

def send_command(serial_instance, command, value = None):
    if value is None:
        cmd = command + chr(13)
    else:
        cmd = command + str(value) + chr(13)
    rospy.loginfo("Razor IMU -> Sending: %s", cmd)
    expected_len = len(cmd)
    res = serial_instance.write(cmd)
    if (res != expected_len):
        rospy.logerr("Razor IMU -> Expected serial command len (%d) didn't match amount of bytes written (%d) for command %s", expected_len, res, command)
    sleep(0.05) # Don't spam serial with too many commands at once

def write_and_check_config(serial_instance, calib_dict):
    send_command(serial_instance, SET_CALIB_ACC_X_MIN, calib_dict["accel_x_min"])
    send_command(serial_instance, SET_CALIB_ACC_X_MAX, calib_dict["accel_x_max"])
    send_command(serial_instance, SET_CALIB_ACC_Y_MIN, calib_dict["accel_y_min"])
    send_command(serial_instance, SET_CALIB_ACC_Y_MAX, calib_dict["accel_y_max"])
    send_command(serial_instance, SET_CALIB_ACC_Z_MIN, calib_dict["accel_z_min"])
    send_command(serial_instance, SET_CALIB_ACC_Z_MAX, calib_dict["accel_z_max"])

    if not calib_dict["magn_use_extended"]:
        send_command(serial_instance, SET_MAG_X_MIN, calib_dict["magn_x_min"])
        send_command(serial_instance, SET_MAG_X_MAX, calib_dict["magn_x_max"])
        send_command(serial_instance, SET_MAG_Y_MIN, calib_dict["magn_y_min"])
        send_command(serial_instance, SET_MAG_Y_MAX, calib_dict["magn_y_max"])
        send_command(serial_instance, SET_MAG_Z_MIN, calib_dict["magn_z_min"])
        send_command(serial_instance, SET_MAG_Z_MAX, calib_dict["magn_z_max"])
    else:
        send_command(serial_instance, SET_MAG_ELLIPSOID_CENTER_0, calib_dict["magn_ellipsoid_center"][0])
        send_command(serial_instance, SET_MAG_ELLIPSOID_CENTER_1, calib_dict["magn_ellipsoid_center"][1])
        send_command(serial_instance, SET_MAG_ELLIPSOID_CENTER_2, calib_dict["magn_ellipsoid_center"][2])
        send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_0_0, calib_dict["magn_ellipsoid_transform"][0][0])
        send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_0_1, calib_dict["magn_ellipsoid_transform"][0][1])
        send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_0_2, calib_dict["magn_ellipsoid_transform"][0][2])
        send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_1_0, calib_dict["magn_ellipsoid_transform"][1][0])
        send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_1_1, calib_dict["magn_ellipsoid_transform"][1][1])
        send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_1_2, calib_dict["magn_ellipsoid_transform"][1][2])
        send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_2_0, calib_dict["magn_ellipsoid_transform"][2][0])
        send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_2_1, calib_dict["magn_ellipsoid_transform"][2][1])
        send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_2_2, calib_dict["magn_ellipsoid_transform"][2][2])

    send_command(serial_instance, SET_GYRO_AVERAGE_OFFSET_X, calib_dict["gyro_average_offset_x"])
    send_command(serial_instance, SET_GYRO_AVERAGE_OFFSET_Y, calib_dict["gyro_average_offset_y"])
    send_command(serial_instance, SET_GYRO_AVERAGE_OFFSET_Z, calib_dict["gyro_average_offset_z"])

    send_command(serial_instance, GET_CALIBRATION_VALUES)
    config = ""
    for _ in range(0, 21):
        # Format each line received from serial into proper yaml with lowercase variable names
        config += serial_instance.readline().lower().replace(":", ": ")
        # TODO: round the numbers, otherwise we will get false negatives in the check phase

    config_parsed = yaml.load(config)
    for key in calib_dict:
        if config_parsed[key] != calib_dict[key]:
            rospy.logwarn("The calibration value of [%s] did not match. Expected: %s, received: %s",
                          key, str(calib_dict[key]), str(config_parsed[key]))

rospy.init_node("razor_imu")
#We only care about the most recent measurement, i.e. queue_size=1
pub_imu = rospy.Publisher('imu', Imu, queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time()

imuMsg = Imu()
imuMsg.orientation_covariance = rospy.get_param('~orientation_covariance')
imuMsg.angular_velocity_covariance = rospy.get_param('~velocity_covariance')
imuMsg.linear_acceleration_covariance = rospy.get_param('~acceleration_covariance')
imuMsg.header.frame_id = rospy.get_param('~frame_header', 'base_imu_link')

publish_magnetometer = rospy.get_param('~publish_magnetometer', False)

if publish_magnetometer:
    pub_mag = rospy.Publisher('mag', MagneticField, queue_size=1)
    magMsg = MagneticField()
    magMsg.magnetic_field_covariance = rospy.get_param('~magnetic_field_covariance')
    magMsg.header.frame_id = rospy.get_param('~frame_header', 'base_imu_link')
    #should a separate diagnostic for the Magnetometer should be done?

default_port='/dev/ttyUSB0'
port = rospy.get_param('~port', default_port)

#read calibration parameters

calib_dict = {}

#accelerometer
calib_dict["accel_x_min"] = rospy.get_param('~accel_x_min', -250.0)
calib_dict["accel_x_max"] = rospy.get_param('~accel_x_max', 250.0)
calib_dict["accel_y_min"] = rospy.get_param('~accel_y_min', -250.0)
calib_dict["accel_y_max"] = rospy.get_param('~accel_y_max', 250.0)
calib_dict["accel_z_min"] = rospy.get_param('~accel_z_min', -250.0)
calib_dict["accel_z_max"] = rospy.get_param('~accel_z_max', 250.0)

# magnetometer
calib_dict["magn_x_min"] = rospy.get_param('~magn_x_min', -600.0)
calib_dict["magn_x_max"] = rospy.get_param('~magn_x_max', 600.0)
calib_dict["magn_y_min"] = rospy.get_param('~magn_y_min', -600.0)
calib_dict["magn_y_max"] = rospy.get_param('~magn_y_max', 600.0)
calib_dict["magn_z_min"] = rospy.get_param('~magn_z_min', -600.0)
calib_dict["magn_z_max"] = rospy.get_param('~magn_z_max', 600.0)
calib_dict["magn_use_extended"] = rospy.get_param('~calibration_magn_use_extended', False)
calib_dict["magn_ellipsoid_center"] = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
calib_dict["magn_ellipsoid_transform"] = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])

# gyroscope
calib_dict["gyro_average_offset_x"] = rospy.get_param('~gyro_average_offset_x', 0.0)
calib_dict["gyro_average_offset_y"] = rospy.get_param('~gyro_average_offset_y', 0.0)
calib_dict["gyro_average_offset_z"] = rospy.get_param('~gyro_average_offset_z', 0.0)

imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# Check your COM port and baud rate
rospy.loginfo("Razor IMU -> Opening %s...", port)
connection_attempts = 5
for connection_tries in range (0,connection_attempts + 1):
    try:
        ser = serial.Serial(port=port, baudrate=57600, timeout=1)
        break
    except serial.serialutil.SerialException:
        rospy.logerr("Razor IMU not found at port " + port + ". Did you specify the correct port in the launch file? Trying %d more times...", 5 -  connection_tries)
        
    if connection_tries == connection_attempts:
        #exit
        sys.exit(0)
    time.sleep(3)

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.


### configure board ###
#stop datastream
send_command(ser, STOP_DATASTREAM)
write_and_check_config(ser, calib_dict)

if publish_magnetometer:
    send_command(ser, SET_TEXT_EXTENDED_FORMAT_WITH_MAG)
    line_start = LINE_START_WITH_MAG
else:
    send_command(ser, SET_TEXT_EXTENDED_FORMAT_NO_MAG)
    line_start = LINE_START_NO_MAG

send_command(ser, START_DATASTREAM)

rospy.loginfo("Razor IMU up and running")

while not rospy.is_shutdown():
    line = ser.readline()
    if not line.startswith(line_start):
        rospy.logerr_throttle(1, "Did not find correct line start in the received IMU message")
        continue
    line = line.replace(line_start,"")   # Delete "#YPRAG=" or "#YPRAGM="
    words = string.split(line,",")    # Fields split
    if len(words) > 2:
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        yaw_deg = -float(words[0])
        yaw_deg = yaw_deg + imu_yaw_calibration
        if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0
        yaw = yaw_deg*degrees2rad
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        pitch = -float(words[1])*degrees2rad
        roll = float(words[2])*degrees2rad

        # Publish message
        # AHRS firmware accelerations are negated
        # This means y and z are correct for ROS, but x needs reversing
        imuMsg.linear_acceleration.x = -float(words[3]) * accel_factor
        imuMsg.linear_acceleration.y = float(words[4]) * accel_factor
        imuMsg.linear_acceleration.z = float(words[5]) * accel_factor

        imuMsg.angular_velocity.x = float(words[6])
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        imuMsg.angular_velocity.y = -float(words[7])
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        imuMsg.angular_velocity.z = -float(words[8])

        if publish_magnetometer:
            #according to line 178 the units published of the mag are mGauss
            #REP103 specifys the units of magnetic field to be Teslas
            #The axis of the MPU magnetometer are X forward, Y right and Z down
            #  when the chip is facing forward, in the sparkfun board, the chip is facing the left side
            # but Sparkfun the firmware interchanges x and y and changes the sign of y
            # so to get it to REP103 we need to make X and Z negative
            #The magn frames form sparkfun can be seen in line 178 from Sensors.ino
            magMsg.magnetic_field.x = float(words[9]) * 1e-7
            magMsg.magnetic_field.y = -float(words[10]) * 1e-7
            magMsg.magnetic_field.z = -float(words[11]) * 1e-7
            #check frame orientation and units

    q = quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.seq = seq
    seq = seq + 1
    pub_imu.publish(imuMsg)

    if publish_magnetometer:
        magMsg.header.stamp = imuMsg.header.stamp
        magMsg.header.seq = imuMsg.header.seq
        pub_mag.publish(magMsg)

    if (diag_pub_time < rospy.get_time()) :
        diag_pub_time += 1
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = rospy.get_rostime()
        diag_arr.header.frame_id = '1'
        diag_msg = DiagnosticStatus()
        diag_msg.name = 'Razor_Imu'
        diag_msg.level = DiagnosticStatus.OK
        diag_msg.message = 'Received AHRS measurement'
        diag_msg.values.append(KeyValue('roll (deg)',
                                str(roll*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('pitch (deg)',
                                str(pitch*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('yaw (deg)',
                                str(yaw*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('sequence number', str(seq)))
        diag_arr.status.append(diag_msg)
        diag_pub.publish(diag_arr)
        
ser.close
