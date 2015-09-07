#!/usr/bin/env python

# Copyright (c) 2015, Markus Bader
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
import copy
from time import sleep
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import tf
from genpy.rostime import Duration


class RosNode:
    
    
    def __init__(self):
        '''
        Constructor
        '''
        rospy.init_node('imu2tf', anonymous=True)
        rospy.Subscriber("imu", Imu, self.callbackIMU)
        self.br = tf.TransformBroadcaster()
        self.pose = PoseWithCovarianceStamped()
        self.measurment_last = Imu()
        self.imu_callback_counter = np.uint64(0);
        self.twist = Twist();
        self.duration = Duration()
        rospy.spin()
    
    
    def callbackIMU(self, msg):
        # rospy.loginfo("ServoCommand received!")
        avr_samples = 20.0
        if (self.imu_callback_counter < avr_samples) :
            if (self.imu_callback_counter == 0) :
                self.linear_acceleration_offest = Vector3()
                self.linear_acceleration_offest.x = 0
                self.linear_acceleration_offest.y = 0
                self.linear_acceleration_offest.z = 0
            self.measurment_last = copy.copy(msg)
            self.linear_acceleration_offest.x += msg.linear_acceleration.x / avr_samples;
            self.linear_acceleration_offest.y += msg.linear_acceleration.y / avr_samples;
            self.linear_acceleration_offest.z += msg.linear_acceleration.z / avr_samples;
        else :   
            self.duration = (msg.header.stamp - self.measurment_last.header.stamp)
            dt = self.duration.to_sec()
            self.twist.linear.x = self.twist.linear.x + (msg.linear_acceleration.x - self.linear_acceleration_offest.x) * dt 
            self.twist.linear.y = self.twist.linear.y + (msg.linear_acceleration.y - self.linear_acceleration_offest.y) * dt 
            self.twist.linear.z = self.twist.linear.z + (msg.linear_acceleration.z - self.linear_acceleration_offest.z) * dt 
            
            self.pose.pose.pose.position.x = self.pose.pose.pose.position.x + self.twist.linear.x * dt;
            self.pose.pose.pose.position.y = self.pose.pose.pose.position.y + self.twist.linear.y * dt;
            self.pose.pose.pose.position.z = self.pose.pose.pose.position.z + self.twist.linear.z * dt;
            
            self.br.sendTransform((self.pose.pose.pose.position.x, self.pose.pose.pose.position.y, self.pose.pose.pose.position.z), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "base", msg.header.frame_id)
            
        self.measurment_last = copy.copy(msg)
        self.imu_callback_counter = self.imu_callback_counter + 1
        
if __name__ == '__main__':
    node = RosNode()
