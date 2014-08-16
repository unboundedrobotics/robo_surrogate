#!/usr/bin/env python

"""
Converts PS3 joystick to cartesian twist controller for arm
"""

from __future__ import print_function

import rospy
import rosbag
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import sys

class PS3ToArmTwist:
    def __init__(self):
        self.pub = rospy.Publisher('/arm_controller/cartesian_twist/command', Twist)
        self.sub = rospy.Subscriber('joy', Joy, self.joyCB)
        self.joy_msg = None

    def update(self):
        joy_msg = self.joy_msg
        self.joy_msg = None
        if joy_msg is not None:
            cmd = Twist()
            if joy_msg.buttons[8]: #left bottom trigger button
                vel_scale = 1.0
                cmd.linear.x = joy_msg.axes[3] * vel_scale #right stick, forward/back
                cmd.linear.y = joy_msg.axes[2] * vel_scale #right stick, side-to-side
                cmd.linear.z = joy_msg.axes[1] * vel_scale #left stick, forward/back
            elif joy_msg.buttons[9]: #right bottom trigger button
                ang_scale = 1.0
                cmd.angular.x = joy_msg.axes[2] * ang_scale #right stick, side-to-side
                cmd.angular.y = joy_msg.axes[3] * ang_scale #right stick, forward/back
                cmd.angular.z = joy_msg.axes[0] * ang_scale #left stick, side-to-side
            self.pub.publish(cmd)
            #print(cmd)
            
    def joyCB(self, joy_msg):
        self.joy_msg = joy_msg


if __name__=="__main__":
    rospy.init_node('command_twist')

    ps3 = PS3ToArmTwist()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ps3.update()
        rate.sleep()

