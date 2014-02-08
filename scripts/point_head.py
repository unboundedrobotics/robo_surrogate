#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('pr2_surrogate')
import rospy
import actionlib
from control_msgs.msg import PointHeadActionGoal, PointHeadAction
import control_msgs.msg 
from math import pi,cos,sin
import sys
import time

def genGoal(angle, min_duration=0.0, link='base_link'):
    angle *= pi/180.
    goal = control_msgs.msg.PointHeadGoal()
    goal.pointing_axis.x = 1
    goal.pointing_axis.y = 0
    goal.pointing_axis.z = 0
    goal.target.header.frame_id = link
    goal.target.point.x = cos(angle)*10
    goal.target.point.y = sin(angle)*10
    goal.target.point.z = 1    
    goal.max_velocity = 1.0
    goal.min_duration = rospy.Duration.from_sec(min_duration)
    goal.pointing_frame = link
    return goal

if __name__ == "__main__":
    angle = float(sys.argv[1])*pi/180.0

    rospy.init_node('point_head',anonymous=True)
    client =  actionlib.SimpleActionClient('head_controller/point_head', PointHeadAction)
    print("Waiting for server")
    client.wait_for_server()

    goal = genGoal(0)
    client.send_goal(goal)
    print("Waiting for action to complete")
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    time.sleep(0.5)
    
    for i in range(20):
        angle = (i+1)*5.0
        goal = genGoal(angle, 0.4)
        client.send_goal(goal)
        #client.wait_for_result(rospy.Duration.from_sec(5.0))
        time.sleep(0.2)

    
