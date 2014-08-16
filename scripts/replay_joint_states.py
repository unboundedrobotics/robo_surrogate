#!/usr/bin/env python

"""
Republish JointStates from bagfile under different ROS topic (arm_target/joint_states)
"""

from __future__ import print_function

import rospy
import rosbag
from sensor_msgs.msg import JointState
import sys



if __name__=="__main__":
    if len(sys.argv) != 2:
        print("Usage: replay_joint_states.py <filename.bag>")
        exit(1)

    rospy.init_node('replay_joint_states')
    pub = rospy.Publisher('arm_target/joint_states', JointState)

    names = ['elbow_flex_joint', 'forearm_roll_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'upperarm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint', 'left_gripper_joint', 'right_gripper_joint']

    bagfn = sys.argv[1]
    bag = rosbag.Bag(bagfn)
    for topic, msg, t, in bag.read_messages(topics=['joint_states']):        
        values = {n:(p,v,e) for n,p,v,e in zip(msg.name, msg.position, msg.velocity, msg.effort)}
        msg.name = names
        msg.position = [values[n][0] for n in names]
        msg.velocity = [values[n][1] for n in names]
        msg.effort = [values[n][2] for n in names]
        msg.header.stamp = rospy.Time.now()
        print(msg)
        pub.publish(msg)
        rospy.sleep(100e-3)
        if rospy.is_shutdown():
            break
    bag.close




