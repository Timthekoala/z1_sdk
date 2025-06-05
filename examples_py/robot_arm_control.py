#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import os
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np

def twist_callback(msg):
    print(unitree_arm_interface.LowlevelState)
    angular_vel_x = msg.angular.x
    angular_vel_y = msg.angular.y
    angular_vel_z = msg.angular.z
    linear_vel_x = msg.linear.x
    linear_vel_y = msg.linear.y
    linear_vel_z = msg.linear.z
    arm.cartesianCtrlCmd(np.array([linear_vel_x, linear_vel_y, linear_vel_z, angular_vel_x, angular_vel_y, angular_vel_z, -1]), 0.3, 0.3)

if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node('robot_arm_controller')

    np.set_printoptions(precision=3, suppress=True)
    arm = unitree_arm_interface.ArmInterface(hasGripper=True)
    armState = unitree_arm_interface.ArmFSMState
    arm.loopOn()

    arm.labelRun("forward")
    arm.startTrack(armState.CARTESIAN)

    twist_sub = rospy.Subscriber('/cmd_vel', Twist, twist_callback)

    rospy.spin()

    arm.backToStart()
    arm.loopOff()

