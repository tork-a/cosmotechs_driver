#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import actionlib
from cosmotechs_driver.msg import (
    MultiJointPositionAction, MultiJointPositionGoal)

if __name__ == '__main__':
    rospy.init_node('check')

    sys.argv = rospy.myargv()
    
    if not sys.argv[1:]:
        print "Usage: check.py <angle> <time>"
        exit
    ref_angle, ref_time = float(sys.argv[1]), float(sys.argv[2])
    print "angle: ", ref_angle
    print "time: ", ref_time

    client = actionlib.SimpleActionClient('/pcpg23i_node/command', MultiJointPositionAction)

    if client.wait_for_server(timeout=rospy.Duration(1)):
        goal = MultiJointPositionGoal(
            positions=[ref_angle, 0], min_duration=rospy.Duration(ref_time))
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        print result
    else:
        print "Cannot connect to action server. Check the server is available"
