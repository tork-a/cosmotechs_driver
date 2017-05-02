#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import rospy
import actionlib
from cosmotechs_driver.msg import (
    MultiJointPositionAction, MultiJointPositionGoal)

class TestPcpg23i(unittest.TestCase):
    def test_command(self):

        client = actionlib.SimpleActionClient('/pcpg23i_node/command', MultiJointPositionAction)

        rospy.loginfo('wait0')
        client.wait_for_server()
        rospy.loginfo('wait1')
        goal = MultiJointPositionGoal(positions=[1, 2])
        client.send_goal(goal)
        rospy.loginfo('wait2')
        client.wait_for_result()
        result = client.get_result()
        self.assertEqual(result.positions, (1,2))

if __name__ == '__main__':
    rospy.init_node('test_pcpg23i')

    import rostest
    rostest.rosrun('cosmotechs_driver', 'test_pcpg23i', TestPcpg23i)
