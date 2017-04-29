#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import unittest

class TestPcio32ha(unittest.TestCase):
    def test_set_port(self):
        from cosmotechs_driver.srv import SetPort

        rospy.wait_for_service('set_port', timeout=1)
        set_port = rospy.ServiceProxy('set_port', SetPort)
        set_port(0, 0)

    def test_get_port(self):
        from cosmotechs_driver.srv import GetPort

        rospy.wait_for_service('get_port', timeout=1)
        get_port = rospy.ServiceProxy('get_port', GetPort)
        r = get_port(0)
        self.assertEqual(0, r.data)

    def test_get_port_bit(self):
        from cosmotechs_driver.srv import GetPortBit

        rospy.wait_for_service('get_port_bit', timeout=1)
        get_port_bit = rospy.ServiceProxy('get_port_bit', GetPortBit)
        r = get_port_bit(0, 0)
        self.assertEqual(0, r.data)

    def test_set_port_bit(self):
        from cosmotechs_driver.srv import SetPortBit

        rospy.wait_for_service('set_port_bit', timeout=1)
        set_port_bit = rospy.ServiceProxy('set_port_bit', SetPortBit)
        set_port_bit(0, 0, 1)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('cosmotechs_driver', 'test_pcio32ha', TestPcio32ha)
