#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from modules.Pathplaning import Path


if __name__ == '__main__':
    try:
        waypoint_navigator = Path()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
