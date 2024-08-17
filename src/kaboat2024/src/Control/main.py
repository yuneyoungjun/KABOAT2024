# -*- coding:utf-8 -*-
#!/usr/bin/env python3

import rospy
from modules.Waypoint import WaypointNavigator

#########################aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaass
if __name__ == '__main__':
    try:
        waypoint_navigator = WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
