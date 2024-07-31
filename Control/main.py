import rospy
from modules.Waypoint import WaypointNavigator


if __name__ == '__main__':
    try:
        waypoint_navigator = WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
