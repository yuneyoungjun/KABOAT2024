#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32, Float64MultiArray
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber

class WaypointNavigator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('waypoint_navigator', anonymous=True)

        # Subscribers
        self.utm_sub = Subscriber('/UTM', Float64MultiArray)
        self.imu_sub = Subscriber('/Heading', Float32)

        # Publisher
        self.angle_pub = rospy.Publisher('/waypoint_angle', Float32, queue_size=10)

        # Synchronize UTM and IMU messages
        self.ats = ApproximateTimeSynchronizer([self.utm_sub, self.imu_sub], queue_size=10, slop=0.1, allow_headerless=True)
        self.ats.registerCallback(self.callback)

        # List of waypoints
        self.waypoints = [[0, 0], [5, 5]]  # Fill in your waypoints
        self.current_waypoint_index = 0

    def callback(self, utm_msg, imu_msg):
        # Get current UTM coordinates
        utm_coords = np.array(utm_msg.data)

        # Get current orientation from IMU (assuming yaw angle in degrees)
        current_yaw = np.radians(imu_msg.data)

        # Get current waypoint
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("All waypoints reached")
            return

        current_waypoint = np.array(self.waypoints[self.current_waypoint_index])

        # Calculate angle to waypoint
        angle_to_waypoint = self.calculate_angle(utm_coords, current_waypoint, current_yaw)

        print(angle_to_waypoint)
        # Publish angle
        self.angle_pub.publish(angle_to_waypoint)

        # Check if waypoint is reached (threshold distance)
        if np.linalg.norm(current_waypoint - utm_coords) < 1.0:  # 1 meter threshold
            self.current_waypoint_index += 1

    def calculate_angle(self, current_pos, waypoint, current_yaw):
        # Calculate the angle from current position to waypoint
        delta_y = waypoint[1] - current_pos[1]
        delta_x = waypoint[0] - current_pos[0]
        angle_to_waypoint = np.arctan2(delta_y, delta_x)

        # Convert angle to be relative to North (0 degrees) and within -180 to 180
        relative_angle = angle_to_waypoint - current_yaw
        relative_angle = np.degrees(relative_angle)
        relative_angle = (relative_angle + 180) % 360 - 180

        return Float32(data=relative_angle)

if __name__ == '__main__':
    try:
        waypoint_navigator = WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
