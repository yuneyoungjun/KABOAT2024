# -*- coding:utf-8 -*-
#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32, Float64MultiArray, Int16MultiArray
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber

class WaypointNavigator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('waypoint_navigator', anonymous=True)

        # Subscribers in Unity
        # self.utm_sub = Subscriber('/GPS', Float64MultiArray)
        # self.imu_sub = Subscriber('/IMU', Float32)
        # Subscribers in Real
        # self.utm_sub = Subscriber('/KABOAT/UTM', Float64MultiArray)
        # self.imu_sub = Subscriber('/KABOAT/Heading', Float32)
        self.utm_sub = Subscriber('/GPS', Float64MultiArray)
        self.imu_sub = Subscriber('/IMU', Float32)

        self.waypoint_sub = rospy.Subscriber('/Waypoint', PointStamped, self.waypointCallback)

        # Publishers
        self.angle_pub = rospy.Publisher('/waypoint_angle', Float32, queue_size=10)
        self.control_pub = rospy.Publisher('/control', Int16MultiArray, queue_size=10)

        # Synchronize UTM and IMU messages
        self.ats = ApproximateTimeSynchronizer([self.utm_sub, self.imu_sub], queue_size=10, slop=0.1, allow_headerless=True)
        self.ats.registerCallback(self.callback)

        # List of waypoints
        self.waypoints = []  # Fill in your waypoints
        self.current_waypoint_index = 0

    def waypointCallback(self, wp):
        if(wp.point.z == -1):
            self.waypoints = []
        else:
            self.waypoints.append([wp.point.x, wp.point.y])

    def callback(self, utm_msg, imu_msg):
        # Get current UTM coordinates
        utm_coords = np.array(utm_msg.data)

        # Get current orientation from IMU (assuming yaw angle in degrees)
        current_yaw = np.radians(imu_msg.data)
        if(len(self.waypoints) == 0):
            self.current_waypoint_index = 0
            rospy.loginfo("Waypoint Empty")
            self.publish_stop()
            return
        # Get current waypoint
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("All waypoints reached")
            self.publish_stop()
            return

        current_waypoint = np.array(self.waypoints[self.current_waypoint_index])

        # Calculate angle to waypoint
        angle_to_waypoint = self.calculate_angle(utm_coords, current_waypoint, current_yaw)

        # Publish angle
        self.angle_pub.publish(angle_to_waypoint)

        # Calculate control values based on relative angle
        self.publish_control_values(angle_to_waypoint.data, imu_msg.data)

        # Check if waypoint is reached (threshold distance)
        if np.linalg.norm(current_waypoint - utm_coords) < 3.0:  # 3 meter threshold
            self.current_waypoint_index += 1

    def calculate_angle(self, current_pos, waypoint, current_yaw):
        # Calculate the angle from current position to waypoint
        delta_y = waypoint[1] - current_pos[1]
        delta_x = waypoint[0] - current_pos[0]
        angle_to_waypoint = np.arctan2(delta_x, delta_y)

        # Convert angle to be relative to North (0 degrees) and within -180 to 180
        relative_angle = angle_to_waypoint - current_yaw
        relative_angle = np.degrees(relative_angle)
        relative_angle = (relative_angle + 180) % 360 - 180

        return Float32(data=relative_angle)

    def publish_control_values(self, relative_angle, imu_angle):
        global timepast, psi_error_past, psi_error_sum, u_error_past, u_error_sum, k

        dt = rospy.get_time() - self.timepast

        control_values = Int16MultiArray()
        control_values.data = [0, 0, 0, 0, 0, 0]  # 초기값 설정

        psi_error = relative_angle
        u_error = 150

        ##### Parameter #####
        u_p_gain = 1.0
        u_d_gain = 0.0
        u_i_gain = 0.0
        psi_p_gain = 2.5
        psi_d_gain = 0.5
        psi_i_gain = 0.0

        maxThrust = 250
        minThrust = -250

        ##########
        rospy.loginfo("Error of psi : %f [degree]" , psi_error)

        psi_error_dot = (psi_error - self.psi_error_past) / dt
        self.psi_error_past = psi_error
        self.psi_error_sum += psi_error * dt

        tau_N = psi_p_gain * psi_error + psi_d_gain * psi_error_dot + psi_i_gain * self.psi_error_sum
        
        u_error_dot = (u_error - self.u_error_past) / dt
        self.u_error_past = u_error
        self.u_error_sum += u_error * dt

        tau_X = u_p_gain * u_error +  u_d_gain * u_error_dot + u_i_gain * self.u_error_sum


        Lpwm =  (tau_X * -1.0 + tau_N * -0.5)
        Rpwm =  (tau_X * -1.0 + tau_N * 0.5)
        LRpwm =  (tau_X * 0.0 + tau_N * 1.0)

        Lpwm = int(min(max(Lpwm, minThrust), maxThrust))
        Rpwm = int(min(max(Rpwm, minThrust), maxThrust))
        LRpwm = int(min(max(LRpwm, minThrust), maxThrust))

        control_values.data[3] = Lpwm
        control_values.data[4] = Rpwm
        control_values.data[5] = -LRpwm
        rospy.loginfo("Motor speed(L) : %f " , Lpwm)    
        rospy.loginfo("Motor speed(R) : %f " , Rpwm)
        rospy.loginfo("Motor speed(LR) : %f " , LRpwm)
        rospy.loginfo(control_values)
        self.control_pub.publish(control_values)
        self.timepast = rospy.get_time()

    def publish_stop(self):
        self.control_pub.publish(Int16MultiArray(data = [0, 0, 0, 0, 0, 0]))

if __name__ == '__main__':
    try:
        waypoint_navigator = WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
