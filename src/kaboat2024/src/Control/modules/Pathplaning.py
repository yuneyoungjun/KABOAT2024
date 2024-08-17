# -*- coding:utf-8 -*-
#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32, Float64MultiArray, Int16MultiArray
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import LaserScan
from math import floor, ceil
import math


class Path:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('Pathplan', anonymous=True)
        # Subscribers in Unity
        self.utm_sub = Subscriber('/GPS', Float64MultiArray)
        self.imu_sub = Subscriber('/IMU', Float32)
        self.lidar_sub=Subscriber('/Lidar', Float64MultiArray)
        # Subscribers in Real
        # self.utm_sub = Subscriber('/KABOAT/UTM', Float64MultiArray)
        # self.imu_sub = Subscriber('/KABOAT/Heading', Float32)
        # self.lidar_sub=Subscriber('/laserscan', LaserScan)


        # self.waypoint_sub = rospy.Subscriber('/Waypoint', PointStamped, self.waypointCallback)
        self.waypoint_sub = rospy.Subscriber('/pub_waypoint', PointStamped, self.waypointCallback)
        self.ats = ApproximateTimeSynchronizer([self.utm_sub, self.imu_sub,self.lidar_sub], queue_size=10, slop=10000000000, allow_headerless=True)
        self.ats.registerCallback(self.pasthCallback)
        self.angle_pub = rospy.Publisher('/waypoint_angle_plot', Float32, queue_size=10)
        self.control_pub = rospy.Publisher('/control', Int16MultiArray, queue_size=10)
        self.waypoints = []  # Fill in your waypoints
        self.current_waypoint_index = 0
        self.num=241
        self.temp = [1000] * self.num
    def CostFuncAngle(self,x):
        x = abs(x)
        if(x <= 10):
            return 0.01 * x
        else:
            return 0.1*(x - 10)
    def CostFuncDistance(self,x, avoidRange):
        if(x > avoidRange+2):
            return 20
        elif(x == 0):
            return 20.2
        else:
            return 24 + 2*avoidRange - 2* x 
    def distance_callback(self,data):
        for i, j in enumerate(data.ranges):
            if j == math.inf and i%2==0:
                self.temp[i] = 100
            elif j != math.inf and i%2==0:
                self.temp[i] = j
            else:
                self.temp[i] = 100
        self.temp=[1000]*self.num
        return self.temp

    def waypointCallback(self, wp):
        if(wp.point.z == -1):
            self.waypoints = []
        else:
            self.waypoints.append([wp.point.x, wp.point.y])
    def pasthCallback(self,gps,imu,lidar):
        Gain_Psi = 3
        Gain_Distance = 6
        avoidRange = 2
        BoatWidth = 6
        k=0
        psi = imu.data
        Pos = gps.data
        lidardata = self.distance_callback(lidar)
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
        WP=np.array(self.waypoints[self.current_waypoint_index])
        ld = np.array(lidardata)
        if (np.sqrt((WP[0]-Pos[0])**2 + (WP[1]-Pos[1])**2) < 10.0):
            k += 1   
        yaw = (psi)%360    
        if yaw > 180: yaw -= 360

        Goal_Psi_d = np.arctan2(WP[0] - Pos[0], WP[1]  - Pos[1]) * 180 / np.pi
        Goal_Psi_d = int(Goal_Psi_d)
        Goal_Psi = np.arctan2(WP[0]  - Pos[0], WP[1]  - Pos[1]) * 180 / np.pi - yaw  
        Goal_Psi = int(Goal_Psi)
        
        if(Goal_Psi < -180): Goal_Psi += 360
        elif(Goal_Psi > 180): Goal_Psi -= 360    
        safeZone = [avoidRange] * 360

        for i in range(self.num):
            if(ld[i]!=0 and ld[i] < avoidRange):
                safeZone[i] = 0

        temp = np.array(safeZone)

        for i in range(self.num):
            if(safeZone[i] > safeZone[i + 1]): ### 
                for j in range(floor(i + 1 - np.arctan2(BoatWidth/2, ld[i + 1]) * 180 / np.pi),i + 1):
                    temp[j] = 0
            if(safeZone[i] < safeZone[i + 1]): ### 
                for j in range(i, ceil(i + np.arctan2(BoatWidth/2, ld[i]) * 180 / np.pi) + 1):
                    temp[j] = 0
        safeZone = temp


        # thetaList = [[-61, 10000 * abs(-61-Goal_Psi)], [61, 10000 * abs(61-Goal_Psi)]]
        thetaList = [[-61, 1000000000 * abs(-61-Goal_Psi)], [61, 1000000000000 * abs(61-Goal_Psi)]]

        if(safeZone[Goal_Psi] > 0):
            thetaList.append([Goal_Psi,np.round_(self.CostFuncDistance(ld[i],avoidRange)*Gain_Distance + self.CostFuncAngle(-Goal_Psi)*Gain_Psi ,2)])    

        for i in range(self.num):
            if(safeZone[i] > 0):
                cost = np.round_(self.CostFuncDistance(ld[i],avoidRange)*Gain_Distance + self.CostFuncAngle(i - Goal_Psi)*Gain_Psi ,1)
                thetaList.append([i, cost])
            
        # if(safeZone[Goal_Psi] > 0):
        #     thetaList.append([Goal_Psi, 20.5])
        
        
        thetaList = sorted(thetaList, key = lambda x : x[1])
        print(Goal_Psi)
        print(thetaList) 
        Psi_d = (thetaList[0][0]-120)//2

        psi_error = Psi_d

        # psi_error %= 360

        # if psi_error > 180: 
        #     psi_error -=360
        self.angle_pub.publish(yaw+psi_error)
        self.publish_control_values(psi_error, imu.data)
        if np.linalg.norm(WP - gps.data) < 3.0:  # 3 meter threshold
            self.current_waypoint_index += 1
        return yaw+psi_error
    def publish_stop(self):
        self.control_pub.publish(Int16MultiArray(data = [0, 0, 0, 0, 0, 0]))


    def publish_control_values(self, relative_angle, imu_angle):
        control_values = Int16MultiArray()
        control_values.data = [0, 0, 0, 0, 0, 0]  # 초기값 설정


        ##################################### Change #####################################
        # Calculate control values based on relative angle
        if relative_angle > 0:  # 오른쪽으로 회전
            control_values.data[3] = min(500, control_values.data[0] + int(relative_angle * 10))  # left
            control_values.data[4] = max(-500, control_values.data[1] - int(relative_angle * 10))  # right
            # control_values.data[2] = min(500, control_values.data[2] + 50)  # front
        elif relative_angle < 0:  # 왼쪽으로 회전
            control_values.data[3] = max(-500, control_values.data[0] - int(-relative_angle * 10))  # left
            control_values.data[4] = min(500, control_values.data[1] + int(-relative_angle * 10))  # right
            # control_values.data[2] = max(-500, control_values.data[2] - 50)  # front

        control_values.data[3] = min(500, control_values.data[3] + 200)
        control_values.data[4] = min(500, control_values.data[4] + 200)
        ##################################################################################
        
        # Publish control values
        print(relative_angle,control_values.data)
        self.control_pub.publish(control_values)
if __name__ == '__main__':
    try:
        waypoint_navigator = Path()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
