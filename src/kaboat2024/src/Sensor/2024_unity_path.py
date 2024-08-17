# -*- coding:utf-8 -*-
#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32, Float64MultiArray, Int16MultiArray
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from math import floor, ceil
import math


class Path:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('Pathplan', anonymous=True)
        self.utm_sub = Subscriber('/GPS', Float64MultiArray)
        self.imu_sub = Subscriber('/IMU', Float32)
        self.lidar_sub=Subscriber('/Lidar', Float64MultiArray)
        self.waypoint_sub = rospy.Subscriber('/Waypoint', PointStamped, self.waypointCallback)
        self.ats = ApproximateTimeSynchronizer([self.utm_sub, self.imu_sub,self.lidar_sub], queue_size=10, slop=0.1, allow_headerless=True)
        self.ats.registerCallback(self.pasthCallback)
        self.angle_pub = rospy.Publisher('/waypoint_angle', Float32, queue_size=10)
        self.control_pub = rospy.Publisher('/control', Int16MultiArray, queue_size=10)
        self.waypoints = []  # Fill in your waypoints
        self.current_waypoint_index = 0

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


    def waypointCallback(self, wp):
        if(wp.point.z == -1):
            self.waypoints = []
        else:
            self.waypoints.append([wp.point.x, wp.point.y])
    def pasthCallback(self,gps,imu,lidar):

        avoidRange = 8
        BoatWidth = 10
        k=0
        psi = imu.data
        Pos = gps.data
        lidardata = lidar.data
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
        for i in range(0,360,1):
            if ld[i]==0:
                ld[i]=1000
        for i in range(61, 300, 1):
            ld[i] = 0


        safeZone = [avoidRange] * 360
        for i in range(-40, 40):
            if(ld[i]!=0 and ld[i]*i/90< avoidRange):
                safeZone[i] = 0
            temp = np.array(safeZone)

        for i in range(-60, 61):
            if(safeZone[i] > safeZone[i + 1]): ### 
                for j in range(floor(i + 1 - np.arctan2(BoatWidth/2, ld[i + 1]) * 180 / np.pi),i + 1):
                    temp[j] = 0
            if(safeZone[i] < safeZone[i + 1]): ### 
                for j in range(i, ceil(i + np.arctan2(BoatWidth/2, ld[i]) * 180 / np.pi) + 1):
                    temp[j] = 0
        safeZone = temp

        

        Goal_Distance = np.sqrt(np.power(WP[0] - Pos[0], 2) + np.power(WP[1] - Pos[1], 2))

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



        DistanceCost = np.zeros([120, 1])
        AngleCost = np.zeros([120, 1])
        Cost = np.zeros([120, 1])
        DistanceCost_weight = 500000000
        DistanceCost_weight = 5000
        for i in range(-60, 61):
            if(ld[i]!=0 and ld[i] < avoidRange):
                safeZone[i] = 0

        temp = np.array(safeZone)

        for i in range(-60, 60):

        
            if ld[i] == 0: 
                DistanceCost[i + 60]=0
            else:
                DistanceCost[i + 60] = DistanceCost_weight *abs(1 / (ld[i] + 1)) * (1 / (ld[i] + 1)) * 1 / (ld[i] + 1)


        AngleCost[i + 60] = math.sqrt(abs(i - Goal_Psi))



        DistanceCostSketch = DistanceCost
        DistanceCostSketch_ver=DistanceCost
        Nowweight=1
        NearWeight=1.5

        DistanceCost[0]=(Nowweight*DistanceCostSketch[1]+NearWeight*(DistanceCostSketch[1]+DistanceCostSketch[1]))
        DistanceCostSketch[0]=DistanceCost[0]/(Nowweight+NearWeight)
        DistanceCost[i]/=(4*NearWeight+Nowweight)
        DistanceCost[118]=(Nowweight*DistanceCostSketch[118]+NearWeight*(DistanceCostSketch[118]+DistanceCostSketch[1]))
        DistanceCostSketch[0]=DistanceCost[0]/(Nowweight+NearWeight)
        DistanceCost[i]/=(4*NearWeight+Nowweight)
        for i in range(0,120):
            if i>2 and i<118:
                DistanceCost[i]=(NearWeight*(DistanceCostSketch[i-1]+DistanceCostSketch[i-2])+Nowweight*DistanceCostSketch[i]+NearWeight*(DistanceCostSketch[i+1]+DistanceCostSketch[i+2]))
                DistanceCostSketch[i]=DistanceCost[i]/(4*NearWeight+Nowweight)
                DistanceCost[i]/=(4*NearWeight+Nowweight)


        for i in range(119,-1,-1):
            if i>2 and i<118:
                DistanceCost[i]=(NearWeight*(DistanceCostSketch[i-1]+DistanceCostSketch[i-2])+Nowweight*DistanceCostSketch[i]+NearWeight*(DistanceCostSketch[i+1]+DistanceCostSketch[i+2]))
                DistanceCostSketch[i]=DistanceCost[i]/(4*NearWeight+Nowweight)
                DistanceCost[i]/=(2*NearWeight+Nowweight)
            elif i==0:
                DistanceCost[i]=(Nowweight*DistanceCostSketch[i]+NearWeight*DistanceCostSketch[i+1])
                DistanceCostSketch[i]=DistanceCost[i]/(4*Nowweight+NearWeight)
                DistanceCost[i]/=(Nowweight+NearWeight)









        Smaller = [50000000000000000000000000000000000000000000000000000000000000000000000000]
        final = 0
        # for i in range(2, 118):
        #     if Cost[i] < Smaller[0]:
        #         Smaller[0] = Cost[i]
        #         final = i - 60
        #         FinalPsi = i - 60


    


        Cost=DistanceCost+AngleCost
        # for i in range(-5,5):
        #     if(safeZone[i]==0):
        #         Cost[i+60]+=100000


        for i in range(15,105):
            if Cost[i] < Smaller[0]:
                if(safeZone[i-60]==0):
                    pass
                else:
                    Smaller[0] = Cost[i]
                    final = i - 60
                    FinalPsi = i - 60

        # if(safeZone[Goal_Psi] > 0):
        #     thetaList.append([Goal_Psi, 20.5])
        
        self.publish_control_values(final, imu.data)
        if np.linalg.norm(WP - gps.data) < 3.0:  # 3 meter threshold
            self.current_waypoint_index += 1
        return final



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
