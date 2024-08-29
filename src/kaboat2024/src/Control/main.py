# -*- coding:utf-8 -*-
#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray, Float32, Float32MultiArray
import message_filters
import AutonomousModule

class Boat:
    def __init__(self):
        self.position = [0, 0]
        self.psi = 0
        self.scan = [0] * 360
        self.waypoints = []

class CallBack:
    threshold = 100.0  # 특정 거리 임계값 설정 (예: 100.0 센티미터)
    @staticmethod
    def laser_scan_callback(data):
        distances = np.array(data.ranges)  # 수신한 거리 데이터
        distances[distances > CallBack.threshold] = 0  # 임계값 초과 시 0으로 변경
        boat.scan = distances

    @staticmethod
    def simulator_laser_scan_callback(data):
        distances = np.array(data.data)  # 수신한 거리 데이터
        distances[distances > CallBack.threshold] = 0  # 임계값 초과 시 0으로 변경
        boat.scan = distances

    @staticmethod
    def waypoint_callback(data):
        if(data.point.z == -1):
            boat.waypoints = []
        else:
            boat.waypoints.append([data.point.x, data.point.y])
        print(boat.waypoints)
    @staticmethod
    def gps_callback(data):
        boat.position = data.data

    @staticmethod
    def imu_callback(data):
        boat.psi = data.data




def listener(is_simulator=False):
    rospy.init_node('distance_visualizer', anonymous=True)

    # 퍼블리셔 설정
    command_publish = rospy.Publisher('/command', Float32MultiArray, queue_size=10)  # 주제를 '/command'로 변경


    # 적절한 토픽 구독
    if is_simulator:
        rospy.Subscriber("Lidar", Float64MultiArray, CallBack.simulator_laser_scan_callback)
    else:
        rospy.Subscriber("scan", LaserScan, CallBack.laser_scan_callback)

    gps_sub = message_filters.Subscriber("KABOAT/UTM", Float64MultiArray)
    imu_sub = message_filters.Subscriber("KABOAT/Heading", Float32)

    # Waypoint 및 Waypoint Angle 토픽 구독
    rospy.Subscriber("/Waypoint", PointStamped, CallBack.waypoint_callback)

    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ts.registerCallback(lambda gps_data, imu_data: (CallBack.gps_callback(gps_data), CallBack.imu_callback(imu_data)))

    while(1):
        path = AutonomousModule.pathplan(boat)
        if(path == True):
            print("PAssed")
            command_publish.publish(Float32MultiArray(data = [0, 0]))
        elif(type(path)==list and len(path) > 0):
            command_publish.publish(Float32MultiArray(data = path))


if __name__ == '__main__':
    try:
        boat = Boat()
        listener(is_simulator=True)  # 시뮬레이터 여부에 따라 설정
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
