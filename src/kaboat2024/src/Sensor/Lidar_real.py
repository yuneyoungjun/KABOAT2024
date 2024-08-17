#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Float32
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import message_filters
from geometry_msgs.msg import PointStamped
# -*- coding:utf-8 -*-
num = 241
# 전역 변수로 temp 리스트 선언
temp = [1000] * num
angles2 = np.radians(np.arange(num//2))/2
angles1= -np.radians(np.arange(num//2))/2
angles1=np.flip(angles1)
print(angles1)
angles1=np.append(angles1,0)
angles=np.append(angles1,angles2)
# angles=np.radians(np.arange(num))
heading_angle = 0.0  # 방위각
gps_position = [0.0, 0.0]  # [x, y] 좌표
desired_angle = 400
destination = [0.0, 0.0]  # [x, y] 좌표
def imu_callback(data):
    global heading_angle, desired_angle
    heading_angle = data.data # 방위각 업데이트
    desired_angle = 400


def desired_callback(data):
    global desired_angle
    desired_angle = data.data  # 방위각 업데이트


def destination_callback(data):
    global destination
    destination[0]=data.point.y
    destination[1]=data.point.x
    print(destination)

def gps_callback(data):
    global gps_position
    gps_position = data.data  # GPS 위치 업데이트 (예: [x, y])

def distance_callback(data):
    global temp
    for i, j in enumerate(data.ranges):
        if j == math.inf and i%2==0:
            temp[i] = 100
        elif j != math.inf and i%2==0:
            temp[i] = j
        else:
            temp[i] = 100
# def distance_callback(data):
#     global temp
#     for i, j in enumerate(data.data):
#         if j == math.inf:
#             temp[i - 180] = 100
#         else:
#             temp[i - 180] = j


def update_plot(frame):
    global temp, gps_position, heading_angle,destination
    ax.clear()
    # polar_ax.clear()  # Polar 플롯도 매 프레임마다 갱신합니다.

    # Cartesian 플롯 설정
    ax.set_title('Distance Data in Cartesian Coordinates')
    ax.set_xlim(-40, 40)  # x축 범위 설정
    ax.set_ylim(-40, 40)  # y축 범위 설정

    # 라이다 데이터의 Cartesian 좌표 변환
    x_positions = temp * np.sin(angles + np.radians(heading_angle)) + gps_position[0]
    y_positions = temp * np.cos(angles + np.radians(heading_angle)) + gps_position[1]
    

    # Cartesian 그래프 업데이트
    ax.scatter(x_positions, y_positions, color='blue', s=5)  # 거리 데이터 점으로 표시
    # ax.scatter(x_desti, y_desti, color='red', s=5)  # 거리 데이터 점으로 표시
    ax.scatter(gps_position[0], gps_position[1], color='red', s=50, label='GPS Position')
    ax.scatter(destination[0], destination[1], color='purple', s=50, label='Destination Position')

    # IMU 각도에 따라 초록색 직선 그리기
    line_length = 5  # 직선의 길이 설정
    line_x_end = gps_position[0] + line_length * np.sin(np.radians(heading_angle))
    line_y_end = gps_position[1] + line_length * np.cos(np.radians(heading_angle))
    ax.plot([gps_position[0], line_x_end], [gps_position[1], line_y_end], color='green', linewidth=2, label='Heading Direction')

    if desired_angle != 400:
        line_length = 20  # 직선의 길이 설정
        line_x_end = gps_position[0] + line_length * np.sin(np.radians(desired_angle))
        line_y_end = gps_position[1] + line_length * np.cos(np.radians(desired_angle))
        ax.plot([gps_position[0], line_x_end], [gps_position[1], line_y_end], color='red', linewidth=2, label='Desired Direction')

    ax.grid(True)
    ax.legend()

    # Polar 플롯 설정
    # polar_ax.set_title('Distance Data in Polar Coordinates')
    # polar_ax.set_theta_zero_location('N')  # 0도를 북쪽으로 설정
    # polar_ax.set_theta_direction(-1)  # 시계 방향으로 각도 증가
    
    # # 라이다 데이터 플로팅
    # polar_ax.scatter(-angles + np.radians(heading_angle)-np.radians(90), temp, color='blue', s=5)
    
    # # IMU 방향 표시 polar_ax = fig.add_subplot(122, projection='polar')  # 1행 2열     print('sadssegfrewfs')중 두 번째, 극좌표계

    # # 원하는 방향 표시 (if available)
    # if desired_angle != 400:
    #     polar_ax.plot([np.radians(desired_angle), np.radians(desired_angle)], [0, max(temp)], color='red', linewidth=2, label='Desired Direction')
    
    # polar_ax.set_ylim(0, 5)  # r축(반지름) 범위 설정
    # polar_ax.legend()

# 이전 코드는 그대로 유지

def listener():
    rospy.init_node('plot', anonymous=True)

    # lidar_sub = message_filters.Subscriber("/laserscan", LaserScan)
    # gps_sub = message_filters.Subscriber("/KABOAT/UTM", Float64MultiArray)
    # imu_sub = message_filters.Subscriber("/KABOAT/Heading", Float32)
    # destination = rospy.Subscriber("/Waypoint", PointStamped,destination_callback )
    destination = rospy.Subscriber("/pub_waypoint", PointStamped,destination_callback )
    lidar_sub = message_filters.Subscriber("/Lidar", Float64MultiArray)
    gps_sub = message_filters.Subscriber("/GPS", Float64MultiArray)
    imu_sub = message_filters.Subscriber("/IMU", Float32)

    ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, gps_sub, imu_sub], queue_size=10, slop=1000000, allow_headerless=True)
    ts.registerCallback(callback)
    # desired_sub = rospy.Subscriber("/waypoint_angle", Float32, desired_callback)
    desired_sub = rospy.Subscriber("/waypoint_angle_plot", Float32, desired_callback)
    # destination = rospy.Subscriber("/Waypoint", PointStamped,destination_callback )

    global ax, polar_ax, fig
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111)  # 1행 2열 중 첫 번째
    # polar_ax = fig.add_subplot(122, projection='polar')  # 1행 2열 중 두 번째, 극좌표계

    # matplotlib 초기화
    ani = FuncAnimation(fig, update_plot, interval=10)  # 10ms 간격으로 업데이트
    plt.show()  # 그래프 표시
    rospy.spin()

# 나머지 코드는 그대로 유지


def callback(lidar, GPS, IMU):
    distance_callback(lidar)
    gps_callback(GPS)
    imu_callback(IMU)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    print('sadssegfrewfs')