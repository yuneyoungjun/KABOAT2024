#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped  # PointStamped 메시지 임포트
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 초기 거리 데이터 설정
distances = []
angles = []
waypoints = []  # Waypoint 데이터를 저장할 리스트
threshold = 100.0  # 특정 거리 임계값 설정 (예: 100.0 센티미터)

# 시각화 함수
def update(frame):
    global distances, angles, waypoints

    ax.clear()
    ax.set_title('Distance Data in Polar Coordinates', va='bottom')
    ax.set_ylim(0, 20)  # Y축 범위 설정

    ax.set_theta_zero_location("N")  # 북쪽을 0도로 설정
    ax.set_theta_direction(-1)
    
    # 점으로만 표시
    ax.scatter(angles, distances, color='blue', s=2)  # 라이다 데이터 점으로 표시

    # Waypoint 데이터 표시
    if waypoints:
        waypoint_x = [point[0] for point in waypoints]
        waypoint_y = [point[1] for point in waypoints]
        ax.scatter(np.arctan2(waypoint_y, waypoint_x), np.sqrt(np.square(waypoint_x) + np.square(waypoint_y)), color='red', label='Waypoints')  # Waypoint 점으로 표시

    ax.grid(True)
    ax.legend()

def laser_scan_callback(data):
    global distances, angles
    distances = np.flip(data.ranges)  # 수신한 거리 데이터
    distances[distances > threshold] = 0  # 임계값 초과 시 0으로 변경
    angles = np.linspace(data.angle_min, data.angle_max, len(distances))  # 각도 생성

def simulator_laser_scan_callback(data):
    global distances, angles
    distances = np.array(data.data)  # 수신한 거리 데이터
    distances[distances > threshold] = 0  # 임계값 초과 시 0으로 변경
    angles = np.radians(np.arange(len(distances)))  # 각도 생성 (0 ~ 360도)

def waypoint_callback(data):
    global waypoints
    if(data.point.z == -1):
        waypoints = []
    else:
        waypoints.append([data.point.x, data.point.y])

def listener(is_simulator=False):
    rospy.init_node('distance_visualizer', anonymous=True)

    # 적절한 토픽 구독
    if is_simulator:
        rospy.Subscriber("Lidar", Float64MultiArray, simulator_laser_scan_callback)
    else:
        rospy.Subscriber("scan", LaserScan, laser_scan_callback)

    # Waypoint 토픽 구독
    rospy.Subscriber("/Waypoint", PointStamped, waypoint_callback)

    # Matplotlib 설정
    global fig, ax
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

    # 애니메이션 설정
    ani = FuncAnimation(fig, update, interval=100)  # 100ms마다 업데이트
    plt.show()

if __name__ == '__main__':
    try:
        listener(is_simulator=True)  # 시뮬레이터 여부에 따라 설정
    except rospy.ROSInterruptException:
        pass
