#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import PointStamped
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import message_filters

# 초기 거리 데이터 설정
distances = []
angles = []
waypoints = []  # Waypoint 데이터를 저장할 리스트
waypoint_angle = 0  # Waypoint 각도를 저장할 리스트
threshold = 100.0  # 특정 거리 임계값 설정 (예: 100.0 센티미터)

gps_position = []
heading_angle = 0

# 시각화 함수
def update(frame):
    global distances, angles, waypoints, waypoint_angle

    ax.clear()
    ax.set_title('Distance Data in Polar Coordinates', va='bottom')
    ax.set_ylim(0, 20)  # Y축 범위 설정

    ax.set_theta_zero_location("N")  # 북쪽을 0도로 설정
    ax.set_theta_direction(-1)
    
    # 점으로만 표시
    ax.scatter(angles, distances, color='blue', s=2)  # 라이다 데이터 점으로 표시

    # Waypoint 데이터 표시
    if waypoints:
        waypoint_x = [(point[0]-gps_position[0]) for point in waypoints]
        waypoint_y = [(point[1]-gps_position[1]) for point in waypoints]
        print(waypoint_x, waypoint_y)

        # Waypoint 점으로 표시
        ax.scatter(np.radians(np.arctan2(waypoint_x, waypoint_y) * 180 / np.pi - heading_angle), 
                    np.sqrt(np.power(waypoint_x, 2) + np.power(waypoint_y, 2)), 
                    color='red', label='Waypoints')  

        # 각 Waypoint에 인덱스 번호 텍스트 추가
        for idx, (wx, wy) in enumerate(zip(waypoint_x, waypoint_y)):
            angle = np.radians(np.arctan2(wx, wy) * 180 / np.pi - heading_angle)
            distance = np.sqrt(np.power(wx, 2) + np.power(wy, 2))
            ax.text(angle, distance, str(idx + 1), color='black', fontsize=8, ha='center', va='bottom')

    # Waypoint 각도 표시
    ax.plot([0, np.radians(waypoint_angle)], [0, 15], color='green', label='Waypoint Angle')  # 각도는 고정된 거리에서 표시

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

def waypoint_angle_callback(data):
    global waypoint_angle
    waypoint_angle = data.data # 각도 데이터 추가

def gps_callback(data):
    global gps_position
    gps_position = data.data  # GPS 위치 업데이트 (예: [x, y])

def imu_callback(data):
    global heading_angle
    heading_angle = data.data  # 방위각 업데이트

def listener(is_simulator=False):
    rospy.init_node('distance_visualizer', anonymous=True)

    # 적절한 토픽 구독
    if is_simulator:
        rospy.Subscriber("Lidar", Float64MultiArray, simulator_laser_scan_callback)
    else:
        rospy.Subscriber("scan", LaserScan, laser_scan_callback)

    gps_sub = message_filters.Subscriber("KABOAT/UTM", Float64MultiArray)
    imu_sub = message_filters.Subscriber("KABOAT/Heading", Float32)

    # Waypoint 및 Waypoint Angle 토픽 구독
    rospy.Subscriber("/Waypoint", PointStamped, waypoint_callback)
    rospy.Subscriber("/waypoint_angle", Float32, waypoint_angle_callback)

    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ts.registerCallback(lambda gps_data, imu_data: (gps_callback(gps_data), imu_callback(imu_data)))

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
