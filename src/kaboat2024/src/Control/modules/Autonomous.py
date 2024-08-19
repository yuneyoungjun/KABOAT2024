#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import PointStamped
import numpy as np
from math import ceil
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import message_filters
from AutonomousBoatController import AutonomousBoatController

# 초기 거리 데이터 설정
distances = []
angles = []
waypoints = []  # Waypoint 데이터를 저장할 리스트
waypoint_angle = 0  # Waypoint 각도를 저장할 리스트
threshold = 100.0  # 특정 거리 임계값 설정 (예: 100.0 센티미터)

gps_position = []
heading_angle = 0

Goal_Psi = 0
Goal_Distance = 0
cost_function_values = []  # Cost function 값을 저장할 리스트
desired_heading_publisher = None  # 퍼블리셔를 위한 변수
desired_heading_history = []  # desired_heading 값을 저장할 리스트

goal_threshold = 2
def normalize_angle(angle): return (angle + 180) % 360 - 180
# 시각화 함수
def update(frame):
    global distances, angles, waypoints, waypoint_angle, Goal_Psi, Goal_Distance, cost_function_values, desired_heading_publisher, desired_heading_history

    # 첫 번째 subplot: 거리 데이터 시각화
    ax.clear()
    ax.set_title('Distance Data in Polar Coordinates', va='bottom')
    ax.set_ylim(0, 30)  # Y축 범위 설정

    ax.set_theta_zero_location("N")  # 북쪽을 0도로 설정
    ax.set_theta_direction(-1)

    # 점으로만 표시
    ax.scatter(angles, distances, color='blue', s=5)  # 라이다 데이터 점으로 표시

    if(len(distances) == 0):
        return
    safe_ld = autonomousController.calculate_safe_zone(distances)
    cost_function = autonomousController.calculate_optimal_psi_d(safe_ld, int(Goal_Psi))
    desired_heading = sorted(cost_function, key=lambda x: x[1])[0][0]

    #### 목적지 까지 장애물이 없을 때 ####
    if(goal_check()):
        desired_heading = Goal_Psi
        # desired_heading 시각화
        ax.plot([0, np.radians(desired_heading)], [0, Goal_Distance], color='blue', label='Desired Heading')  # desired_heading 히스토리 플롯
    
    else:
        # desired_heading 시각화
        ax.plot([0, np.radians(desired_heading)], [0, 10], color='green', label='Desired Heading')  # desired_heading 히스토리 플롯


    # desired_heading을 Float32 형태로 publish
    desired_heading_msg = Float32()
    desired_heading_msg.data = desired_heading
    desired_heading_publisher.publish(desired_heading_msg)  # 퍼블리셔를 통해 데이터 전송

    cost_function_values = np.transpose(cost_function)

    ax.fill(angles, safe_ld, color=[0, 1, 0, 0.2])  # 안전 구역 표시

    # Waypoint 데이터 표시
    if waypoints:
        waypoint_x = [(point[0] - gps_position[0]) for point in waypoints]
        waypoint_y = [(point[1] - gps_position[1]) for point in waypoints]

        if(goal_passed(waypoints[0][0], waypoints[0][1])):
            waypoints.pop(0)


        Goal_Psi = np.arctan2(waypoint_x[0], waypoint_y[0]) * 180 / np.pi - heading_angle
        Goal_Distance = np.sqrt(np.power(waypoint_x[0], 2) + np.power(waypoint_y[0], 2))

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
    # ax.plot([0, np.radians(waypoint_angle)], [0, 15], color='green', label='Waypoint Angle')  # 각도는 고정된 거리에서 표시

    
    ax.legend()
    ax.grid(True)
    ax.legend()

    # 두 번째 subplot: cost_function 시각화
    # ax2.clear()
    # ax2.set_title('Cost Function Over Time')
    # ax2.set_xlabel('Time (frames)')
    # ax2.set_ylabel('Cost Function Value')
    # ax2.set_ylim(0, 50)  # Y축 범위 설정
    # ax2.scatter(cost_function_values[0], cost_function_values[1], color='orange', s=2)

def goal_check():
    l = Goal_Distance
    theta = ceil(np.degrees(np.arctan2(autonomousController.BOAT_WIDTH/2, l)))

    check_ld = [0] * 360
    isAble = True

    for i in range(0, 90 - theta):
        angle = normalize_angle(int(Goal_Psi) - 90 + i)
        r = autonomousController.BOAT_WIDTH /(2 *np.cos(np.radians(i)))
        check_ld[angle] = r
        if(distances[angle] == 0):
            continue
        if(r > distances[angle]):
            isAble = False

    for i in range(-theta, theta + 1):
        check_ld[normalize_angle(int(Goal_Psi) + i)] = l
        if(distances[normalize_angle(int(Goal_Psi) + i)] < l):
            isAble = False

    for i in range(0, 90 - theta):
        angle = normalize_angle(int(Goal_Psi) + 90 - i)
        r = autonomousController.BOAT_WIDTH /(2 *np.cos(np.radians(i)))
        check_ld[angle] = r
        if(distances[angle] == 0):
            continue
        if(r > distances[angle]):
            isAble = False
    ax.fill(angles, check_ld, color=[0, 0, 1, 0.2])

    return isAble

def goal_passed(goal_x, goal_y):
    isPassed = False
    if((gps_position[0] - goal_x)**2 + (gps_position[1] - goal_y)**2 < goal_threshold ** 2):
        isPassed = True
    return isPassed

def laser_scan_callback(data):
    global distances, angles
    distances = np.array(data.ranges)  # 수신한 거리 데이터
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
    waypoint_angle = data.data  # 각도 데이터 추가

def gps_callback(data):
    global gps_position
    gps_position = data.data  # GPS 위치 업데이트 (예: [x, y])

def imu_callback(data):
    global heading_angle
    heading_angle = data.data  # 방위각 업데이트

def listener(is_simulator=False):
    global desired_heading_publisher
    rospy.init_node('distance_visualizer', anonymous=True)

    # 퍼블리셔 설정
    desired_heading_publisher = rospy.Publisher('/desired_heading', Float32, queue_size=10)

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
    global fig, ax, ax2
    fig = plt.figure(figsize=(10, 10))  # 전체 그림 크기 설정
    ax = fig.add_subplot(111, projection='polar')  # 첫 번째 subplot: 극좌표계
    # ax2 = fig.add_subplot(122)  # 두 번째 subplot: 직교좌표계
    # 애니메이션 설정
    ani = FuncAnimation(fig, update, interval=10)  # 100ms마다 업데이트
    plt.show()

if __name__ == '__main__':
    try:
        autonomousController = AutonomousBoatController()
        listener(is_simulator=True)  # 시뮬레이터 여부에 따라 설정
    except rospy.ROSInterruptException:
        pass
