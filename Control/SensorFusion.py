import rospy
from std_msgs.msg import Float64MultiArray, Float32
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import message_filters

# 초기 거리 데이터 설정
distances = [0] * 360
angles = np.radians(np.arange(360))

# 위치 및 방위각 변수
gps_position = [0.0, 0.0]  # [x, y] 좌표
heading_angle = 0.0  # 방위각

# 시각화 함수
def update(frame):
    global distances, gps_position, heading_angle
    
    # 평면 그래프 업데이트
    ax.clear()
    ax.set_title('Distance Data in Cartesian Coordinates')
    ax.set_xlim(-10, 80)  # x축 범위 설정
    ax.set_ylim(-10, 40)  # y축 범위 설정

    # 라이다 데이터의 Cartesian 좌표 변환
    x_positions = distances * np.sin(angles + np.radians(heading_angle)) + gps_position[0]
    y_positions = distances * np.cos(angles + np.radians(heading_angle)) + gps_position[1]

    # 라이다 데이터 시각화
    ax.scatter(x_positions, y_positions, color='blue', s=2)  # 거리 데이터 점으로 표시

    # GPS 위치 표시
    ax.scatter(gps_position[0], gps_position[1], color='red', s=50, label='GPS Position')
    
    # IMU 각도에 따라 초록색 직선 그리기
    line_length = 10  # 직선의 길이 설정
    line_x_end = gps_position[0] + line_length * np.sin(np.radians(heading_angle))
    line_y_end = gps_position[1] + line_length * np.cos(np.radians(heading_angle))
    ax.plot([gps_position[0], line_x_end], [gps_position[1], line_y_end], color='green', linewidth=2, label='Heading Direction')

    ax.grid(True)
    ax.legend()

def distance_callback(data):
    global distances
    distances = data.data  # 수신한 거리 데이터로 업데이트

def gps_callback(data):
    global gps_position
    gps_position = data.data  # GPS 위치 업데이트 (예: [x, y])

def imu_callback(data):
    global heading_angle
    heading_angle = data.data  # 방위각 업데이트

def listener():
    rospy.init_node('distance_visualizer', anonymous=True)

    # 메시지 필터 설정
    lidar_sub = message_filters.Subscriber("Lidar", Float64MultiArray)
    gps_sub = message_filters.Subscriber("GPS", Float64MultiArray)
    imu_sub = message_filters.Subscriber("IMU", Float32)

    # 동기화 설정 (1초 이내의 메시지를 동기화)
    ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, gps_sub, imu_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ts.registerCallback(callback)

    # Matplotlib 설정
    global fig, ax
    fig, ax = plt.subplots()
    
    # 애니메이션 설정
    ani = FuncAnimation(fig, update, interval=100)  # 100ms마다 업데이트
    plt.show()

def callback(lidar_data, gps_data, imu_data):
    distance_callback(lidar_data)
    gps_callback(gps_data)
    imu_callback(imu_data)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
