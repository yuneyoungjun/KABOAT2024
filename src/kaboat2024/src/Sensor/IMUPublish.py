#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import tf
from math import pi
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import Vector3Stamped
import message_filters

# 필터 파라미터 및 전역 변수 초기화
alpha = 0.97
current_heading = 0.0
last_time = None  # 초기화 시 None으로 설정
time_data = []
gyro_data = []
accel_data = []
yaw_data = []    # Yaw 데이터를 저장할 리스트
heading_data = [] # 상보 필터의 결과 데이터를 저장할 리스트
MAX_POINTS = 100  # 실시간 플롯에서 표시할 데이터 포인트 수 제한

def getYaw(q):
    quaternion = (q.x, q.y, q.z, q.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = -1.0 * euler[2] * 180.0 / pi+154
    if yaw > 180: 
        yaw -= 360
    return yaw

def callback(msg,mag):
    global current_heading, last_time, alpha, time_data, gyro_data, accel_data, yaw_data, heading_data,Yaw_average

    current_time = rospy.Time.now()
    
    if last_time is None:  # last_time이 None일 경우 초기화
        last_time = current_time
        return

    dt = (current_time - last_time).to_sec()  # 시간 차이 계산
    last_time = current_time

    # 자이로스코프 데이터
    gyro_z = -msg.angular_velocity.z * 180 / pi
    gyro_data.append(gyro_z)

    # 가속도계를 이용해 각도 계산
    accel_angle = calculate_accel_angle(mag)
    accel_data.append(accel_angle)
    yaw = getYaw(msg.orientation)
    yaw_data.append(yaw)


    # 상보 필터 적용
    current_heading = alpha * (current_heading+gyro_z*dt) + (1 - alpha) * accel_angle

    # HEADING 퍼블리시
    heading_msg = Float64()
    heading_msg.data = current_heading
    heading_publisher.publish(heading_msg)
    heading_data.append(current_heading)
    # Yaw 계산

    # 시간 데이터 추가
    time_data.append(rospy.get_time()) 

    # 데이터 포인트 수 제한
    if len(time_data) > MAX_POINTS:
        time_data = time_data[-MAX_POINTS:]
        gyro_data = gyro_data[-MAX_POINTS:]
        accel_data = accel_data[-MAX_POINTS:]
        yaw_data = yaw_data[-MAX_POINTS:]
        heading_data = heading_data[-MAX_POINTS:]



def calculate_accel_angle(msg):
    # 가속도계 데이터에서 x, y 성분을 가져옵니다.
    accel_x = msg.vector.x
    accel_y = msg.vector.y
    accel_z = msg.vector.z # 중력 보정

    # 아크탄젠트를 사용하여 각도를 계산합니다.
    angle = math.atan2(accel_y, accel_x)*180/math.pi
    return angle



def plot(frame):
    global time_data, gyro_data, accel_data, yaw_data, heading_data
    ax1.clear()
    ax2.clear()
    try:
        # 첫 번째 서브플롯: 자이로 및 가속도 데이터
        ax1.plot(time_data, yaw_data, label='Gyro Z', color='r')
        ax1.plot(time_data, accel_data, label='Accel Angle', color='b')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Degrees')
        ax1.set_title('Gyro and Accel Data')
        ax1.legend()
        ax1.grid()

        # 두 번째 서브플롯: Yaw와 필터 Heading 데이터
        ax2.plot(time_data, yaw_data, label='Yaw (IMU)', color='g')
        ax2.plot(time_data, heading_data, label='Heading (Filter)', color='m')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Degrees')
        ax2.set_title('Yaw and Filtered Heading Data')
        ax2.legend()
        ax2.grid()
    except:
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('complementary_filter_node', anonymous=True)

        # IMU 데이터 구독
        gyro_subscriber = message_filters.Subscriber('/imu/data', Imu)
        mag_subscriber = message_filters.Subscriber('/imu/mag', Vector3Stamped)
        ts = message_filters.ApproximateTimeSynchronizer([gyro_subscriber, mag_subscriber], queue_size=1000, slop=0.1, allow_headerless=True)
        
        # HEADING 데이터 퍼블리셔
        heading_publisher = rospy.Publisher('/HEADING', Float64, queue_size=10)
        ts.registerCallback(callback)

        # last_time 초기화
        last_time = rospy.Time.now()  # 노드가 초기화된 후 시간 설정

        # Matplotlib 설정
        plt.ion()
        global fig, ax1, ax2
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))  # 두 개의 서브플롯
        ani = FuncAnimation(fig, plot, interval=100)
        plt.show()

        # ROS 루프 실행
        rate = rospy.Rate(10)  # 10Hz로 실행
        while not rospy.is_shutdown():
            plt.pause(0.01)  # Matplotlib 업데이트
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
