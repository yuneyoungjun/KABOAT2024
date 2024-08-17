#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 전역 변수로 temp 리스트 선언
temp = [0] * 241

def callback(data):
    global temp
    for i, j in enumerate(data.ranges):
        print(len(data.ranges))
        if j == math.inf:
            temp[i-60] = 100
        else:
            temp[i-60] = j

    # ROS 메시지 발행
    ld = Float64MultiArray()
    ld.data = temp
    pub.publish(ld)

def update_plot(frame):
    # 그래프 업데이트
    plt.cla()  # 이전 플롯을 지우고
    plt.plot(temp,'ro')
    plt.ylim(-10, 10)  # y축 범위 설정
    plt.title("Lidar Data Visualization")
    plt.xlabel("Angle (degrees)")
    plt.ylabel("Distance (m)")

if __name__ == '__main__':
    rospy.init_node('Lidar_talker', anonymous=False)
    pub = rospy.Publisher('/LidarData', Float64MultiArray, queue_size=10)
    rospy.Subscriber('/laserscan', LaserScan, callback)

    # matplotlib 초기화
    fig = plt.figure()
    ani = FuncAnimation(fig, update_plot, interval=100)  # 100ms 간격으로 업데이트

    plt.show()  # 그래프 표시
    
    rospy.spin()
