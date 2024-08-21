#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

# 시뮬레이션 시간 및 파라미터 설정
dt = 0.01  # 시간 간격
time = np.arange(0, 100, dt)  # 시뮬레이션 시간
window_size = 50.0  # 애니메이션에서 표시할 시간 창 (50초)

# PD 제어기 파라미터
Kp = 2.0  # 비례 게인
Kd = 1.5  # 미분 게인

# 초기화
error_prev = 0.0
position = 0.0
velocity = 0.0
position_list = []
time_list = []
error = 0.0  # Global variable to store the subscribed error

state = True
initial_error = 0
text_displayed = False  # 텍스트가 표시되었는지 여부를 추적

# ROS 노드 초기화
rospy.init_node('pd_controller', anonymous=True)

# 에러 값 콜백 함수
def error_callback(msg):
    global error, state, initial_error, text_displayed
    if state and abs(msg.data) > 0:
        initial_error = msg.data
        state = False
    error = msg.data
    # print("Initial Value {0} / Error {1}".format(initial_error, error))

def waypointCallback(msg):
    global state, position_list, time_list, error_prev, position, velocity, text_displayed, time
    
    # 데이터 초기화
    position_list = []
    time_list = []
    error_prev = 0.0
    position = 0.0
    velocity = 0.0
    state = True  # 새로운 initial error를 설정하기 위해 초기화
    text_displayed = False  # 새로운 waypoint에 대해 텍스트를 다시 표시
    time = np.arange(0, 100, dt)  # 시뮬레이션 시간

    print(f"New waypoint received: ({msg.point.x}, {msg.point.y}, {msg.point.z})")
    
# ROS 구독자 설정
rospy.Subscriber('/error', Float32, error_callback)
rospy.Subscriber('/Waypoint', PointStamped, waypointCallback)

# 애니메이션 설정
fig, ax = plt.subplots()
ax.set_xlim(0, window_size)
ax.set_ylim(-180, 180)  # 목표 값보다 약간 더 큰 범위
line, = ax.plot([], [], lw=2)
initial_line = ax.axhline(initial_error, color='r', linestyle='--', label='Initial Error')
initial_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=12, verticalalignment='top')  # 초기 텍스트 설정

def init():
    line.set_data([], [])
    return line,

def update(frame):
    global  velocity, error_prev, error, initial_error, text_displayed,time
    t = frame
    
    
    # 데이터 업데이트
    position_list.append(initial_error - error)
    time_list.append(t)
    error_prev = error
    
    # 시간 창 업데이트: 윈도우 사이즈보다 오래된 데이터는 삭제
    if t > window_size:
        ax.set_xlim(t - window_size, t)
        while time_list and time_list[0] < t - window_size:
            time_list.pop(0)
            position_list.pop(0)
    
    line.set_data(time_list, position_list)
    
    # 업데이트된 initial_error 값을 반영
    initial_line.set_ydata(initial_error)
    
    # initial_error가 설정되면 화면에 텍스트로 표시
    if not text_displayed and not state:
        initial_text.set_text(f'Initial Error: {initial_error:.2f}')
        text_displayed = True
    
    return line, initial_line, initial_text

# 애니메이션 생성
ani = FuncAnimation(fig, update, frames=time, init_func=init, blit=True, interval=dt*1000)

# 애니메이션 표시
plt.title('PD Control Animation - Position Tracking with Window')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.legend()

# 애니메이션을 ROS와 함께 실행
try:
    plt.show()
    rospy.spin()  # Keep the ROS node running
except rospy.ROSInterruptException:
    pass
