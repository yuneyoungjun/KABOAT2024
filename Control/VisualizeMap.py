
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 초기 거리 데이터 설정
distances = [0] * 360
angles = np.radians(np.arange(360))

# 시각화 함수
def update(frame):
    # 거리 데이터 업데이트
    global distances
    
    # 극좌표 그래프 업데이트
    ax.clear()
    ax.set_title('Distance Data in Polar Coordinates', va='bottom')
    ax.set_ylim(0, max(distances) + 1)
    
    # 점으로만 표시
    ax.scatter(angles, distances, color='blue', s = 2)  # 점으로 표시
    ax.grid(True)

def distance_callback(data):
    global distances
    distances = data.data  # 수신한 거리 데이터로 업데이트

def listener():
    rospy.init_node('distance_visualizer', anonymous=True)
    rospy.Subscriber("Lidar", Float64MultiArray, distance_callback)
    
    # Matplotlib 설정
    global fig, ax
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    
    # 애니메이션 설정
    ani = FuncAnimation(fig, update, interval=100)  # 100ms마다 업데이트
    plt.show()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
