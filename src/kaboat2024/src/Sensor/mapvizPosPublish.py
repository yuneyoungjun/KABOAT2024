#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import tf
import math
import utm  # UTM 변환을 위한 패키지

# 기준 GPS 위치
ref_gps_x = 36.368706  # 기준 위도
ref_gps_y = 127.345411  # 기준 경도

gps_x = 0.0
gps_y = 0.0
quaternion = (0.0, 0.0, 0.0, 1.0)

def gps_callback(gps_msg):
    global gps_x, gps_y
    # GPS 데이터에서 위치 정보를 가져옵니다.
    gps_x = gps_msg.latitude  # 위도를 y로 사용
    gps_y = gps_msg.longitude   # 경도를 x로 사용

def heading_callback(msg):
    global quaternion
    # PoseStamped 메시지 생성
    pose = PoseStamped()
    pose.header.frame_id = "map"  # GPS 데이터의 기준 프레임
    pose.header.stamp = rospy.Time.now()

    # 기준 GPS 좌표를 UTM으로 변환
    base_utm = utm.from_latlon(ref_gps_x, ref_gps_y)
    # 현재 GPS 좌표를 UTM으로 변환
    current_utm = utm.from_latlon(gps_x, gps_y)

    # UTM 좌표 차이를 계산
    delta_x = current_utm[0] - base_utm[0]  # UTM X 차이
    delta_y = current_utm[1] - base_utm[1]  # UTM Y 차이

    # PoseStamped 메시지의 위치 설정
    pose.pose.position.x = delta_x + 6.9 # 기준 GPS로부터의 X 거리
    pose.pose.position.y = delta_y + 1.18 # 기준 GPS로부터의 Y 거리
    pose.pose.position.z = 0.0  # 필요에 따라 조정

    # Heading을 도(degree)에서 라디안으로 변환
    heading_rad = math.radians(90 - msg.data)

    # Heading을 쿼터니언으로 변환
    quaternion = tf.transformations.quaternion_from_euler(0, 0, heading_rad)

    # PoseStamped 메시지의 orientation 값 설정
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    print(quaternion)
    # PoseStamped 메시지를 publish
    pose_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('heading_to_pose_publisher')

    # GPS 데이터 수신을 위한 구독자
    rospy.Subscriber('/RTK_GPS/smc_2000/fix', NavSatFix, gps_callback)
    # 헤딩 데이터 수신을 위한 구독자
    rospy.Subscriber('/KABOAT/Heading', Float32, heading_callback)

    # PoseStamped 메시지를 publish 하기 위한 퍼블리셔
    pose_pub = rospy.Publisher('/pose_stamped', PoseStamped, queue_size=10)

    # TF 변환 브로드캐스터 생성
    tf_broadcaster = tf.TransformBroadcaster()

    # 주기적으로 TF 변환을 publish
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # TF 변환을 publish
        tf_broadcaster.sendTransform(
            translation=(gps_x, gps_y, 0.0),
            rotation=(0, 0, 0, 0),
            time=rospy.Time.now(),
            child="vehicle",  # 차량의 프레임 이름
            parent="map"      # 기준 프레임 이름
        )
        rate.sleep()
