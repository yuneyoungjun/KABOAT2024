#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix,Imu
import tf
import math

# 기준 GPS 위치
ref_gps_x = 36.368706  # 기준 위도
ref_gps_y = 127.345411  # 기준 경도

gps_x = 0.0
gps_y = 0.0
quaternion = (0.0, 0.0, 0.0, 1.0)
i=0
preX=0
preY=0
XList=[]
YList=[]
def gps_callback(gps_msg):
    global gps_x, gps_y,i,preX,preY,XList,YList
    # GPS 데이터에서 위치 정보를 가져옵니다.
    i=0
    gps_x = gps_msg.longitude  # 위도를 y로 사용
    gps_y = gps_msg.latitude   # 경도를 x로 사용
    XList.append(gps_x)
    YList.append(gps_y)
    if len(XList)>=3:
        XList.pop(0)
        YList.pop(0)
    preX=gps_x
    preY=gps_y
def gps_callback_2(msg):
    global gps_x, gps_y,i,preX,preY,XList,YList
    if len(XList)==2 and i<150:
        gps_x=preX+i*0.01*(XList[1]-XList[0])
        gps_y=preY+i*0.01*(YList[1]-YList[0])
    else:
        gps_x=preX
        gps_y=preY
    i+=1


def heading_callback(msg):
    global quaternion
    # PoseStamped 메시지 생성
    pose = PoseStamped()
    pose.header.frame_id = "/wgs84"  # GPS 데이터의 기준 프레임
    pose.header.stamp = rospy.Time.now()
    # PoseStamped 메시지의 위치 설정
    pose.pose.position.x = gps_x # 기준 GPS로부터의 X 거리
    pose.pose.position.y = gps_y # 기준 GPS로부터의 Y 거리
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

    # PoseStamped 메시지를 publish
    pose_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('heading_to_pose_publisher')

    # GPS 데이터 수신을 위한 구독자
    ####rospy.Subscriber('/RTK_GPS/smc_2000/fix', NavSatFix, gps_callback)
    rospy.Subscriber('/android/fix', NavSatFix, gps_callback)
    # 헤딩 데이터 수신을 위한 구독자
    rospy.Subscriber('/KABOAT/Heading', Float32, heading_callback)
    rospy.Subscriber('/android/imu', Imu, gps_callback_2)

    # PoseStamped 메시지를 publish 하기 위한 퍼블리셔
    pose_pub = rospy.Publisher('/pose_stamped', PoseStamped, queue_size=10)
    rospy.spin()
