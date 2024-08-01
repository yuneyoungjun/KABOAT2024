#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import utm
import RTK.RTKSetting

def callback(data):
    # 클릭한 포인트를 UTM으로 변환
    utm_x, utm_y, _, _ = utm.from_latlon(data.point.y, data.point.x)  # 데이터의 y가 위도, x가 경도

    # 상대 좌표 계산
    relative_x = utm_x - RTK.RTKSetting.ref_utm_x
    relative_y = utm_y - RTK.RTKSetting.ref_utm_y

    # 새로운 PointStamped 생성
    relative_point = PointStamped()
    relative_point.header.stamp = rospy.Time.now()
    relative_point.point.x = relative_x  # 상대 x 좌표
    relative_point.point.y = relative_y  # 상대 y 좌표
    relative_point.point.z = 0  # z 좌표 (0으로 설정)

    # 상대 좌표를 퍼블리시
    relative_pub.publish(relative_point)

    # 새로운 마커 생성
    marker = Marker()
    marker.header.frame_id = "wgs84"  # 프레임 ID 설정
    marker.header.stamp = rospy.Time.now()
    marker.ns = "points" + str(len(point_list.markers))  # 네임스페이스
    marker.id = len(point_list.markers)  # 고유 ID
    marker.type = Marker.SPHERE  # 마커 타입: 구
    marker.action = Marker.ADD  # 추가 동작
    marker.pose.position.x = data.point.x  # 포인트 x 좌표
    marker.pose.position.y = data.point.y  # 포인트 y 좌표
    marker.pose.position.z = 0  # z 좌표 (0으로 설정)
    marker.scale.x = 2  # 크기 설정
    marker.scale.y = 2
    marker.scale.z = 2
    marker.color.a = 0.8  # 완전 불투명
    marker.color.r = 0.0  # 빨간색
    marker.color.g = 0.8
    marker.color.b = 0.0

    # Marker를 MarkerArray에 추가
    point_list.markers.append(marker)

    # MarkerArray 퍼블리시
    pub.publish(point_list)
    print(f"Current markers count: {len(point_list.markers)}")  # 현재 마커 수 출력

if __name__ == "__main__":
    rospy.init_node("Point_Node")
    rospy.Subscriber("/mapviz/clicked_point", PointStamped, callback)

    point_list = MarkerArray()  # MarkerArray 초기화
    pub = rospy.Publisher('/visual_rviz', MarkerArray, queue_size=10)  # 마커 퍼블리셔 생성
    relative_pub = rospy.Publisher('/Waypoint', PointStamped, queue_size=10)  # 상대 좌표 퍼블리셔 생성
    
    rospy.spin()  # 노드가 종료될 때까지 대기
