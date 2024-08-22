#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import cv_bridge

def callback(data):
    try:
        # CvBridge를 사용하여 ROS CompressedImage 메시지를 OpenCV 이미지로 변환
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # 이미지를 윈도우에 표시
        cv2.imshow('Compressed Image', image_np)
        cv2.waitKey(1)  # OpenCV 창을 유지하기 위해 필요
    except Exception as e:
        rospy.logerr("Error converting CompressedImage to OpenCV image: %s", str(e))

def listener():
    rospy.init_node('image_listener', anonymous=True)

    # '/camera/image/compressed' 토픽 구독
    rospy.Subscriber("/Imageaaa", CompressedImage, callback)

    # ROS 이벤트 루프
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 프로그램 종료 시 모든 OpenCV 창 닫기
        cv2.destroyAllWindows()
