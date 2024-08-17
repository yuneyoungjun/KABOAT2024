#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16MultiArray

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)

        # PWM 제어를 위한 퍼블리셔
        self.control_pub = rospy.Publisher('/control', Int16MultiArray, queue_size=10)

        # 희망 Heading 값을 구독
        rospy.Subscriber('/desired_heading', Float32, self.heading_callback)

    def heading_callback(self, data):
        relative_angle = data.data  # 희망 Heading 값
        control_values = Int16MultiArray()
        control_values.data = [0, 0, 0, 0, 0, 0]  # 초기값 설정

        # 각도 차이를 -180 ~ 180 범위로 조정
        if relative_angle > 180:
            relative_angle -= 360
        elif relative_angle < -180:
            relative_angle += 360

        ##################################### Change #####################################
        # Calculate control values based on relative angle
        if relative_angle > 0:  # 오른쪽으로 회전
            control_values.data[3] = min(500, control_values.data[0] + int(relative_angle * 10))  # left
            control_values.data[4] = max(-500, control_values.data[1] - int(relative_angle * 10))  # right
        elif relative_angle < 0:  # 왼쪽으로 회전
            control_values.data[3] = max(-500, control_values.data[0] - int(-relative_angle * 10))  # left
            control_values.data[4] = min(500, control_values.data[1] + int(-relative_angle * 10))  # right

        # 모터 속도 증가
        control_values.data[3] = min(500, control_values.data[3] + 120)
        control_values.data[4] = min(500, control_values.data[4] + 120)
        ##################################################################################

        # PWM 값 퍼블리시
        print(relative_angle, control_values.data)
        self.control_pub.publish(control_values)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
