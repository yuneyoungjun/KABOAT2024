#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16MultiArray
import message_filters

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)

        # PWM 제어를 위한 퍼블리셔
        self.control_pub = rospy.Publisher('/control', Int16MultiArray, queue_size=10)

        # 희망 Heading 값을 구독
        self.desired_heading_sub=message_filters.Subscriber('/heading_error', Float32)
        self.Tau_X_sub=message_filters.Subscriber('/Tau_X', Float32)
        ts = message_filters.ApproximateTimeSynchronizer([self.desired_heading_sub, self.Tau_X_sub], queue_size=10, slop=0.1, allow_headerless=True)
        ts.registerCallback(self.callback)
        self.timepast=0
        self.psi_error_past=0
        self.psi_error_sum=0
        self.u_error_past=0
        self.u_error_sum=0

    def callback(self, psi_error_data,tau_X_data):
        global timepast, psi_error_past, psi_error_sum, u_error_past, u_error_sum, k
        self.timepast=0
        dt = rospy.get_time() - self.timepast
        psi_error=psi_error_data.data
        tau_X=tau_X_data.data
        control_values = Int16MultiArray()
        control_values.data = [0, 0, 0, 0, 0, 0]  # 초기값 설정

        u_error = 150

        ##### Parameter #####
        psi_p_gain = 30
        psi_d_gain = 0.5

        maxThrust = 1000
        minThrust = -1000

        ##########
        # rospy.loginfo("Error of psi : %f [degree]" , psi_error)

        psi_error_dot = (psi_error - self.psi_error_past) / dt
        self.psi_error_past = psi_error
        self.psi_error_sum += psi_error * dt

        tau_N = psi_p_gain * psi_error + psi_d_gain * psi_error_dot 
        




        Rpwm =  (tau_X * 1.0 + tau_N * -0.5)
        Lpwm =  (tau_X * 1.0 + tau_N * 0.5)
        LRpwm =  (tau_X * 0.0 + tau_N * -1.0)

        Lpwm = int(min(max(Lpwm, minThrust), maxThrust))
        Rpwm = int(min(max(Rpwm, minThrust), maxThrust))
        LRpwm = int(min(max(LRpwm, minThrust), maxThrust))
        


        control_values.data[3] = Lpwm
        control_values.data[4] = Rpwm
        control_values.data[5] = -LRpwm
        # rospy.loginfo("Motor speed(L) : %f " , Lpwm)    
        # rospy.loginfo("Motor speed(R) : %f " , Rpwm)
        # rospy.loginfo("Motor speed(LR) : %f " , LRpwm)
        # rospy.loginfo(control_values)
        self.control_pub.publish(control_values)
        self.timepast = rospy.get_time()













    # def callback(self, data):
    #     relative_angle = data.data  # 희망 Heading 값
    #     control_values = Int16MultiArray()
    #     control_values.data = [0, 0, 0, 0, 0, 0]  # 초기값 설정

    #     # 각도 차이를 -180 ~ 180 범위로 조정
    #     if relative_angle > 180:
    #         relative_angle -= 360
    #     elif relative_angle < -180:
    #         relative_angle += 360

    #     ##################################### Change #####################################
    #     # Calculate control values based on relative angle
    #     if relative_angle > 0:  # 오른쪽으로 회전
    #         control_values.data[3] = min(500, control_values.data[0] + int(relative_angle * 10))  # left
    #         control_values.data[4] = max(-500, control_values.data[1] - int(relative_angle * 10))  # right
    #     elif relative_angle < 0:  # 왼쪽으로 회전
    #         control_values.data[3] = max(-500, control_values.data[0] - int(-relative_angle * 10))  # left
    #         control_values.data[4] = min(500, control_values.data[1] + int(-relative_angle * 10))  # right

    #     # 모터 속도 증가
    #     control_values.data[3] = min(500, control_values.data[3] + 120)
    #     control_values.data[4] = min(500, control_values.data[4] + 120)
    #     ##################################################################################

    #     # PWM 값 퍼블리시
    #     print(relative_angle, control_values.data)
    #     self.control_pub.publish(control_values)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
