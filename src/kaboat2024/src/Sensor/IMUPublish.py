#!/usr/bin/env python3
# Quaternion 값으로 들어오는 IMU의 정보를 Yaw값으로 변환시켜주는 코드
# Quaternion => Yaw(Degree)

import rospy
import tf
from std_msgs.msg import Float32
from math import pi
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped


def getYaw(q):
    quaternion = (q.x, q.y, q.z, q.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = -1.0 * euler[2] * 180.0/pi
    return yaw

def callback(msg):
    data = Float32()
    data.data = getYaw(msg.orientation)
    # data.data = getYaw(msg.quaternion)
    print("Psi : {0:0.1f}" .format(data.data))
    pub.publish(data)


if __name__ == '__main__':
    rospy.init_node("Heading_Node")
    pub = rospy.Publisher('KABOAT/Heading', Float32, queue_size=100)
    # rospy.Subscriber('/filter/quaternion', QuaternionStamped, callback)
    rospy.Subscriber('/imu/data', Imu, callback)
    rospy.spin()