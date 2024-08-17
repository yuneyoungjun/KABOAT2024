#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
# from ublox_msgs.msg import NavRELPOSNED
from sensor_msgs.msg import Imu
import tf
import math
def getYaw(q):
    quaternion = (q.x, q.y, q.z, q.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = -1.0 * euler[2] * 180.0/math.pi
    return yaw
def callback(msg):
    data = Float32()
    data.data = getYaw(msg.orientation)
    data.data = float(getYaw(msg.orientation))
    if data.data > 180: data.data -= 360
    print("HEADING : ", data.data)
    pub.publish(data)
# def callback(msg):
#     heading = Float32()
#     head = float(msg.relPosHeading/100000)
#     if head > 180: head -= 360
#     print("HEADING : ", head)
#     heading.data = head

#     pub.publish(heading)
    

if __name__=="__main__":
    rospy.init_node("Heading_Node")
    pub = rospy.Publisher("KABOAT/Heading", Float32, queue_size=10)
    rospy.Subscriber("/android/imu",Imu, callback)
    rospy.spin()
    
    
    
# import rospy
# import tf
# from geometry_msgs.msg import QuaternionStamped
# from std_msgs.msg import Float32
# from math import pi


# def getYaw(q):
#     quaternion = (q.x, q.y, q.z, q.w)
#     euler = tf.transformations.euler_from_quaternion(quaternion)
#     yaw = -1.0 * euler[2] * 180.0/pi
#     return yaw

# def callback(msg):
#     data = Float32()
#     data.data = getYaw(msg.quaternion)
#     data.data = float(getYaw(msg.quaternion)/100000)
#     if data.data > 180: data.data -= 360
#     pub.publish(data)


# if __name__ == '__main__':
#     rospy.init_node('IMU_talker', anonymous=False)
#     pub = rospy.Publisher('IMUData', Float32, queue_size=100)
#     rospy.Subscriber('/filter/quaternion', QuaternionStamped, callback)
#     rospy.spin()
