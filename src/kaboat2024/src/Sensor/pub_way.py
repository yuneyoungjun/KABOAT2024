#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import PointStamped,Point

def callback(GPS):
    gps=list(GPS.data)
    print(gps)
    point=Point(x=gps[1],y=gps[0]+10)
    des=PointStamped(point=point)
    des_pub.publish(des)

def listener():
    rospy.init_node('pub_des', anonymous=True)

    gps_sub =  rospy.Subscriber("/KABOAT/UTM", Float64MultiArray,callback)
    rospy.spin()


if __name__ == '__main__':
    des_pub = rospy.Publisher('/pub_waypoint', PointStamped, queue_size=10)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    print('sadssegfrewfs')