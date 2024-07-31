#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
import utm
from std_msgs.msg import Float64MultiArray



def callback(data):
    rospy.loginfo((data.longitude, data.latitude))
    gps_rawdata=[data.longitude,data.latitude]


    utm_x, utm_y,_,_ = utm.from_latlon(data.latitude,data.longitude)
    rospy.loginfo((utm_x, utm_y))
    UTMdata = Float64MultiArray()
    UTMdata.data=[utm_x, utm_y]
    pub.publish(UTMdata)
    



def listener():
    rospy.init_node('Gps_Utm', anonymous=True)
    sub1=rospy.Subscriber('/smc_2000/fix', NavSatFix,callback)
    rospy.spin()
    
    

if __name__ == '__main__':
    pub = rospy.Publisher('UTM', Float64MultiArray, queue_size=10)
    listener()
    rospy.spin()