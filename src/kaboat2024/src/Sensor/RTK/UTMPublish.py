#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
import utm
from std_msgs.msg import Float64MultiArray
import RTKSetting

ref_x = RTKSetting.ref_utm_x
ref_y = RTKSetting.ref_utm_y

def callback(data):
    utm_x, utm_y, _, _ = utm.from_latlon(data.latitude, data.longitude)
    
    # 상대 좌표 계산
    relative_x = utm_x - ref_x
    relative_y = utm_y - ref_y
    
    print("GPS : ", data.longitude, data.latitude)
    print("UTM : ", utm_x, utm_y)
    print("Relative UTM : ", relative_x, relative_y)
    print()
    
    UTMdata = Float64MultiArray()
    UTMdata.data = [relative_x, relative_y]  # 상대 좌표 저장
    pub.publish(UTMdata)

def listener():
    rospy.init_node('GPStoUTM_Node', anonymous=True)
    sub1 = rospy.Subscriber('RTK_GPS/smc_2000/fix', NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('KABOAT/UTM', Float64MultiArray, queue_size=10)
    listener()
