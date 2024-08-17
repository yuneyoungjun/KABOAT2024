#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix,Imu
import utm
from std_msgs.msg import Float64MultiArray
import RTKSetting

ref_x = RTKSetting.ref_utm_x
ref_y = RTKSetting.ref_utm_y
UTMdata = Float64MultiArray()
PreUtm=[]
i=0
def callback(data):
    global UTMdata,PreUtm,i
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
    PreUtm.append(UTMdata.data)
    if len(PreUtm)>=3:
        PreUtm.pop(0)
    i=0
def callback_2(data):
    global i
    i+=1
    utmdata=Float64MultiArray()
    if len(PreUtm)==2 and i<200:
        utmdata.data=[float(UTMdata.data[0]+i*0.005*(PreUtm[1][0]-PreUtm[0][0])),float(UTMdata.data[1]+i*0.005*(PreUtm[1][1]-PreUtm[0][1]))]
    else:
        utmdata=UTMdata
    print("Relative UTM : ", utmdata.data)
    pub.publish(utmdata)

def listener():
    rospy.init_node('GPStoUTM_Node', anonymous=True)
    sub1 = rospy.Subscriber('/android/fix', NavSatFix, callback)
    sub2 = rospy.Subscriber('/android/imu', Imu, callback_2)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('KABOAT/UTM', Float64MultiArray, queue_size=10)
    listener()
