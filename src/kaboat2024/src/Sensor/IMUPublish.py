#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from ublox_msgs.msg import NavRELPOSNED

def callback(msg):
    heading = Float32()
    head = float(msg.relPosHeading/100000)
    if head > 180: head -= 360
    print("HEADING : ", head)
    heading.data = head

    pub.publish(heading)
    

if __name__=="__main__":
    rospy.init_node("Heading_Node")
    pub = rospy.Publisher("KABOAT/Heading", Float32, queue_size=10)
    rospy.Subscriber("RTK_GPS/smc_plus/navrelposned",NavRELPOSNED, callback)
    rospy.spin()
