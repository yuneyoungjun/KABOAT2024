import rospy
from std_msgs.msg import Float32
from ublox_msgs.msg import NavRELPOSNED

def HEADcallback(msg):
    head = float(msg.relPosHeading/100000)
    if head > 180: head -= 360
    print(head)
    

if __name__=="__main__":
    rospy.init_node("Heading_Node")
    # pub = rospy.Publisher("/mbc_head", Float32, queue_size=10)
    rospy.Subscriber("/smc_2000/navrelposned",NavRELPOSNED, HEADcallback)
    rospy.spin()