#!/usr/bin/env python3

import rospy
import time
import math
from std_msgs.msg import Int16MultiArray

rospy.init_node('Arduino', anonymous=False)
pub = rospy.Publisher('control', Int16MultiArray, queue_size=100)
data = Int16MultiArray()

#                                   Data Info
# [servo_left, servo_right, servo_front, thrust_left, thrust_right, thrust_front]
# servo range : -45deg to 45deg
# thrust range : -500 to 500

i = 0
while(1):
    a = int(300 * math.sin(i))
    b = int(300 * math.cos(i))

    i += 0.1

    if(i > 5):
        a=b=0
    data.data = [0, 0, 0, 0, 0, a]
    print(data.data)
    pub.publish(data)
    time.sleep(0.1)