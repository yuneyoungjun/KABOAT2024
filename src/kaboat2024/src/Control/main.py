# -*- coding:utf-8 -*-
#!/usr/bin/env python3

import rospy
from modules.Waypoint import WaypointNavigator
from std_msgs.msg import Float32,Float64MultiArray
from modules.Pathplaning import Path
from modules.version import version
#########################aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaass


def callback(data):
    if version==0:
        waypoint_navigator = WaypointNavigator()
    elif version==1:
        waypoint_navigator = Path()
if __name__ == '__main__':
    try:
        if version==0:
            waypoint_navigator = WaypointNavigator()
        elif version==1:
            waypoint_navigator = Path()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass







# # -*- coding:utf-8 -*-
# #!/usr/bin/env python3

# import rospy
# from modules.Waypoint import WaypointNavigator
# from std_msgs.msg import Float32

# #########################aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaass
# def callback(data):
#     if data==0:
#         waypoint_navigator = WaypointNavigator()
        
# if __name__ == '__main__':
#     try:
#         rospy.init_node("Main")
#         ver_sub=rospy.Subscriber("version",Float32,callback)
#         waypoint_navigator = WaypointNavigator()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
