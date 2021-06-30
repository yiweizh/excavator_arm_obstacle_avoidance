#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import time
from motion_planning.msg import joint_angle_control

def talker():
    pub_p = rospy.Publisher('target_angles', joint_angle_control, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #array = np.array([0,0,0])

    while not rospy.is_shutdown():
        #array += 1
        joint_angle = joint_angle_control()
        joint_angle.joint_name = "base"
        joint_angle.angle = 15.0
        

        print("%s,%f"%(joint_angle.joint_name,joint_angle.angle))
        pub_p.publish(joint_angle)
        rate.sleep()

        
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
