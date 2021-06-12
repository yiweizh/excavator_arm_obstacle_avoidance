#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import time
from motion_planning.msg import excavator_angles

def talker():
    pub_p = rospy.Publisher('target_angles', excavator_angles, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    array = np.array([0,0,0])
    while not rospy.is_shutdown():
        array += 1
        left_top = excavator_angles()
        left_top.angle0 = array[0]
        left_top.angle1 = array[1]
        left_top.angle2 = array[2]

        print("%f,%f,%f"%(left_top.angle0,left_top.angle1,left_top.angle2))
        pub_p.publish(left_top)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
