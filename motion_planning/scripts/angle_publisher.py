#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import time
from motion_planning.msg import excavator_angles
import serial





def talker():
    pub_p = rospy.Publisher('measured_angles', excavator_angles, queue_size=1)
    rospy.init_node('measured_angles', anonymous=True)
    rate = rospy.Rate(50) # 50hz

    #ser = serial.Serial(port='/dev/ttyUSB0',baudrate=9600)
    left_top = excavator_angles()

    s = serial.Serial(port='/dev/ttyUSB0',baudrate=9600)
    
    while not rospy.is_shutdown():
        s.flushInput()
        raw_data = s.readline()
        raw_data = s.readline()
        if raw_data:
            data = raw_data.split(',')
            #print(data)

            left_top.angle_stick = float(data[2])
            left_top.angle_boom = float(data[1])
            left_top.angle_base = float(data[3].split('\r')[0])         

                
                
            
            

        print("%f,%f,%f"%(left_top.angle_stick,left_top.angle_boom,left_top.angle_base))
        pub_p.publish(left_top)
        #rate.sleep()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
