#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import time
from motion_planning.msg import excavator_control_signals # msg type for publishing data
from angle_subscriber import AngleSubscriber # get target excavator joint angles

def SimpleController():
    pub_p = rospy.Publisher('excavator_control', excavator_control_signals, queue_size=1)
    rospy.init_node('ExcavatorSimpleController', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #array = np.array([0,0,0])

    # get the target excavator joint angles
    target_angle_subscriber = AngleSubscriber(topic = 'target_angles')
    measured_angle_subscriber = AngleSubscriber(topic = 'measured_angles')


    while not rospy.is_shutdown():
        #array += 1
        # get the current targets
        target_angles = target_angle_subscriber.get_angles()
        base_target = target_angles[0]
        boom_target = target_angles[1]
        stick_target = target_angles[2]

        # get the real angles
        measured_angles = measured_angle_subscriber.get_angles()
        base_measured = measured_angles[0]
        boom_measured = measured_angles[1]
        stick_measured = measured_angles[2]

        # create msg to pubslish
        control_signals = excavator_control_signals()


        # calculate the control signals
        # skip for a moment


        control_signals.stick_servo = 45;
        control_signals.boom_servo = 135;
        control_signals.pump_mode = 1;
        control_signals.base_pwm = 4.5;
        

        
        pub_p.publish(control_signals)
        rate.sleep()

        
 
if __name__ == '__main__':
    try:
        SimpleController()
    except rospy.ROSInterruptException:
        pass
