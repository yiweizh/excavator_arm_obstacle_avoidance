#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import time

from motion_planning.msg import excavator_control_signals

class AngleSubscriber(object):
    def __init__(self):
        self.angles = excavator_control_signals()
        self.angle_subscribe = rospy.Subscriber('measured_angles',excavator_control_signals,self.angle_callback)

    def get_angles(self):
        return self.angles

    def angle_callback(self,data):
        self.angles.joint_name = data.joint_name
        self.angles.angle = data.angle
        


if __name__ == '__main__':
    rospy.init_node('angle_listener', anonymous=True)
    angle_subscriber = AngleSubscriber()
    try:
        while not rospy.is_shutdown():
            new_angle = angle_subscriber.get_angles()

            #if joint_angle != new_angle:
            #    print("joint: %s, angle: %f"%(new_angle.joint_name,new_angle.angle))

            print("joint: %s, angle: %f"%(new_angle.joint_name,new_angle.angle))

            joint_angle = new_angle

            time.sleep(0.001)

    except KeyboardInterrupt or rospy.ROSInterruptException:
        pass
