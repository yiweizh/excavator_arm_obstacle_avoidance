#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import time

from motion_planning.msg import excavator_angles

class AngleSubscriber(object):
    def __init__(self,topic="measured_angles"):
        self.angles = [0,0,0]
        self.topic = topic
        rospy.wait_for_message(self.topic,excavator_angles)
        self.angle_subscribe = rospy.Subscriber(self.topic,excavator_angles,self.angle_callback, queue_size = 1)
        print(self.topic + " Angle Subscriber initialized")

    def get_angles(self):
        return self.angles

    def angle_callback(self,data):
        self.angles[0] = data.angle_base
        self.angles[1] = data.angle_boom
        self.angles[2] = data.angle_stick


if __name__ == '__main__':
    rospy.init_node('angle_listener', anonymous=True)
    angle_subscriber = AngleSubscriber()
    try:
        while not rospy.is_shutdown():
            angles = angle_subscriber.get_angles()
            print("base: %f, boom: %f, stick: %f"%(angles[0],angles[1],angles[2]))
            time.sleep(0.001)

    except KeyboardInterrupt or rospy.ROSInterruptException:
        pass
