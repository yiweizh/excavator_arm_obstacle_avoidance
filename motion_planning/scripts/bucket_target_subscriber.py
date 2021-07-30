#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import time

from geometry_msgs.msg import Point
import copy

class BucketTargetSubscriber(object):
    def __init__(self,topic="/bucket_target_point"):
        self.target_xyz = [0,0,0]
        self.topic = topic
        initial_point = rospy.wait_for_message(self.topic,Point)
        self.target_xyz = [initial_point.x,initial_point.y,initial_point.z]
        self.target_subscriber = rospy.Subscriber(self.topic,Point,self.target_callback, queue_size = 1)
        print(self.topic + "Bucket Target Subscriber initialized")

    def get_target(self):
        return copy.deepcopy(self.target_xyz)

    def target_callback(self,data):
        self.target_xyz[0] = data.x
        self.target_xyz[1] = data.y
        self.target_xyz[2] = data.z


if __name__ == '__main__':
    rospy.init_node('target_listener', anonymous=True)
    target_subscriber = BucketTargetSubscriber()
    try:
        while not rospy.is_shutdown():
            target = target_subscriber.get_target()
            print("base: %f, boom: %f, stick: %f"%(target[0],target[1],target[2]))
            time.sleep(0.001)

    except KeyboardInterrupt or rospy.ROSInterruptException:
        pass