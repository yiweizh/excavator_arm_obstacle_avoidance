#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np


import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class PointCloudPublisher(object):
    def __init__(self,topic = "/excavator_arm_pts",color = 'r'):
        self.pub_p = rospy.Publisher(topic, PointCloud2, queue_size=1)

        self.header = Header()
        self.header.frame_id = "map"

        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1),
                       # PointField('rgb', 12, PointField.UINT32, 1),
                       PointField('rgba', 12, PointField.UINT32, 1),
                       ]

        if color == 'r':
            r = 255
            g = 0
            b = 0
            a = 255
        elif color == 'g':
            r = 0
            g = 255
            b = 0
            a = 255
        elif color == 'b':
            r = 0
            g = 0
            b = 255
            a = 255
        else:
            r = 255
            g = 255
            b = 0
            a = 255

        self.rgba = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

    def publish(self,data_3d):
        # data_3d is a list of [x,y,z]
        new_data = []
        for data in data_3d:
            new_data.append(data + [self.rgba])

        print(new_data)

        self.header.stamp = rospy.Time.now()
        pc2 = point_cloud2.create_cloud(self.header, self.fields, new_data)

        self.pub_p.publish(pc2)


if __name__ == '__main__':
    rospy.init_node('PointCloudPublisher', anonymous=True)

    point_cloud_publisher = PointCloudPublisher()
    data = []
    counter = 0
    num_of_pts = 500

    while not rospy.is_shutdown():
        data.append([counter,counter,counter])
        counter += 1
        if counter >= num_of_pts:
            break

        point_cloud_publisher.publish(data)
        rospy.sleep(0.1)