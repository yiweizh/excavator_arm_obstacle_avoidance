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

        #print(new_data)

        self.header.stamp = rospy.Time.now()
        pc2 = point_cloud2.create_cloud(self.header, self.fields, new_data)

        self.pub_p.publish(pc2)

def generate_pts_from_bounding_box(right_down_pt, lwh, num_pt_per_direction = 10):
    data = []

    # right face, y unchanged
    y_r = right_down_pt[1]
    # left face, y unchanged
    y_l = right_down_pt[1] + lwh[1]
    for ii in range(num_pt_per_direction):
        x = right_down_pt[0] + lwh[0] / num_pt_per_direction * ii
        for jj in range(num_pt_per_direction):
            z = right_down_pt[2] + lwh[2] / num_pt_per_direction * jj
            data.append([x,y_r,z])
            data.append([x,y_l,z])
    
    # front face, x unchanged
    x_f = right_down_pt[0]
    # back face, x unchanged
    x_b = right_down_pt[0] + lwh[0]
    for ii in range(num_pt_per_direction):
        y = right_down_pt[1] + lwh[1] / num_pt_per_direction * ii
        for jj in range(num_pt_per_direction):
            z = right_down_pt[2] + lwh[2] / num_pt_per_direction * jj
            data.append([x_f,y,z])
            data.append([x_b,y,z])
    
    # upper face, z unchanged
    z_u = right_down_pt[2] + lwh[2]
    # bottom face, z unchanged
    z_b = right_down_pt[2]
    for ii in range(num_pt_per_direction):
        x = right_down_pt[0] + lwh[0] / num_pt_per_direction * ii
        for jj in range(num_pt_per_direction):
            y = right_down_pt[1] + lwh[1] / num_pt_per_direction * jj
            data.append([x,y,z_u])
            data.append([x,y,z_b])
    
    return data

if __name__ == '__main__':
    rospy.init_node('PointCloudPublisher', anonymous=True)

    point_cloud_publisher = PointCloudPublisher(topic = 'Obstacle_pointcloud',color = 'b')
    data0 = generate_pts_from_bounding_box([0.2,0.5,0.0],[0.1,0.15,0.4],10)
    data1 = generate_pts_from_bounding_box([0.2,0.2,0.4],[0.1,0.15,0.4],10)
    data2 = generate_pts_from_bounding_box([0.2,-0.4,0.4],[0.1,0.15,0.4],10)
    data3 = generate_pts_from_bounding_box([0.2,-0.7,0.0],[0.1,0.15,0.4],10)
    data5 = generate_pts_from_bounding_box([0.2,-0.4,0.5],[0.1,0.15,0.4],10)
    # counter = 0
    # num_of_pts = 500
    #data = data0 + data1 + data2 + data3
    #data = data0 + data1 + data3 + data5
    #data = data0  + data3
    data = data3
    while not rospy.is_shutdown():
        # data.append([counter,counter,counter])
        # counter += 1
        # if counter >= num_of_pts:
        #     break

        point_cloud_publisher.publish(data)
        rospy.sleep(0.1)