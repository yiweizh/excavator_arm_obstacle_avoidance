#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import time

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

class PointCloudSubscriber(object):
    def __init__(self,topic="Obstacle_pointcloud"):
         
        self.topic = topic
        self.obstacle_pointcloud = rospy.wait_for_message(self.topic,PointCloud2) # store the pointcloud
        self.angle_subscribe = rospy.Subscriber(self.topic,PointCloud2,self.pcl_callback, queue_size = 1)
        print(self.topic + "Obstacle listener initialized")

    def get_pointcloud(self):
        # return [x,y,z] coordinates for pointcloud
        # to get each data point, use 
        # for p in xyz_pts:
        #    print('x: %f, y: %f, z: %f' % (p[0],p[1],p[2]))


        #print(self.obstacle_pointcloud)
        gen = point_cloud2.read_points(self.obstacle_pointcloud, field_names=("x", "y", "z"), skip_nans=True)
        xyz_pts = np.array([[p[0],p[1],p[2]] for p in gen])
        
        return xyz_pts

    def pcl_callback(self,data):
        self.obstacle_pointcloud = data


if __name__ == '__main__':
    rospy.init_node('PointCloudSubscriber', anonymous=True)
    angle_subscriber = PointCloudSubscriber("/rslidar_points")
    try:
        while not rospy.is_shutdown():
            xyz_pts = angle_subscriber.get_pointcloud()
            print(xyz_pts)
            

    except KeyboardInterrupt or rospy.ROSInterruptException:
        pass
