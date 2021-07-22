#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from point_cloud_publisher import float_to_rgb
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


    def get_pointcloud_with_color(self, x_lower, x_upper, y_lower, y_upper, z_lower, z_upper):
        gen = point_cloud2.read_points(self.obstacle_pointcloud, field_names=("x", "y", "z","rgba"), skip_nans=True)
        
        pointcloud_pt_rgba = []
        
        for pt in gen:
            if pt[0] >= x_lower and pt[0] <= x_upper and pt[1] >= y_lower and pt[1] <= y_upper and pt[2] >= z_lower and pt[2] <= z_upper:
                r,g,b,a = self.hex2rgba(pt[3])
                pointcloud_pt_rgba.append([pt[0],pt[1],pt[2],r,g,b,a])

        return pointcloud_pt_rgba
    
    def get_pointcloud_with_rgb(self, x_lower, x_upper, y_lower, y_upper, z_lower, z_upper):
        gen = point_cloud2.read_points(self.obstacle_pointcloud, field_names=("x", "y", "z","rgba"), skip_nans=True)
        
        pointcloud_pt_rgb = []
        
        for pt in gen:
            if pt[0] >= x_lower and pt[0] <= x_upper and pt[1] >= y_lower and pt[1] <= y_upper and pt[2] >= z_lower and pt[2] <= z_upper:
                rgb = float_to_rgb(pt[3])
                pointcloud_pt_rgb.append([pt[0],pt[1],pt[2], rgb[0], rgb[1], rgb[2]])

        return pointcloud_pt_rgb

    def hex2rgba(self,rgba):
        b = rgba % 256
        
        rgba /= 256
        g = rgba % 256
        rgba /= 256
        r = rgba % 256

        a = rgba / 256

        return r,g,b,a


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
