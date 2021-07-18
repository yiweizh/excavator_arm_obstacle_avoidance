#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MarkerPublisher(object):
    def __init__(self,topic = 'collision_free_path',type = 1,id = 0,color = 'r',scale = 0.2):
        self.types = [Marker.POINTS, Marker.LINE_STRIP, Marker.LINE_LIST]
        self.pub = rospy.Publisher(topic, Marker, queue_size=10)

        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = topic
        self.marker.pose.orientation.w = 1.0
        self.marker.type = self.types[type]
        self.marker.action = Marker.ADD

        if type == 0:
            # point
            self.marker.scale.x = scale
            self.marker.scale.y = scale

        elif type == 1:
            self.marker.scale.x = scale

        else:
            self.marker.scale.x = scale


        self.marker.color.a = 1.0
        if color == 'r':
            self.marker.color.r = 1.0
            self.marker.color.g = 0.0
            self.marker.color.b = 0.0
        elif color == 'g':
            self.marker.color.r = 0.0
            self.marker.color.g = 1.0
            self.marker.color.b = 0.0
        elif color == 'b':
            self.marker.color.r = 0.0
            self.marker.color.g = 0.0
            self.marker.color.b = 1.0

        self.published = False

    def publish(self,pt_array):
        # pt_array is an array of 3D points
        self.marker.points = []
        self.marker.header.stamp = rospy.Time.now()

        for pt in pt_array:
            new_pt = Point(pt[0],pt[1],pt[2])
            self.marker.points.append(new_pt)
        
        #if self.published:
        #    self.marker.action = Marker.
        self.pub.publish(self.marker)


if __name__ == '__main__':
    rospy.init_node('MarkerPublisher', anonymous=True)

    marker_publisher0 = MarkerPublisher(topic = 'obstacle_0',type = 1,id = 0,scale = 0.1)
    marker_publisher1 = MarkerPublisher(topic = 'obstacle_1',type = 1,id = 1,scale = 0.1)
    marker_publisher2 = MarkerPublisher(topic = 'obstacle_2',type = 1,id = 2,scale = 0.1)
    marker_publisher3 = MarkerPublisher(topic = 'obstacle_3',type = 1,id = 3,scale = 0.1)

    # data = np.array([[0,0,0],[1,1,1],[2,2,2],[2,2,4],[2,4,4],[4,4,4]])
    # counter = 0
    # num_of_pts = 100

    data0 = np.array([[0.2,0.7,0.0],[0.2,0.5,0.0],[0.2,0.5,0.4],[0.2,0.7,0.4],[0.2,0.7,0.0]])

    data1 = np.array([[0.2,0.3,0.8],[0.2,0.1,0.8],[0.2,0.1,0.4],[0.2,0.3,0.4],[0.2,0.3,0.8]])

    data2 = np.array([[0.2,-0.1,0.8],[0.2,-0.3,0.8],[0.2,-0.3,0.4],[0.2,-0.1,0.4],[0.2,-0.1,0.8]])

    data3 = np.array([[0.2,-0.7,0.0],[0.2,-0.5,0.0],[0.2,-0.5,0.4],[0.2,-0.7,0.4],[0.2,-0.7,0.0]])


    while not rospy.is_shutdown():

        # counter += 1
        # if counter >= num_of_pts:
        #     break

        
        marker_publisher0.publish(data0)
        marker_publisher1.publish(data1)
        marker_publisher2.publish(data2)
        marker_publisher3.publish(data3)
        rospy.sleep(0.1)