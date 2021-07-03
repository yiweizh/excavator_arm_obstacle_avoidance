#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Author: Shiji Liu, Jinjie Liu, XUn Tu

# This script implements our method for planning a collision free path
# based on the detected obstacles in our environment
# Input: 
# Obstacles represented as an array of [x,y,z]

# Output:
# Next target for the excavator arm to reach


##### Imports #####
import rospy
import numpy as np
import time

from point_cloud_subscriber import PointCloudSubscriber # subscriber class for obstacles
from angle_subscriber import AngleSubscriber # subscriber class for real excavator angles

from motion_planning.msg import excavator_angles # data type for control command


##### End Imports #####

class ExcavatorMotionPlanning(object):
    def __init__(self):
        # Initialize the ExcavatorMotionPlanning class

        # set up ros publisher and listeners
        self.target_publisher = rospy.Publisher('target_angles', excavator_angles, queue_size=1) # publisher for selected target
        self.measured_angle_subscriber = AngleSubscriber(topic = 'measured_angles') # subscriber for excavator angles, will be used for selecting target
        self.obstacle_subscriber = PointCloudSubscriber("Obstacle_pointcloud") # subscriber for obstacles

        # set constants

        # tolerance
        self.middle_target_tolerance = 3 # how much angle difference we can tolerate for a middle target along a path
        self.end_target_tolerance = 1 # how much angle difference we can tolerate for the final target

        # bounds and counters for re-planning
        self.replanning_bound = 20 # how much time we should re-plan our path
        self.replanning_counter = 0


        # storage variables
        self.bucket_goal = [] # goal for the bucket to reach, in terms of excavator coordinate system
        self.current_path = [] # a series of goal points for our excavator arm to reach

        pass


    # function for setting final goal for the excavator bucket
    def set_bucket_goal(self,bucket_goal):
        # Interface for users to set the goal for the bucket to reach
        # bucket_goal in the format [x,y,z]
        self.bucket_goal = bucket_goal
        pass


    # function for planning a collision free path
    def path_planning(self):
        # plan a collision free path
        # the algorithm we will use for path planning hasn't been decided
        pass


    # function for selecting next target for the excavator to reach
    def get_next_target(self):
        # based on the planned path, select the next target and publish the target
        # call re-planning if necessary
        pass

    # help functions:

    # forward kinematics
    def forward_kinematics(self):
        pass

    # inverse kinematics
    def inverse_kinematics(self):
        pass

    # check collision
    def check_collision(self,configuration):
        pass

    # 

if __name__ == '__main__':
    rospy.init_node('ExcavatorMotionPlanning', anonymous=True)
    excavator_motion_planning = ExcavatorMotionPlanning()
    try:
        while not rospy.is_shutdown():
            xyz_pts = excavator_motion_planning.get_next_target()
            
            

    except KeyboardInterrupt or rospy.ROSInterruptException:
        pass