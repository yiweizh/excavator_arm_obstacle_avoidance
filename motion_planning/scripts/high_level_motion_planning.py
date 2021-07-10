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

from marker_publisher import MarkerPublisher # publishers for excavator points, collision free path and bucker target


import fibheap
from joint_space_representation import JointSpace
from sklearn.neighbors import KDTree
##### End Imports #####

class ExcavatorMotionPlanning(object):
    def __init__(self):
        # Initialize the ExcavatorMotionPlanning class

        # set up ros publisher and listeners
        self.target_publisher = rospy.Publisher('target_angles', excavator_angles, queue_size=1) # publisher for selected target
        self.measured_angle_subscriber = AngleSubscriber(topic = 'measured_angles') # subscriber for excavator angles, will be used for selecting target
        self.obstacle_subscriber = PointCloudSubscriber("Obstacle_pointcloud") # subscriber for obstacles

        # set constants
        self.collision_distance = 0.2 # unit: m
        self.close_distance = 0.3 # unit: m

        # tolerance
        self.middle_target_tolerance = 3 # how much angle difference we can tolerate for a middle target along a path
        self.end_target_tolerance = 1 # how much angle difference we can tolerate for the final target

        # bounds and counters for re-planning
        self.replanning_bound = 20 # how much time we should re-plan our path
        self.replanning_counter = 0


        # storage variables
        self.bucket_goal = [] # goal for the bucket to reach, in terms of excavator coordinate system， [x,y,z]
        self.current_path = [] # a series of goal points for our excavator arm to reach [angle_base,angle_boom,angle_stick]


        # flags
        self.visualize = True # whether visualize data or not

        # instances for visualization
        self.excavator_pts_visualization = MarkerPublisher(topic = '/excavator_arm_pts', type = 0,id = 0,color = 'b') # publisher for pts on the excavator
        self.path_visualization = MarkerPublisher('/collision_free_path',type = 1,id = 1,color = 'g') # publish as as a line strip
        self.target_visualization = MarkerPublisher('/bucket_target',type = 0,id = 0,color = 'r') # publish the target of the bucket

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

        # use self variables:
        # self.bucket_goal (utilize)
        # self.current_path = [] (modify)

        # use function:
        # self.check_collision


        # idea 1: call self.astar() method


        pass


    def astar(self):

        # get the obstacles
        obstacle = obstacle_subscriber.get_pointcloud()

        # form a KDtree of the obstacles
        self.obstacle_tree = KDTree(np.array(obstacle))

        # transform the goal into cell coordinate
        goal_cell_coordinate = self.inverse_kinematics(self.bucket_goal)

        # joint space
        joint_space = JointSpace(goal_cell_coordinate)


        # open list
        open_list = fibheap.Fheap() # use a heap to store the open list
        
        # closed list
        closed_coordinate_list = [] # store the cell-coordinate of the expanded nodes

        # get the joint coordinate of the current pos
        current_joint_coordinate = self.measured_angle_subscriber.get_angles()

        # get the init cell
        init_cell = joint_space.get_cell(current_joint_coordinate)

        # get the init cell coordinate
        init_cell_coordinate = init_cell.get_joint_space_pos()

        # calculate the heuristic value of the init cell
        init_h = self.heuristic_function(init_cell_coordinate,goal_cell_coordinate)
        
        # set the g value to the init cell
        init_cell.set_cost_to_come(0) # no cost from init
        
        # create a node in init cell
        init_cell.node = fibheap.Node([0 + init_h, init_cell_coordinate])

        # put the init_cell into the open_list
        open_list.insert(init_cell.node)

        # whether a result is found
        path_found = False

        # loop for ASTAR search
        while True:
            # check if the open_list is empty, if true, break
            #if self.open_list == []:
            if open_list.num_nodes == 0:
                break

            new_item = open_list.extract_min().key # get the first item, and remove it from the open list
            cell_coordinate = new_item[1] # get the cell coordinate

            # get the cell corresponding to the coordinate, retrieve world coordinate, and mark the cell as no in open_list
            new_cell = joint_space.get_cell(cell_coordinate)
            new_gs = new_cell.get_cost_to_come()
            new_world_coordinate = new_cell.get_joint_space_pos()
            new_cell.in_open_list = False

            # check whether goal has been reached
            if joint_space.is_goal(cell_coordinate): # goal has been reached
                path_cost = new_gs + self.cost_function(cell_coordinate,goal_cell_coordinate)
                path_found = True
                break

            # expand the current node, check whether the gscore will be smaller for the current node's unexplored neighbor when going from this node
            successor_list = joint_space.expand(cell_coordinate)

            for s_prime in successor_list:
                # get the cell corresponding to the coordinate s', and 
                successor_cell = joint_space.get_cell(s_prime)

                collision = successor_cell.has_collided()
                if collision == 2: # colision condition unknown, apply check collision methods to check
                    collision, min_distance = self.check_collision(s_prime)
                    # set the collision to the cell
                    successor_cell.set_collision(collision)
                    successor_cell.set_min_distance_to_obstacle(min_distance)
                
                if collision == 1:
                    continue # if this cell is colliding, directly go to the next

                min_distance = successor_cell.get_min_distance_to_obstacle()

                # use the cost function to calculate the distance between the current node and the successor
                cost_s_sprime = self.cost_function(cell_coordinate, s_prime, min_distance)

                #get the g(s) 
                successor_gs = successor_cell.get_cost_to_come()
                
                # check whether g(s) + c(s, s') < g(s')
                temp_gs = new_gs + cost_s_sprime

                if temp_gs < successor_gs:
                    successor_cell.set_cost_to_come(temp_gs) # set new cost to come to the successor cell
                    successor_cell.set_parent(cell_coordinate) # set the current cell as parent

                    # calculate the heuristic value for the successor cell
                    h_s_prime = self.heuristic_function(s_prime,goal_cell_coordinate)
                    
                    # check whether the neighbor has been added into the open_list
                    if successor_cell.inside_open_list():
                        # the cell has been put into the open list, directly decrease key
                        self.open_list.decrease_key(successor_cell.node,[temp_gs + h_s_prime,s_prime]) # decrease key
                        
                    else:
                        # mark the cell as in open list
                        successor_cell.in_open_list = True
                        # create a node for the cell
                        successor_cell.node = fibheap.Node([temp_gs + h_s_prime,s_prime])
                        # add the node to open list
                        self.open_list.insert(successor_cell.node)



            # finish the loop for A*, check whether the route has been found
            if path_found:
                # found path, back track
                current_best_path = []
                last_cell_pos = goal_cell_coordinate

                # loop to backtrack
                while True:
                    # retrieve the cell by cell coordinate
                    last_cell = joint_space.get_cell(last_cell_pos)
                    current_best_path.append(last_cell.get_joint_space_pos()) # store the world pos of the cell
                        
                    # check if the initial cell has been reached
                    if last_cell_pos == init_cell_coordinate:
                        break
                        
                    # get the parent cell, if the parent is empty, we have reached the initial value
                    parent_cell_pos = last_cell.get_parent()

                    # set the parent to last
                    last_cell_pos = parent_cell_pos

                # reverse the order of the path
                current_best_path.reverse()
                self.collision_free_path = current_best_path

            else:
                print("Failed to find a feasible path under current cell width")
                self.collision_free_path = []
            


    def heuristic_function(self,current_cell_coordinate, goal_cell_coordinate):
        # idea 1:
        # use l1 distance in joint space, 3D

        return self.l1_cost(current_cell_coordinate,goal_cell_coordinate)
        
    def cost_function(self,current_cell_coordinate,goal_cell_coordinate, min_distance = 0):
        # cost function consists of 3 parts
        # one is the l1 distance
        # the other one is the penalty for changing dof to move
        # the last one is induced when the cell is close to obstacle
        cost_l1 = self.l1_cost(current_cell_coordinate,goal_cell_coordinate)

        # change penalty
        change_penalty = 0
        count = 0
        for ii in range(3):
            if current_cell_coordinate[ii] != goal_cell_coordinate[ii]:
                count += 1
        
        if count > 1:
            change = 100 #penalty for changing dof

        # close penalty
        close_penalty = 0
        if min_distance > self.collision_distance and min_distance <= self.close_distance:
            close_penalty = min_distance ** 3


        return cost_l1 + change_penalty + close_penalty          

    def l1_cost(self,current_cell_coordinate, goal_cell_coordinate):
        return np.sum(abs(np.array(current_cell_coordinate) - np.array(goal_cell_coordinate)))



    # function for selecting next target for the excavator to reach
    def get_next_target(self):
        # based on the planned path, select the next target and publish the target
        # call re-planning if necessary
        pass

    # help functions:

    # forward kinematics
    def forward_kinematics(self, joint_coordinates):
        # input: joint_coordinates [angle_base,angle_boom,angle_stick]
        # output: a list of cartesian_coordinates [x,y,z]
        #         [bucket_pt, stick_boom_joint,boom_origin, boom_curve_pt]
        pass

    # inverse kinematics
    def inverse_kinematics(self, cartesian_coordinates):
        # input: cartesian_coordinates [x,y,z]
        # output: joint_coordinates [angle_base,angle_boom,angle_stick]
        pass

    # check collision
    def check_collision(self,joint_coordinates):
        # Input: a joint coordinates
        # Output: 
        # collision: whether the input configuration leads to collision, 0 for no collision, 1 for collision
        # min_distance: min_distance from excavator pts to obstacle

        excavator_pts = self.forward_kinematics(joint_coordinates)
        collision = False
        min_distance = np.inf
        excavator_pts.append( (np.array(excavator_pts[0]) + np.array(excavator_pts[1])) / 2)
        for pt in excavator_pts:
            dist, ind = tree.query(pt.reshape(-1,1),1)
            distance = dist[0][0]
            if min_distance > distance:
                min_distance = distance
            if min_distance < self.collision_distance:
                collision = True
                break

        
        return collision, min_distance

    # 

if __name__ == '__main__':
    rospy.init_node('ExcavatorMotionPlanning', anonymous=True)
    excavator_motion_planning = ExcavatorMotionPlanning()
    try:
        while not rospy.is_shutdown():
            xyz_pts = excavator_motion_planning.get_next_target()
            
            

    except KeyboardInterrupt or rospy.ROSInterruptException:
        pass