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
import math
import numpy as np
import time
import copy

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
        self.collision_distance = 0.1 # unit: m
        self.close_distance = 0.3 # unit: m
        self.boom_len = 0.482 # unit: m
        self.stick_len = 0.255 # unit: m
        self.boom_origin_dev = 0.04 # unit: m [WARNING: NOT MEASURED YET]
        self.curve_dev = 0.2 # unit: m [WARNING: NOT MEASURED YET]
        self.curve_h = 0.13 # unit: m  [WARNING: NOT MEASURED YET]

        # tolerance
        self.middle_target_tolerance = 3 # how much angle difference we can tolerate for a middle target along a path
        self.end_target_tolerance = 1 # how much angle difference we can tolerate for the final target

        # bounds and counters for re-planning
        self.replanning_bound = 20 # how much time we should re-plan our path
        self.replanning_counter = 0


        # storage variables
        self.bucket_goal = [] # goal for the bucket to reach, in terms of excavator coordinate system， [x,y,z]
        self.collision_free_path = [] # a series of goal points for our excavator arm to reach [angle_base,angle_boom,angle_stick]

        self.obstacle_tree = []


        # flags
        self.visualize = True # whether visualize data or not

        # instances for visualization
        self.excavator_pts_visualization = MarkerPublisher(topic = '/excavator_arm_pts', type = 1,id = 0,color = 'b',scale = 0.1) # publisher for pts on the excavator
        self.path_visualization = MarkerPublisher('/collision_free_path',type = 1,id = 1,color = 'g',scale = 0.05) # publish as as a line strip
        self.target_visualization = MarkerPublisher('/bucket_target',type = 0,id = 2,color = 'r',scale = 0.1) # publish the target of the bucket
        self.stick_boom_joint_path_visualization = MarkerPublisher('/boom_stick_joint_path',type = 1,id = 3,color = 'g',scale = 0.05) # publish as as a line strip
        self.base_path_visualization = MarkerPublisher('/base_path',type = 1,id = 4,color = 'g',scale = 0.05) # publish as as a line strip

        pass


    # function for setting final goal for the excavator bucket
    def set_bucket_goal(self,bucket_goal):
        # Interface for users to set the goal for the bucket to reach
        # bucket_goal in the format [x,y,z]
        self.bucket_goal = bucket_goal
        print("goal: %f, %f, %f"%(bucket_goal[0],bucket_goal[1],bucket_goal[2]))
        if self.visualize:
            self.target_visualization.publish([bucket_goal])

        pass


    # function for planning a collision free path
    def path_planning(self):
        # plan a collision free path
        # the algorithm we will use for path planning hasn't been decided

        # use self variables:
        # self.bucket_goal (utilize)
        # self.collision_free_path = [] (modify)

        # use function:
        # self.check_collision


        # idea 1: call self.astar() method
        self.astar()
        if self.collision_free_path != [] and self.visualize:
            self.bucket_path = []
            self.boom_stick_path = []
            self.base_path = []
            for pt in self.collision_free_path:
                excavator_pts = self.forward_kinematics(pt)
                self.bucket_path.append(excavator_pts[0])
                self.boom_stick_path.append(excavator_pts[1])
                self.base_path.append(excavator_pts[3])
            self.path_visualization.publish(self.bucket_path)
            self.stick_boom_joint_path_visualization.publish(self.boom_stick_path)
            self.base_path_visualization.publish(self.base_path)
            self.target_visualization.publish([self.bucket_goal])


    def astar(self):

        # get the obstacles
        obstacle = self.obstacle_subscriber.get_pointcloud()

        #print(obstacle.shape)

        # form a KDtree of the obstacles
        self.obstacle_tree = KDTree(np.array(obstacle))

        # transform the goal into cell coordinate
        goal_cell_coordinate = self.inverse_kinematics(self.bucket_goal)
        #print("goal cell coordinate: %f, %f, %f"%(goal_cell_coordinate[0],goal_cell_coordinate[1],goal_cell_coordinate[2]))

        # joint space
        joint_space = JointSpace(goal_cell_coordinate)

        # check whether the goal is valid
        if not joint_space.check_goal_valid():
            print("Input goal invalid! No path generation")
            return


        # open list
        open_list = fibheap.Fheap() # use a heap to store the open list
        

        # get the joint coordinate of the current pos
        current_joint_coordinate = self.measured_angle_subscriber.get_angles()
        print("Measured joint angles:")
        print(current_joint_coordinate)

        if current_joint_coordinate[0] == 0 and current_joint_coordinate[1] == 0 and current_joint_coordinate[2] == 0:
            return

        # get the init cell
        init_cell = joint_space.get_cell(current_joint_coordinate)

        # get the init cell coordinate
        init_cell_coordinate = init_cell.get_joint_space_pos()

        #print("init cell coordinate: %f, %f, %f"%(init_cell_coordinate[0],init_cell_coordinate[1],init_cell_coordinate[2]))

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

        print("init cell joint space: %f, %f, %f"%(init_cell_coordinate[0],init_cell_coordinate[1],init_cell_coordinate[2]))
        idx = joint_space.get_cell_index(init_cell_coordinate)
        print("init cell index: %f, %f, %f"%(idx[0],idx[1],idx[2]))

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

            #print("equal?: ",np.array_equal(np.array(cell_coordinate),np.array(new_world_coordinate)))

            # check whether goal has been reached
            if joint_space.is_goal(cell_coordinate): # goal has been reached
                path_cost = new_gs + self.cost_function(cell_coordinate,goal_cell_coordinate)
                path_found = True
                print("Found path with path cost = %f"%(path_cost))
                break

            # expand the current node, check whether the gscore will be smaller for the current node's unexplored neighbor when going from this node
            successor_list = joint_space.expand([int(cell_coordinate[0]),int(cell_coordinate[1]),int(cell_coordinate[2])])

            # a = raw_input()
            # if a == "q":
            #     return
            # print("\n---")
            # idx = joint_space.get_cell_index(cell_coordinate)
            # print("new cell idx: %f, %f, %f"%(idx[0],idx[1],idx[2]))

            # print(successor_list)

            for s_prime in successor_list:

                idx = joint_space.get_cell_index(s_prime)
                # print("successor idx: %f, %f, %f"%(idx[0],idx[1],idx[2]))


                # get the cell corresponding to the coordinate s', and 
                successor_cell = joint_space.get_cell(s_prime)

                collision = successor_cell.has_collided()
                if collision == 2: # colision condition unknown, apply check collision methods to check
                    collision, min_distance = self.check_collision(s_prime)
                    # set the collision to the cell
                    successor_cell.set_collision(collision)
                    successor_cell.set_min_distance_to_obstacle(min_distance)
                
                if collision == 1:
                    # print("collision!")
                    continue # if this cell is colliding, directly go to the next


                # print("no collision!")
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
                        open_list.decrease_key(successor_cell.node,[temp_gs + h_s_prime,s_prime]) # decrease key
                        
                    else:
                        # mark the cell as in open list
                        successor_cell.in_open_list = True
                        # create a node for the cell
                        successor_cell.node = fibheap.Node([temp_gs + h_s_prime,s_prime])
                        # add the node to open list
                        open_list.insert(successor_cell.node)



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
        
        # count = 0
        # for ii in range(3):
        #     if current_cell_coordinate[ii] != goal_cell_coordinate[ii]:
        #         count += 1
        
        # if count > 1:
        #     change_penalty = 100 #penalty for changing dof

        # close penalty
        close_penalty = 0
        if min_distance > self.collision_distance and min_distance <= self.close_distance:
            close_penalty = min_distance ** (-3)


        return cost_l1 + change_penalty + close_penalty          

    def l1_cost(self,current_cell_coordinate, goal_cell_coordinate):
        return np.sum(abs(np.array(current_cell_coordinate) - np.array(goal_cell_coordinate)))



    # function for selecting next target for the excavator to reach
    def get_next_target(self):
        # based on the planned path, select the next target and publish the target
        # call re-planning if necessary
        current_joint_coordinate = self.measured_angle_subscriber.get_angles()
        pts = self.forward_kinematics(current_joint_coordinate)
        #print(pts)
        #print(current_joint_coordinate)
        if self.collision_free_path == [] or self.replanning_counter >= self.replanning_bound:
            self.path_planning()
            self.replanning_counter = 0
        else:
            # select next target
            pass

        self.replanning_counter += 1
        if self.visualize:
            self.excavator_pts_visualization.publish(self.forward_kinematics(current_joint_coordinate))
            self.target_visualization.publish([self.bucket_goal])

        if self.collision_free_path != [] and self.visualize:
            
            self.path_visualization.publish(self.bucket_path)
            self.stick_boom_joint_path_visualization.publish(self.boom_stick_path)
            self.base_path_visualization.publish(self.base_path)
            

        pass

    # help functions:

    # forward kinematics
    def forward_kinematics(self, joint_coordinates):
        # input: joint_coordinates [angle_base,angle_boom,angle_stick]
        # output: a list of cartesian_coordinates [x,y,z]
        #         [bucket_pt, stick_boom_joint,boom_curve_pt,boom_origin]
        
        # Exctract the three angles
        raw_base, raw_boom, raw_boom_stick = joint_coordinates
        raw_stick = raw_boom + raw_boom_stick - 180


        angle_base = raw_base * math.pi/180
        angle_stick = raw_stick * math.pi/180
        angle_boom = raw_boom * math.pi/180
        
        # Determine height of the points
        stick_boom_joint_h = self.boom_len * math.sin(angle_boom)
        bucket_pt_h =  stick_boom_joint_h+ self.stick_len * math.sin(angle_stick)
        boom_origin_h = 0
        boom_curve_pt_h = self.curve_dev * math.sin(angle_boom) + self.curve_h * math.cos(angle_boom)
        
        
        # Determine the horziontal projection coordinates of the points
        
        # Horizontal projection length of the joint and the whole arm
        joint_l = self.boom_len * math.cos(angle_boom)
        joint_l_tot = self.boom_origin_dev + joint_l
        arm_l_tot = joint_l_tot + self.stick_len * math.cos(angle_stick)

        # Horizontal projection length of the highest point on the curve
        curve_l = self.curve_dev * math.cos(angle_boom) - self.curve_h * math.sin(angle_boom)
        curve_l_tot = curve_l + self.boom_origin_dev
        
        # Obtain the x & y coordinates
        stick_boom_joint_x = joint_l_tot * math.cos(angle_base)
        stick_boom_joint_y = joint_l_tot * math.sin(angle_base)
        
        bucket_pt_x = arm_l_tot * math.cos(angle_base)
        bucket_pt_y = arm_l_tot * math.sin(angle_base)
        
        boom_origin_x = self.boom_origin_dev * math.cos(angle_base)
        boom_origin_y = self.boom_origin_dev * math.sin(angle_base)
        
        boom_curve_x = curve_l_tot * math.cos(angle_base)
        boom_curve_y = curve_l_tot * math.sin(angle_base)
        
        
        # Return the final results
        res = []
        bucket_pt = [bucket_pt_x, bucket_pt_y, bucket_pt_h]
        stick_boom_joint = [stick_boom_joint_x, stick_boom_joint_y, stick_boom_joint_h]
        boom_origin = [boom_origin_x, boom_origin_y, boom_origin_h]
        boom_curve = [boom_curve_x, boom_curve_y, boom_curve_pt_h]
        res.append(bucket_pt)
        res.append(stick_boom_joint)
        res.append(boom_curve)
        res.append(boom_origin)
        
        return res


    # inverse kinematics
    def inverse_kinematics(self, cartesian_coordinates):
        # input: cartesian_coordinates [x,y,z]
        # output: joint_coordinates [angle_base,angle_boom,angle_stick]
        # input: cartesian_coordinates [x,y,z]
        # output: joint_coordinates [angle_base,angle_boom,angle_stick]
        x,y,z=cartesian_coordinates
        angle_base = math.atan2(y,x)
		 
        total_len = math.sqrt(x*x + y*y)
        r = total_len - self.boom_origin_dev
        
        len_square = r*r + z*z
        
        A = len_square - self.boom_len*self.boom_len + self.stick_len *self.stick_len
        add_term = self.boom_len+self.stick_len
        sub_term = self.boom_len-self.stick_len
        B = math.sqrt((len_square-sub_term*sub_term)*(add_term*add_term-len_square))
		
        angles = []
		
        b = (-r*B+z*A)/(2*len_square)
        #print("z = %f, b = %f"%(z,b))
        angle_boom = math.asin((z-b)/self.boom_len)
		
        if (A-2*z*b>0):
            angle_stick = math.asin(b/self.stick_len)
        else:
            angle_stick = -1*math.asin(b/self.stick_len)-math.pi
		
        raw_base = angle_base * 180/math.pi
        raw_boom = angle_boom * 180/math.pi
        raw_stick = angle_stick * 180/math.pi
        raw_boom_stick = raw_stick + 180 - raw_boom
        angles.append(raw_base)
        angles.append(raw_boom)
        angles.append(raw_boom_stick)
		
        return angles
		
        

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
            np_pt = np.array(pt)
            dist, _ = self.obstacle_tree.query(np_pt.reshape(1,-1),1)
            #print(dist)
            
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
    #pts = excavator_motion_planning.forward_kinematics([0,30,50])
    #excavator_motion_planning.set_bucket_goal(pts[0])

    
    # pts_joint_space = excavator_motion_planning.inverse_kinematics([0.3,0.0,0.5])
    # print("ik result: [%f,%f,%f]"%(pts_joint_space[0],pts_joint_space[1],pts_joint_space[2]))
    # pts_cartesian = excavator_motion_planning.forward_kinematics(pts_joint_space)
    # print("fk target: [%f,%f,%f]"%(pts_cartesian[0][0],pts_cartesian[0][1],pts_cartesian[0][2]))

    cartesian_target = excavator_motion_planning.forward_kinematics([90,45,50])
    pts_joint_space = excavator_motion_planning.inverse_kinematics(cartesian_target[0])
    print("ik result: [%f,%f,%f]"%(pts_joint_space[0],pts_joint_space[1],pts_joint_space[2]))
    pts_cartesian = excavator_motion_planning.forward_kinematics(pts_joint_space)
    print("fk target: [%f,%f,%f]"%(pts_cartesian[0][0],pts_cartesian[0][1],pts_cartesian[0][2]))

    # joint_space = JointSpace(copy.copy(pts_joint_space))

    # pts_joint_space = [9.0,21.0,31.0]

    # print("modified result: [%f,%f,%f]"%(pts_joint_space[0],pts_joint_space[1],pts_joint_space[2]))
    # print(joint_space.is_goal(pts_joint_space))
    


    # cell_coordinate = [-28,20,40]
    # obstacle = excavator_motion_planning.obstacle_subscriber.get_pointcloud()
    # excavator_motion_planning.obstacle_tree = KDTree(np.array(obstacle))
    # collision, min_distance = excavator_motion_planning.check_collision(cell_coordinate)
    # print("collision: ")
    # print(collision)
    # print("min_distance = %f"%(min_distance))

    # cost = excavator_motion_planning.cost_function(cell_coordinate,pts_joint_space,0)
    # print("cost == %f"%(cost))
    # h_cost = excavator_motion_planning.heuristic_function(cell_coordinate, pts_joint_space)
    # print("h_cost == %f"%(h_cost))
    
    

    # successor_list = joint_space.expand(pts_joint_space)
    # print(successor_list)
    


    excavator_motion_planning.set_bucket_goal(pts_cartesian[0])
    try:
        while not rospy.is_shutdown():
            #xyz_pts = excavator_motion_planning.get_next_target()
            excavator_motion_planning.get_next_target()
            rospy.sleep(0.1)

    except KeyboardInterrupt or rospy.ROSInterruptException:
        pass