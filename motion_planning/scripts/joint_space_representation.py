#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Author: Jinjie Liu, Shiji Liu

import numpy as np

# the cell representing each state
class cell(object):
    def __init__(self, collision = 0):
        self.collision = collision # 0 - not collision, 1 - collision, 2 - unknown
        self.cost_to_come = np.inf
        self.cost_to_go = np.inf
        self.parentid = [] # parent id, in terms of cell coordinate
        self.joint_space_pos = [] # [base_angle,boom_angle,stick_angle]
        self.in_open_list = False
        self.min_distance_to_obstacle = None
        # add a fheap node, trying to increase speed
        self.node = None
    
    def has_collided(self):
        return self.collision
    
    def set_collision(self,collision):
        self.collision = collision
    
    def set_min_distance_to_obstacle(self, min_distance):
        self.min_distance_to_obstacle = min_distance

    def inside_open_list(self):
        return self.in_open_list
        
    def set_cost_to_come(self, cost_to_come):
        self.cost_to_come = cost_to_come
        
    def get_cost_to_come(self):
        return self.cost_to_come
    
    def set_joint_space_pos(self, joint_space_pos):
        self.joint_space_pos = joint_space_pos
        
    def get_joint_space_pos(self):
        return self.joint_space_pos
    
    # TODO: align parentid....
    def set_parent(self, parentid):
        self.parentid = parentid
        
    def get_parent(self):
        return self.parentid

    
    def reset(self):
        self.cost_to_come = np.inf
        self.cost_to_go = np.inf
        self.parentid = [] # parent id, in terms of cell coordinate
        self.collision = 2 # unknown
        self.in_open_list = False # whether a cell is currently in open list



class JointSpace(object):
    def __init__(self, goal_joint_coordinate):
        self.goal_cell = []
        pass

    def _create_cells(self):
        # create storage of all pts in the configuration space
        pass


    def get_cell(self, joint_coordinate):
        # input: joint_coordinate [base_angle,boom_angle,stick_angle]
        # output: a reference to a cell
        pass

    def is_goal(self,joint_coordinate):
        # check whether a input joint_coordinate is equivalent to goal coordinate
        pass

    def reset_cell_cost_map(self):
        pass

    def expand(self,joint_coordinate):
        pass