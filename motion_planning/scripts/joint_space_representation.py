#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Author: Jinjie Liu, Shiji Liu


import numpy as np


# the cell representing each state
class Cell(object):
    def __init__(self, collision=2):
        self.collision = collision  # 0 - no collision, 1 - collision, 2 - unknown
        self.cost_to_come = np.inf
        self.cost_to_go = np.inf
        self.parentid = []          # parent id, in terms of cell coordinate
        self.joint_space_pos = []   # [base_angle, boom_angle, stick_angle]
        self.in_open_list = False
        self.min_distance_to_obstacle = None
        # add a fheap node, trying to increase speed
        self.node = None
    
    def has_collided(self):
        return self.collision
    
    def set_collision(self, collision):
        self.collision = collision
    
    def inside_open_list(self):
        return self.in_open_list
        
    def set_min_distance_to_obstacle(self, min_distance):
        self.min_distance_to_obstacle = min_distance

    def get_min_distance_to_obstacle(self):
        return self.min_distance_to_obstacle

    def set_cost_to_come(self, cost_to_come):
        self.cost_to_come = cost_to_come
        
    def get_cost_to_come(self):
        return self.cost_to_come
    
    def set_joint_space_pos(self, joint_space_pos):
        self.joint_space_pos = joint_space_pos
        
    def get_joint_space_pos(self):
        return self.joint_space_pos
    
    def set_parent(self, parentid):
        self.parentid = parentid
        
    def get_parent(self):
        return self.parentid

    def reset(self):
        self.cost_to_come = np.inf
        self.cost_to_go = np.inf
        self.parentid = []  # parent id, in terms of cell coordinate
        self.collision = 2  # unknown
        self.in_open_list = False   # whether a cell is currently in open list


class JointSpace(object):
    def __init__(self, goal_joint_coordinate):
        self.goal = goal_joint_coordinate
        self.goal_cell = []
        self.joint_space = []

        # set step angles for configuration space declaration
        self.step_base_angle = 4    # base angle step, unit: deg.
        self.step_boom_angle = 5    # boom angle step, unit: deg.
        self.step_stick_angle = 5   # stick angle step, unit: deg.

        # set min/max angles for configuration space declaration
        self.min_base_angle = -180  # min base angle, unit: deg.
        self.max_base_angle = 180   # max base angle, unit: deg.
        self.min_boom_angle = 0     # min boom angle, unit: deg.
        self.max_boom_angle = 60    # max boom angle, unit: deg.
        self.min_stick_angle = 15   # min stick angle, unit: deg.
        self.max_stick_angle = 150  # max stick angle, unit: deg.
        

    def check_goal_valid(self):
        if int(round(self.goal[0])) >= self.max_base_angle\
            or int(round(self.goal[0])) <= self.min_base_angle\
                or int(round(self.goal[1])) >= self.max_boom_angle\
                    or int(round(self.goal[1])) <= self.min_boom_angle\
                        or int(round(self.goal[2])) >= self.max_stick_angle\
                            or int(round(self.goal[2])) <= self.min_stick_angle:
            return 0
        else:
            self._create_cells()
            return 1


    def _create_cells(self):
        # create storage of all pts in the configuration space
        self.joint_space = [[[Cell()\
            for i in range((self.max_stick_angle - self.min_stick_angle) // self.step_stick_angle)]\
                for j in range((self.max_boom_angle - self.min_boom_angle) // self.step_boom_angle)]\
                    for k in range((self.max_base_angle - self.min_base_angle) // self.step_base_angle)]

        # set joint coordinates for each cell
        for i in range(len(self.joint_space)):
            for j in range(len(self.joint_space[i])):
                for k in range(len(self.joint_space[i][j])):
                    joint_space_pos = [self.min_base_angle + i * self.step_base_angle + self.step_base_angle // 2,\
                        self.min_boom_angle + j * self.step_boom_angle + self.step_boom_angle // 2,\
                            self.min_stick_angle + k * self.step_stick_angle + self.step_stick_angle // 2]
                    # print(joint_space_pos)
                    self.joint_space[i][j][k].set_joint_space_pos(joint_space_pos)

        # for i in range(len(self.joint_space)):
        #     for j in range(len(self.joint_space[i])):
        #         for k in range(len(self.joint_space[i][j])):
        #             print(self.joint_space[i][j][k].get_joint_space_pos()) 

        # set goal cell index
        self.goal_cell = self.get_cell_index(self.goal)
        print("goal cell: %f, %f, %f"%(self.goal_cell[0],self.goal_cell[1],self.goal_cell[2]))

    def get_cell(self, joint_coordinate):
        # input: joint_coordinate [base_angle, boom_angle, stick_angle]
        # output: a reference to a cell
        base_index = (int(round(joint_coordinate[0])) - self.min_base_angle) // self.step_base_angle
        boom_index = (int(round(joint_coordinate[1])) - self.min_boom_angle) // self.step_boom_angle
        stick_index = (int(round(joint_coordinate[2])) - self.min_stick_angle) // self.step_stick_angle
        # print(int(base_index))
        # print(int(boom_index))
        # print(int(stick_index))
        return self.joint_space[int(base_index)][int(boom_index)][int(stick_index)]
        #return self.joint_space[(base_index)][(boom_index)][(stick_index)]

    def get_cell_index(self, joint_coordinate):
        # input: joint_coordinate [base_angle, boom_angle, stick_angle]
        # output: corresponding cell index
        base_index = (int(round(joint_coordinate[0])) - self.min_base_angle) // self.step_base_angle
        boom_index = (int(round(joint_coordinate[1])) - self.min_boom_angle) // self.step_boom_angle
        stick_index = (int(round(joint_coordinate[2])) - self.min_stick_angle) // self.step_stick_angle
        return [base_index, boom_index, stick_index]

    def is_goal(self, joint_coordinate):
        # check whether a input joint_coordinate is equivalent to goal coordinate
        # print("joint coordinate: %f, %f, %f"%(joint_coordinate[0],joint_coordinate[1],joint_coordinate[2]))
        #print("self.goal = %f, %f, %f"%(self.goal_cell[0],self.goal_cell[1],self.goal_cell[2]))
        current_cell = self.get_cell_index(joint_coordinate)
        #print("joint coordinate: %f, %f, %f"%(current_cell[0],current_cell[1],current_cell[2]))
        return np.array_equal(np.array(current_cell), np.array(self.goal_cell))

    def reset_cell_cost_map(self):
        for i in range(len(self.joint_space)):
            for j in range(len(self.joint_space[i])):
                for k in range(len(self.joint_space[i][j])):
                    self.joint_space[i][j][k].reset()

    def expand(self, joint_coordinate):
        # input: current node joint coordinates [base_angle, boom_angle, stick_angle]
        # output: list of valid neighbors' joint coordinates
        neighbors = []
        current_coordinates = self.get_cell_index(joint_coordinate)

        # +/- base neighbor
        if current_coordinates[0] > 0:
            # append joint space coordinates to valid neighbor list and set parent if no collision detected
            if (self.joint_space[current_coordinates[0] - 1][current_coordinates[1]][current_coordinates[2]].has_collided() != 1):
                neighbors.append(self.joint_space[current_coordinates[0] - 1][current_coordinates[1]][current_coordinates[2]].get_joint_space_pos())

        if current_coordinates[0] < len(self.joint_space) - 1:
            # append joint space coordinates to valid neighbor list and set parent if no collision detected
            if (self.joint_space[current_coordinates[0] + 1][current_coordinates[1]][current_coordinates[2]].has_collided() != 1):
                neighbors.append(self.joint_space[current_coordinates[0] + 1][current_coordinates[1]][current_coordinates[2]].get_joint_space_pos())
        # elif current_coordinates[0] == len(self.joint_space) - 1:
        #     # append joint space coordinates to valid neighbor list and set parent if no collision detected
        #     if (self.joint_space[0][current_coordinates[1]][current_coordinates[2]].has_collided() != 1):
        #         neighbors.append(self.joint_space[0][current_coordinates[1]][current_coordinates[2]].get_joint_space_pos())

        # +/- boom neighbor
        if current_coordinates[1] > 0:
            # append joint space coordinates to valid neighbor list and set parent if no collision detected
            if (self.joint_space[current_coordinates[0]][current_coordinates[1] - 1][current_coordinates[2]].has_collided() != 1):
                neighbors.append(self.joint_space[current_coordinates[0]][current_coordinates[1] - 1][current_coordinates[2]].get_joint_space_pos())

        if current_coordinates[1] < len(self.joint_space[0]) - 1:
            # append joint space coordinates to valid neighbor list and set parent if no collision detected
            if (self.joint_space[current_coordinates[0]][current_coordinates[1] + 1][current_coordinates[2]].has_collided() != 1):
                neighbors.append(self.joint_space[current_coordinates[0]][current_coordinates[1] + 1][current_coordinates[2]].get_joint_space_pos())

        # +/- stick neighbor
        if current_coordinates[2] > 0:
            # append joint space coordinates to valid neighbor list and set parent if no collision detected
            if (self.joint_space[current_coordinates[0]][current_coordinates[1]][current_coordinates[2] - 1].has_collided() != 1):
                neighbors.append(self.joint_space[current_coordinates[0]][current_coordinates[1]][current_coordinates[2] - 1].get_joint_space_pos())

        if current_coordinates[2] < len(self.joint_space[0][0]) - 1:
            # append joint space coordinates to valid neighbor list and set parent if no collision detected
            if (self.joint_space[current_coordinates[0]][current_coordinates[1]][current_coordinates[2] + 1].has_collided() != 1):
                neighbors.append(self.joint_space[current_coordinates[0]][current_coordinates[1]][current_coordinates[2] + 1].get_joint_space_pos())

        # print(neighbors)
        return neighbors

# for debugging
def main():
    js = JointSpace([20, 20, 90])
    js.get_cell([20, 22, 90]).set_collision(0)
    js.get_cell([18, 20, 90]).set_collision(0)
    js.expand([20, 20, 90])


if __name__ == "__main__":
    main()
