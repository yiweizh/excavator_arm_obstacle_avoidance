#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import rospy
import numpy as np
import rospkg


import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import tf

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

    def publish_rgb(self, data_3d, color_list):
        # data_3d is a list of [x,y,z]
        # color_list is a list of [r,g,b]
        new_data = []
        for data, color in zip(data_3d, color_list):
            rgba = struct.unpack('I', struct.pack('BBBB', color[2], color[1], color[0], 255))[0]
            new_data.append(data + [rgba])
            

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

def camera_to_robot_camera(pts):
    # pts: a np array of pts in homogeneous coordinate
    #      shape: (4,#)
    # R = np.array([[0,0,1,0], 
    #               [-1,0,0,0], 
    #               [0,-1,0,0], 
    #               [0,0,0,1]])
    # new_pts = np.dot(R,pts)

    # new_pts[0:3,:] /= new_pts[3,:]

    #new_pts[2,:] *= -1

    xaxis = (1,0,0)
    yaxis = (0,1,0)
    zaxis = (0,0,1)

    Ry = tf.transformations.rotation_matrix(np.deg2rad(90.0),yaxis)
    Rz = tf.transformations.rotation_matrix(np.deg2rad(-90.0),zaxis)
    R = np.dot(Ry,Rz)


    # rotate with respect to x-axis in world frame by 180 degrees
    Rx = tf.transformations.rotation_matrix(np.deg2rad(180.0),xaxis)

    R = np.dot(Rx,R)

    new_pts = np.dot(R,pts)
    new_pts[0:3,:] /= new_pts[3,:]

    

    return new_pts
    

def decode_rgb_from_pcl(rgb):
    # copied from pypcd
    # link: https://github.com/dimatura/pypcd/blob/master/pypcd/pypcd.py
    """ Decode the bit-packed RGBs used by PCL.
    :param rgb: An Nx1 array.
    :rtype: Nx3 uint8 array with one column per color.
    """

    rgb = rgb.copy()
    rgb.dtype = np.uint32
    r = np.asarray((rgb >> 16) & 255, dtype=np.uint8)
    g = np.asarray((rgb >> 8) & 255, dtype=np.uint8)
    b = np.asarray(rgb & 255, dtype=np.uint8)
    rgb_arr = np.zeros((len(rgb), 3), dtype=np.uint8)
    rgb_arr[:, 0] = r
    rgb_arr[:, 1] = g
    rgb_arr[:, 2] = b

    # for ii in range(20):
    #     print("r = %d, g = %d, b = %d" %(r[ii],g[ii],b[ii]))

    return rgb_arr

def load_pcd_to_ndarray(pcd_path):
    with open(pcd_path) as f:
        while True:
            ln = f.readline().strip()
            if ln.startswith('DATA'):
                break

        points = np.loadtxt(f)
        new_pts = []
        pt_color = [] # store the rgb color of a given point
        for pt in points:
            if not (pt[0] == 0 and pt[1] == 0 and pt[2] == 0):
                new_pts.append(np.array([pt[0],pt[1],pt[2],1]))
                #rgb_arr = decode_rgb_from_pcl(pt[3])
                #pt_color.append(rgb_arr)
                pt_color.append(pt[3])

        # apply coordinate transformation, change points from camera coordinate to robot_camera_coordinate

        transformed_pts = camera_to_robot_camera(np.array(new_pts).T)


        print(transformed_pts.shape)
        # ret_val = (transformed_pts.T)[:,0:3].tolist()
        # return ret_val

        # decode rgb
        rbg_list = decode_rgb_from_pcl(np.array(pt_color))


        return transformed_pts, rbg_list

def local2global(pts, z_angle, incline_angle,translation_vec = np.array([0,0,0])):
    # pts: a np array of pts in homogeneous coordinate
    #      shape: (4,#)

    xaxis = (1,0,0)
    yaxis = (0,1,0)
    zaxis = (0,0,1)
    
    # step 1: rotate around z axis of robot_camera_coordinate
    Rz = tf.transformations.rotation_matrix(np.deg2rad(z_angle),zaxis)
    translation_vec = np.dot(Rz[0:3,0:3],translation_vec.reshape(-1,1)).reshape(-1)

    # step 2: rotate around y axis of the rotated robot_camera_coordinate
    Ry = tf.transformations.rotation_matrix(np.deg2rad(incline_angle),yaxis)

    # get full rotational matrix
    R = np.dot(Rz,Ry)

    # full transformation matrix
    R[0:3,3] = translation_vec

    # apply transformation to the matrix
    new_pts = np.dot(R,pts)

    new_pts = new_pts[0:3,:] / new_pts[3,:]

    return new_pts

if __name__ == '__main__':
    rospy.init_node('PointCloudPublisher', anonymous=True)

    point_cloud_publisher = PointCloudPublisher(topic = 'Obstacle_pointcloud',color = 'b')
    data0 = generate_pts_from_bounding_box([0.2,0.5,0.0],[0.1,0.15,0.4],10)
    # print(np.array(data0).shape)
    # data1 = generate_pts_from_bounding_box([0.2,0.2,0.4],[0.1,0.15,0.4],10)
    # data2 = generate_pts_from_bounding_box([0.2,-0.4,0.4],[0.1,0.15,0.4],10)
    # data3 = generate_pts_from_bounding_box([0.2,-0.7,0.0],[0.1,0.15,0.4],10)
    # data5 = generate_pts_from_bounding_box([0.2,-0.4,0.5],[0.1,0.15,0.4],10)
    # counter = 0
    # num_of_pts = 500
    #data = data0 + data1 + data2 + data3
    #data = data0 + data1 + data3 + data5
    #data = data0  + data3
    #data = data3


    rospack = rospkg.RosPack()
    directory = rospack.get_path('motion_planning') + '/scripts/captured_pointclouds/'

    # load in 14 captured frames, try to transform them into global frame
    global_data_list = []
    servo_angle = [2,17,32,44,58,71,84,97,112,123,137,150,165,180]

    color_list = []
    
    for ii in range(1,15):
        local_data, pt_color = load_pcd_to_ndarray(directory + 'Captured_Frame' + format(ii,'02') + '.pcd')

        global_data = local2global(local_data,90 - servo_angle[ii - 1],20,np.array([0.06,0,0]))

        global_data_list += (global_data.T)[:,0:3].tolist()
        color_list += pt_color.tolist()
    
    # local_data = load_pcd_to_ndarray(directory + 'Captured_Frame' + '07' + '.pcd')
    # global_data = local2global(local_data,0,0)
    # global_data_list += (global_data.T)[:,0:3].tolist()

    data = global_data_list #+ data0
    while not rospy.is_shutdown():
        # data.append([counter,counter,counter])
        # counter += 1
        # if counter >= num_of_pts:
        #     break

        point_cloud_publisher.publish(data)
        #point_cloud_publisher.publish_rgb(data,color_list)
        rospy.sleep(0.1)