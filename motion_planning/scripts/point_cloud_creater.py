#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import time

from point_cloud_subscriber import PointCloudSubscriber
from point_cloud_publisher import PointCloudPublisher, remove_excavator_pts_bounding_box,camera_to_robot_camera_1, remove_excavator_pts,camera_global_to_rotation_center,local2global
from motion_planning.msg import excavator_control_signals # different use of the msg, publish commands to control camera motion

if __name__ == '__main__':
    rospy.init_node('PointCloudCreator', anonymous=True)

    # publisher for the obstacle
    point_cloud_publisher = PointCloudPublisher(topic = 'Obstacle_pointcloud',color = 'b')

    # publisher for camera control command
    camera_servo_publisher = rospy.Publisher('excavator_control', excavator_control_signals, queue_size=1)

    # initialize the subscriber for pointcloud
    local_pcl_subscriber = PointCloudSubscriber('/camera/depth/color/points')

    # initialize the control command
    control_signal = excavator_control_signals()
    control_signal.boom_servo = 30
    control_signal.stick_servo = 0
    control_signal.pump_mode = -10
    control_signal.base_pwm = 0

    # load in 14 captured frames, try to transform them into global frame
    global_data_list = []
    servo_angle = [2,17,32,44,58,71,84,97,112,123,137,150,165,180]

    color_list = []

    control_signal.boom_servo = 30
    camera_servo_publisher.publish(control_signal) # go to the front

    time.sleep(5)

    for ii in range(len(servo_angle)):
        control_signal.boom_servo = 30 + ii * 10 # control command we will send
        # send the control command
        camera_servo_publisher.publish(control_signal)

        # wait for the servo to reach target, and wait for the camera to get the pointcloud
        time.sleep(3)
        
        # get the latest point cloud's coordinates and color
        local_data_camera, pt_color = local_pcl_subscriber.get_pointcloud_rgb()

        local_data = camera_to_robot_camera_1(np.array(local_data_camera).T)

        global_data = local2global(local_data,90 - servo_angle[ii],18,np.array([0.06,0,0]))

        global_data_list += (global_data.T)[:,0:3].tolist()
        color_list += pt_color

        # for demonstration
        data_global = camera_global_to_rotation_center(global_data_list)
        
        point_cloud_publisher.publish_rgb(data_global.tolist(),color_list)

    control_signal.boom_servo = 90
    camera_servo_publisher.publish(control_signal) # go to the front

    # remove the excavator points, publish the point cloud
    global_data_list = camera_global_to_rotation_center(global_data_list)
    print("global length: ", global_data_list.shape[0])
    color_list = np.array(color_list, np.uint8)
    lower = np.array([100, 110, 46])
    upper = np.array([140, 255, 255])
    # global_data_list, color_list = remove_excavator_pts(global_data_list, color_list, lower, upper)
    global_data_list, color_list = remove_excavator_pts(global_data_list, color_list, lower, upper, 0.54)
    global_data_list, color_list = remove_excavator_pts_bounding_box(global_data_list,color_list)
    # color_list = color_list.tolist()
    data = global_data_list

    while not rospy.is_shutdown():
        
        point_cloud_publisher.publish_rgb(data,color_list)
        rospy.sleep(0.1)