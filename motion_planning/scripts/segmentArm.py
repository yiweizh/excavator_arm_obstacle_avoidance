import numpy as np
import ctypes
import struct
import cv2
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt

def segmentArm(hsv_img, lower, upper):
    # interval 1
    # mask1 = np.where(np.logical_and(hsv_list >= lower_red, hsv_list <= upper_red))
    mask1 = cv2.inRange(hsv_img, lower, upper)
    # # interval 2
    # lower_red = np.array([156, 43, 46])
    # upper_red = np.array([180, 255, 255])
    # # mask2 = np.where(np.logical_and(hsv_list >= lower_red, hsv_list <= upper_red))
    # mask2 = cv2.inRange(hsv_img, lower_red, upper_red)
    # mask = mask1 + mask2
    mask = mask1
    return mask

if __name__=="__main__":
    img_path = '/home/pyhuang/VE450/excavator_arm_obstacle_avoidance/perception/dataset/aligned_color_frame7.png'
    img = cv2.imread(img_path)
    cv2.namedWindow('imageshow', 2)
    cv2.imshow('imageshow', img)

    # direct access
    B = img[:, :, 0]
    G = img[:, :, 1]
    R = img[:, :, 2]

    # wrong BGR -> RGB
    RGB_img = np.zeros_like(img)
    RGB_img[:, :, 0] = R
    RGB_img[:, :, 1] = G
    RGB_img[:, :, 2] = B
    cv2.namedWindow("RGB?", 2)
    cv2.imshow('RGB?', RGB_img)

    img_hsv = cv2.cvtColor(RGB_img, cv2.COLOR_RGB2HSV)
    def getpos(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(img_hsv[y, x])
    cv2.namedWindow("HSV?", 2)
    cv2.imshow("HSV?", img_hsv)
    cv2.setMouseCallback("RGB?", getpos)
    # lower = np.array([90, 43, 46])
    lower = np.array([100, 110, 46])
    upper = np.array([140, 255, 255])
    mask = segmentArm(img_hsv, lower, upper)
    arm_idx = np.where(mask==255)
    arm_img = RGB_img
    arm_img[arm_idx] = np.array([255, 255, 255])
    cv2.namedWindow("Arm White", 2)
    cv2.imshow('Arm White', arm_img)

    cv2.waitKey(0)