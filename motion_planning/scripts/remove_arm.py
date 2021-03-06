import numpy as np
import ctypes
import struct
import cv2
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt

def rgb2hsv(rgb):
    """ convert RGB to HSV color space

    :param rgb: np.ndarray
    :return: np.ndarray
    """

    rgb = rgb.astype('float')
    maxv = np.amax(rgb, axis=1)
    maxc = np.argmax(rgb, axis=1)
    minv = np.amin(rgb, axis=1)
    minc = np.argmin(rgb, axis=1)

    hsv = np.zeros(rgb.shape, dtype='float')
    hsv[maxc == minc, 0] = np.zeros(hsv[maxc == minc, 0].shape)
    hsv[maxc == 0, 0] = (((rgb[..., 1] - rgb[..., 2]) * 60.0 / (maxv - minv + np.spacing(1))) % 360.0)[maxc == 0]
    hsv[maxc == 1, 0] = (((rgb[..., 2] - rgb[..., 0]) * 60.0 / (maxv - minv + np.spacing(1))) + 120.0)[maxc == 1]
    hsv[maxc == 2, 0] = (((rgb[..., 0] - rgb[..., 1]) * 60.0 / (maxv - minv + np.spacing(1))) + 240.0)[maxc == 2]
    hsv[maxv == 0, 1] = np.zeros(hsv[maxv == 0, 1].shape)
    hsv[maxv != 0, 1] = (1 - minv / (maxv + np.spacing(1)))[maxv != 0]
    hsv[..., 2] = maxv

    return hsv

def rgb2hsv(r, g, b):   
    r, g, b = r/255.0, g/255.0, b/255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx-mn
    if mx == mn:
        h = 0
    elif mx == r:
        h = (60 * ((g-b)/df) + 360) % 360
    elif mx == g:
        h = (60 * ((b-r)/df) + 120) % 360
    elif mx == b:
        h = (60 * ((r-g)/df) + 240) % 360
    if mx == 0:
        s = 0
    else:
        s = df/mx
    v = mx
    H = h / 2
    S = s * 255.0
    V = v * 255.0
    return H, S, V

def float_to_rgb(float_rgb):
    """ Converts a packed float RGB format to an RGB list

        Args:
            float_rgb: RGB value packed as a float

        Returns:
            color (list): 3-element list of integers [0-255,0-255,0-255]
    """
    s = struct.pack('>f', float_rgb)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value

    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)

    color = [r,g,b]

    return color

def load_color_to_ndarray(pcd_path):
    with open(pcd_path) as f:
        points = np.loadtxt(f)
        rbg_list = []
        for pts in points:
            for pt in pts:
                rgb_arr = float_to_rgb(pt)
                rbg_list.append(rgb_arr)
        return rbg_list

def unsorted_rainbow(colors):
    # colors = colors.reshape((-1, 500, 3))
    # for i in range(0, 530000 - colors.shape[0]):
    #     colors.append(np.array([0, 0, 0]))
    colors = colors.reshape((530, 1000, 3))
    
    plt.imshow(colors)
    plt.show()

def segmentRed(hsv_img):
    # interval 1
    lower_red = np.array([0, 43, 46])
    upper_red = np.array([10, 255, 255])
    # mask1 = np.where(np.logical_and(hsv_list >= lower_red, hsv_list <= upper_red))
    mask1 = cv2.inRange(hsv_img, lower_red, upper_red)
    # interval 2
    lower_red = np.array([156, 43, 46])
    upper_red = np.array([180, 255, 255])
    # mask2 = np.where(np.logical_and(hsv_list >= lower_red, hsv_list <= upper_red))
    mask2 = cv2.inRange(hsv_img, lower_red, upper_red)
    mask = mask1 + mask2
    return mask

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

if __name__ == '__main__':
    pcd_path = '/home/pyhuang/VE450/excavator_arm_obstacle_avoidance/pointcloud_xyzrgba.txt'
    data = np.loadtxt(pcd_path)
    rbg_list = np.array(data[..., -3:], dtype=np.uint8)
    R = rbg_list[:, 2]
    G = rbg_list[:, 1]
    B = rbg_list[:, 0]
    rgb_list = np.zeros_like(rbg_list)
    rgb_list[:, 0] = R
    rgb_list[:, 1] = G
    rgb_list[:, 2] = B
    # rbg_list = load_color_to_ndarray(pcd_path)
    old_length = len(rgb_list)
    for i in range(0, 71100 - old_length):
        rgb_list = np.concatenate((rgb_list, np.array([[0, 0, 0]])), axis=0)
    rgb_list = np.array(rgb_list, dtype=np.uint8)
    print(rgb_list.shape)
    red_colors = rgb_list.reshape((711, 100, 3))
    
    #Method 1
    hsv_img = cv2.cvtColor(red_colors, cv2.COLOR_RGB2HSV)
    
    cv2.namedWindow("RGB", 2)
    cv2.imshow('RGB', red_colors)
    lower = np.array([100, 43, 46])
    upper = np.array([140, 255, 255])
    mask = segmentArm(hsv_img, lower, upper)
    # mask = mask.reshape((530, 1000))
    binary_mask = np.where(mask==255)
    dark_idx = np.where(mask == 0)
    dark_place = red_colors[dark_idx]
    red_place = red_colors[binary_mask]
    
    old_length = dark_place.shape[0]
    while old_length % 100 != 0:
        old_length += 1
        dark_place = np.concatenate((dark_place, np.array([[0, 0, 0]])), axis=0)
    old_length = red_place.shape[0]
    while old_length % 100 != 0:
        old_length += 1
        red_place = np.concatenate((red_place, np.array([[0, 0, 0]])), axis=0)
    red_place = np.array(red_place.reshape((-1, 100, 3)), np.uint8)
    cv2.namedWindow("Target", 2)
    cv2.imshow("Target", red_place)
    red_hsv = cv2.cvtColor(red_place, cv2.COLOR_RGB2HSV)
    def getTarget(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("target: ", red_hsv[y, x])
    cv2.setMouseCallback("Target", getTarget)
    dark_place = dark_place.reshape((-1, 100, 3))
    dark_place = np.array(dark_place, np.uint8)
    cv2.namedWindow("rest", 2)
    cv2.imshow("rest", dark_place)
    dark_hsv = cv2.cvtColor(dark_place, cv2.COLOR_RGB2HSV)
    def getpos(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(dark_hsv[y, x])
    cv2.setMouseCallback("rest", getpos)

    cv2.namedWindow("mask", 2)
    cv2.imshow('mask', mask)
    cv2.waitKey(0)

    #Method 2
    # mask = np.zeros_like(red_colors)
    # for x in range(red_colors.shape[0]):
    #     for y in range(red_colors.shape[1]):
    #         r, g, b = red_colors[x, y]
    #         h, s, v = rgb2hsv(r, g, b)
    #         mask[x, y, :] = 0
    #         if 0 < h < 10 or 156 < h < 180:
    #             if 43 < s < 255:
    #                 if 46 < v < 255:
    #                     mask[x, y, :] = 255
    # fig, axs = plt.subplots(2)
    # axs[0].imshow(red_colors)
    # axs[1].imshow(mask, cmap='gray', vmin=0, vmax=255)
    # plt.show()