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

if __name__ == '__main__':
    pcd_path = "/home/pyhuang/VE450/excavator_arm_obstacle_avoidance/src/motion_planning/scripts/pointcloud_xyzrgba.txt"
    rbg_list = load_color_to_ndarray(pcd_path)
    for i in range(0, 530000 - len(rbg_list)):
        rbg_list.append([0, 0, 0])
    rbg_list = np.array(rbg_list, dtype=np.uint8)
    print(rbg_list.shape)
    red_colors = rbg_list.reshape((530, 1000, 3))
    
    #Method 1
    rbg_list = rbg_list.reshape((1, rbg_list.shape[0], 3))
    hsv_img = cv2.cvtColor(rbg_list, cv2.COLOR_RGB2HSV)
    mask = segmentRed(hsv_img)
    mask = mask.reshape((530, 1000))
    fig, axs = plt.subplots(2)
    axs[0].imshow(red_colors)
    axs[1].imshow(mask, cmap='gray', vmin=0, vmax=255)
    plt.show()

    #Method 2
    # mask = np.zeros_like(red_colors)
    # for x in range(red_colors.shape[0]):
    #     for y in range(red_colors.shape[1]):
    #         r, g, b = red_colors[x, y]
    #         h, s, v = rgb2hsv(r, g, b)
    #         if 0 < h < 10 or 156 < h < 180:
    #             mask[x, y, :] = 255
    #         else:
    #             mask[x, y, :] = 0
    # fig, axs = plt.subplots(2)
    # axs[0].imshow(red_colors)
    # axs[1].imshow(mask, cmap='gray', vmin=0, vmax=255)
    # plt.show()