# Perception Tutorial
## Dependecies
This project works on Ubuntu.

To install pcl(Point Cloud Library)
```
sudo apt-get install libpcl-dev
```
To install boost
```
sudo apt-get install libboost-all-dev
```
You also need to install OpenCV 3.4.1 and [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md). The official websites are well-documented. You can follow their guides to install them.

Please notice:
Even though the pcl library installed above is somewhat problematic (but it works for this project), I personally discourage any attempt to build it from source code. The process of dependency installation made me insane.
## PCL Code
### PointCloud Saver
The code is used to save point cloud with aligned depth frame and color frames. It can also save the metadata associated with the frame.

Please notice that the header file "example.hpp" should be located at the source code of librealsense. The path can be `<librealsense>/examples/example.hpp`.

Other header files (`kbhit.hpp`, `cv-helpers.hpp`, `stb_image_write.h`) are either written by me or copied from librealsense source code. You can easily find them.
### Segmentation
The segmentation part is found online. It consists of 
1. SAC_RANSAC with iterative way to segment out plane
2. kdtree with Euclidean cluster way to group the rest of point cloud.

### Visualization
The visualizer code is implemented by pyhuang. It can visualize two groups of point clouds. 

### Usage
To compile pointcloud saver, segmentation and visualization,
```
cd <source code directory>
mkdir build 
cd build/
cmake ..
make -j4
```

To save point cloud, please make sure the realsense camera is connected.
```
mkdir dataset
cd dataset/
../build/pc_saver/pc_saver
<Press Enter to generate point cloud>
```
You can press multiple times to save frames and pcd files. Those saved files are located at dataset/.


To run segmentation, 
```
cd result/
../build/segment/segment <Path of PCD File>
```
The result pcd is stored within result/


To run visualization,
```
../build/visualizer/visualizer <Path of PCD 1> <Path of PCD 2>
```

To segment and visualize,
```
sh segVis.sh <Path of PCD File>
```
