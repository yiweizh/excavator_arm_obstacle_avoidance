# Perception Tutorial
## Dependecies
This project works on Ubuntu.
### Ubuntu 18.04
Empirically speak, Ubuntu 18.04 makes your life easier when you have to build pcl from source code. The dependencies are relatively easy to install.

To install pcl(Point Cloud Library)
```
sudo apt-get install libpcl-dev
```
If the above pcl version doesn't support, try to build it from source code.

1. Install Dependencies:
```
sudo apt-get update  
sudo apt-get install git build-essential linux-libc-dev
sudo apt-get install cmake cmake-gui
sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
sudo apt-get install mpi-default-dev openmpi-bin openmpi-common 
sudo apt-get install libflann1.8 libflann-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libvtk5.10-qt4 libvtk5.10 libvtk5-dev
sudo apt-get install libqhull* libgtest-dev
sudo apt-get install freeglut3-dev pkg-config
sudo apt-get install libxmu-dev libxi-dev
sudo apt-get install mono-complete
sudo apt-get install qt-sdk openjdk-8-jdk openjdk-8-jre
```
2. Build pcl
```
cd pcl
mkdir build
cd build/
cmake-gui
(click configure, generate)
make
sudo make install

```
Tips:
You might need to build VTK8 from source code. You can download it on VTK official website.

### Ubuntu 16.04
You have to build pcl from source code. The dependency installation is insane.
1. qhull
Go to qhull.org and download version 8.0.2. Build it from source code (follow its readme.md)
2. qt5.9.5
Find qt-...-5.9.5.run online and follow some tutorial to install it. When you cmake pcl, you might not be able to find qt5. Search qt5Config.cmake and add that directory to CMakeLists.txt in pcl. You should be able to find qt5 this time.
3. vtk8
```
source code, follow readme.md
```
4. boost1.65
```
source code, lots of online tutorials
```
5. CMake (required: 3.11.1)
If you find errors like "can not make imported target"(I'm not sure , something like this), go to CMake official website, download more recent version of CMake and build it from source.

Good luck!

### Other dependencies
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
