# Perception Tutorial
## PCL Code
### Segmentation
The segmentation part is found online. It consists of 
1. SAC_RANSAC with iterative way to segment out plane
2. kdtree with Euclidean cluster way to group the rest of point cloud.

### Visualization
The visualizer code is implemented by pyhuang. It can visualize two groups of point clouds. 

### Usage
To compile segmentation and visualization,
```
cd <source code directory>
mkdir build 
cd build/
cmake ..
make -j4
```

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
