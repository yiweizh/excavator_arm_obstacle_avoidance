#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/filters/extract_indices.h>
typedef pcl::PointXYZ PointT;
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0,0,0);
}
 
int
 main (int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>), cloud2(new pcl::PointCloud<PointT>);
    pcl::PCDReader reader;
    // 读入点云PCD文件
    reader.read(argv[1],*cloud1);
    reader.read(argv[2], *cloud2);
 
    std::cerr << "Point cloud1 data: " << cloud1->points.size () << " points" << std::endl;

    std::cerr << "Point cloud2 data: " << cloud2->points.size () << " points" << std::endl;

    pcl::visualization::PCLVisualizer viewer ("Segmentation Result");
    int v1(0);
    int v2(1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer.addPointCloud(cloud1, argv[1], v1);
    viewer.addPointCloud(cloud2, argv[2], v2);

    // Set background color
    float bckgr_gray_level = 0.0;  // Black
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size
    // viewer.runOnVisualizationThreadOnce(viewerOneOff);
    while(!viewer.wasStopped()){
      viewer.spinOnce();
    }
    return (0);
}
