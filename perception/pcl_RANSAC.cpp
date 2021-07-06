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
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0,0,0);
}
 
int
 main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_ground(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    // 读入点云PCD文件
    reader.read(argv[1],*cloud);
 
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.1);
    pass.filter (*cloud_filtered);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    // seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    // seg.setDistanceThreshold (0.04);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold (std::atof(argv[2]));
    // seg.setDistanceThreshold (0.15);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
 
    

    // 提取地面
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*cloud_ground);
 
    std::cerr << "Ground cloud after filtering: " << std::endl;
    std::cerr << *cloud_ground << std::endl;
 
    // pcl::PCDWriter writer;
    // writer.write<pcl::PointXYZ> ("3dpoints_ground.pcd", *cloud_filtered, false);
    
 
    // 提取除地面外的物体
    extract.setNegative (true);
    extract.filter (*cloud_object);
 
    std::cerr << "Object cloud after filtering: " << std::endl;
    std::cerr << *cloud_object << std::endl;
 
    // writer.write<pcl::PointXYZ> ("3dpoints_object.pcd", *cloud_filtered, false);
 
    // 点云可视化
    // pcl::visualization::CloudViewer viewer("Filtered");
    pcl::visualization::PCLVisualizer viewer ("Segmentation Result");
    int v1(0);
    int v2(1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer.addPointCloud(cloud_ground, "ground", v1);
    viewer.addPointCloud(cloud_object, "filtered", v2);

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
