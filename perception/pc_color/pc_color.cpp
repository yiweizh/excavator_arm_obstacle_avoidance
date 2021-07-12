#include<iostream>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/principal_curvatures.h>
#include <boost/thread/thread.hpp>
#include <math.h>

using namespace std;

// 分割出底座函数
pcl::PointCloud<pcl::PointXYZRGB>::Ptr holder_extraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr holder(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloudXYZRGBtoXYZHSV(*cloud, *cloud_HSV);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (int i = 0; i < cloud->points.size(); i++)                         //holder
	{
		if (cloud_HSV->points[i].h < 250 && cloud_HSV->points[i].h > 180)            // increase the first parameter to have better visualisation of holder
		{
			inliers->indices.push_back(i);
		}
	}
	pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);
	eifilter.setInputCloud(cloud);
	eifilter.setIndices(inliers);
	eifilter.filter(*holder);

	return holder;
}
// 分割出块茎
pcl::PointCloud<pcl::PointXYZRGB>::Ptr body_extraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr body(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloudXYZRGBtoXYZHSV(*cloud, *cloud_HSV);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (int i = 0; i < cloud->points.size(); i++)                         //holder
	{
		if ((cloud_HSV->points[i].h < 50 && cloud_HSV->points[i].h > 0) || cloud_HSV->points[i].h > 320)            // increase the first parameter to have better visualisation of holder
		{
			inliers->indices.push_back(i);
		}
	}
	pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);
	eifilter.setInputCloud(cloud);
	eifilter.setIndices(inliers);
	eifilter.filter(*body);

	return body;
}

int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//*加载点云pcd文件
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("1.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file \n");
		return(-1);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr holder1 = holder_extraction(cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr body1 = body_extraction(cloud);


	// 点云可视化
	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(body1);
	while (!viewer.wasStopped()) 
	{
	}
		
	system("pause");
	return 0;
}
