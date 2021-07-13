#include <pcl/memory.h>  // for pcl::make_shared
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <dirent.h>
#include <string>
#include <vector>
using namespace std;

using pcl::visualization::PointCloudColorHandlerGenericField;
// using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables 
    //our visualizer
pcl::visualization::PCLVisualizer* p, *res;
//its left and right viewports
int vp_1, vp_2, vp_3;

//convenient structure to handle our pointclouds
struct PCD
{
    PointCloud::Ptr cloud;
    std::string f_name;

    PCD() : cloud(new PointCloud) {};
};

struct PCDComparator
{
    bool operator () (const PCD& p1, const PCD& p2)
    {
        return (p1.f_name < p2.f_name);
    }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray(const PointNormalT& p, float* out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};



/** \brief Display source and target on the vp viewport of the visualizer
 *
 */
void showClouds(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source, int vp)
{
    std::stringstream ss, ss1;
    ss<< vp << "_target";
    ss1 << vp << "_source";
    std::cout << ss.str() << " "<<ss1.str()<< std::endl;
    p->removePointCloud(ss.str());
    p->removePointCloud(ss1.str());

    // PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
    // PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);
    // p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
    // p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);
    p->addPointCloud(cloud_target, ss.str(), vp);
    p->addPointCloud(cloud_source, ss1.str(), vp);

    PCL_INFO("Press q to begin the registration.\n");
    p->spin();
}

/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
    p->removePointCloud("2_source");
    p->removePointCloud("2_target");


    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
    if (!tgt_color_handler.isCapable())
        PCL_WARN("Cannot create curvature color handler!\n");

    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
    if (!src_color_handler.isCapable())
        PCL_WARN("Cannot create curvature color handler!\n");


    p->addPointCloud(cloud_target, tgt_color_handler, "2_target", vp_2);
    p->addPointCloud(cloud_source, src_color_handler, "2_source", vp_2);

    p->spinOnce();
}


/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void loadData(string dir_path, vector<string>& files, std::vector<PCD, Eigen::aligned_allocator<PCD> >& models)
{
    std::string extension(".pcd");
    // Suppose the first argument is the actual test model
    for (int i = 0; i < files.size(); i++)
    {
        std::string fname = dir_path + files[i];
        // Needs to be at least 5: .plot
        if (fname.size() <= extension.size())
            continue;

        // std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);

        //check that the argument is a pcd file
        if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
        {
            // Load the cloud and saves it into the global list of models
            PCD m;
            m.f_name = fname;
            pcl::io::loadPCDFile(fname, *m.cloud);
            //remove NAN points from the cloud
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

            models.push_back(m);
        }
    }
}



/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, const PointCloud::Ptr source, const PointCloud::Ptr target, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample = false)
{
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize(0.05, 0.05, 0.05);
        grid.setInputCloud(cloud_src);
        grid.filter(*src);

        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }


    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);

    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    //
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
    point_representation.setRescaleValues(alpha);

    //
    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon(1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance(0.15);
    // Set the point representation
    reg.setPointRepresentation(pcl::make_shared<const MyPointRepresentation>(point_representation));

    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);



    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(2);
    for (int i = 0; i < 30; ++i)
    {
        PCL_INFO("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation() * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

        prev = reg.getLastIncrementalTransformation();

        // visualize current state
        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
    }

    //
  // Get the transformation from target to source
    targetToSource = Ti.inverse();

    //
    // Transform target back in source frame
    // pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
    pcl::transformPointCloud(*target, *output, targetToSource);

    p->removePointCloud("2_source");
    p->removePointCloud("2_target");

    // PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
    // PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
    // p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
    // p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
    p->addPointCloud(output, "2_target", vp_2);
    p->addPointCloud(source, "2_source", vp_2);

    PCL_INFO("Press q to continue the registration.\n");
    p->spin();

    p->removePointCloud("source");
    p->removePointCloud("target");

    //add the source to the transformed target
    *output += *source;

    final_transform = targetToSource;
}

/** \brief segment a PointCloud dataset and return the plane and the rest
  * \param cloud_src the source PointCloud
  * \param cloud_p the plane PointCloud
  * \param cloud_f the rest PointCloud
  */
void segmentPoint(const PointCloud::Ptr cloud_src, PointCloud::Ptr cloud_p, PointCloud::Ptr cloud_f, std::size_t iteration){
    std::cout << "PointCloud before filtering has: " << cloud_src->points.size () << " data points." << std::endl; //*
    pcl::VoxelGrid<PointT> vg;
    PointCloud::Ptr cloud_filtered (new PointCloud);
    vg.setInputCloud (cloud_src);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
    //创建平面模型分割的对象并设置参数
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); //设置聚类的内点索引
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//平面模型的因子
    PointCloud::Ptr cloud_plane (new PointCloud ());
    // PointCloud::Ptr total_plane (new PointCloud());

    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);    //分割模型
    seg.setMethodType (pcl::SAC_RANSAC);       //随机参数估计方法
    seg.setMaxIterations (100);                //最大的迭代的次数
    seg.setDistanceThreshold (0.02);           //设置阀值

    int i = 0;
    int nr_points = (int)cloud_filtered->points.size ();//剩余点云的数量
    while (cloud_filtered->points.size () > 0.3 * nr_points){
        // 从剩余点云中再分割出最大的平面分量 （因为我们要处理的点云的数据是两个平面的存在的）
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0) //如果内点的数量已经等于0，就说明没有
        {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }

        // 从输入的点云中提取平面模型的内点
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);        //提取内点的索引并存储在其中
        extract.setNegative (false);

        // 得到与平面表面相关联的点云数据
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        // std::stringstream out_name;
        // out_name <<  argv[2];
        // pcl::io::savePCDFileASCII("plane" + out_name.str()+ ".pcd",*cloud_plane);
        *cloud_p += *cloud_plane;
        //  // 移去平面局内点，提取剩余点云
        extract.setNegative (true);
        PointCloud::Ptr temp(new PointCloud);
        extract.filter (*temp);
        *cloud_f += *temp;
        *cloud_filtered = *temp;
    }
    // return cloud plane (cloud_p) and cloud rest (cloud_f)

    std::stringstream plane_name;
    plane_name << "plane_" << iteration <<".pcd";
    pcl::io::savePCDFileASCII(plane_name.str(), *cloud_p);
    std::stringstream rest_name;
    rest_name << "rest_" << iteration << ".pcd";
    pcl::io::savePCDFileASCII(rest_name.str(), *cloud_f);

    // 创建用于提取搜索方法的kdtree树对象
    // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud (cloud_filtered);

    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<PointT> ec;   //欧式聚类对象
    // ec.setClusterTolerance (0.02);                     // 设置近邻搜索的搜索半径为2cm
    // ec.setMinClusterSize (100);                 //设置一个聚类需要的最少的点数目为100
    // ec.setMaxClusterSize (25000);               //设置一个聚类需要的最大点数目为25000
    // ec.setSearchMethod (tree);                    //设置点云的搜索机制
    // ec.setInputCloud (cloud_filtered);
    // ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
    // //迭代访问点云索引cluster_indices,直到分割处所有聚类
    // int j = 0;
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    // { //迭代容器中的点云的索引，并且分开保存索引的点云
    //     PointCloud::Ptr cloud_cluster (new PointCloud);
    //     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    //     //设置保存点云的属性问题
    //     cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    //     cloud_cluster->width = cloud_cluster->points.size ();
    //     cloud_cluster->height = 1;
    //     cloud_cluster->is_dense = true;

    //     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;  
    //     j++;
    //     *add_cloud+=*cloud_cluster;
    //     // std::stringstream out_name;
    //     // out_name <<  argv[2];
    //     // pcl::io::savePCDFileASCII("rest"+out_name.str()+".pcd",*add_cloud);
    // }
}



int endsWith(string s,string sub){
        return s.rfind(sub)==(s.length()-sub.length())?1:0;
}

/* ---[ */
int main(int argc, char** argv)
{
    
    struct dirent *dir_ptr;
    DIR *dir;
    // string path = "/home/pyhuang/VE450/excavator_arm_obstacle_avoidance/perception/dataset/";
    string path(argv[1]);
    dir=opendir(path.c_str());
    vector<string> files;
    cout<<"list of files: "<<endl;
    string extension(".pcd");

    while((dir_ptr =readdir(dir))!=NULL){
        if(dir_ptr->d_name[0] == '.' || !endsWith(dir_ptr->d_name, extension))
            //get rid of . and ..
            continue;
        files.push_back(dir_ptr->d_name);
    }

    closedir(dir);
    // Load data

    std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
    sort(files.begin(), files.end());
    for (int i = 0; i < files.size(); ++i){
        cout << files[i] <<endl;
    }
    loadData(path, files, data);

    // Check user input
    if (data.empty())
    {
        PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]\n", argv[0]);
        PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc\n");
        return (-1);
    }
    PCL_INFO("Loaded %d datasets.\n", (int)data.size());

    // Create a PCLVisualizer object
    p = new pcl::visualization::PCLVisualizer(argc, argv, "Pairwise Incremental Registration example");
    p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
    p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

    res = new pcl::visualization::PCLVisualizer(argc, argv, "Global result demo");
    p->createViewPort(0.0, 0, 0.0, 1.0, vp_3);

    PointCloud::Ptr result(new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
    
    PointCloud::Ptr res_temp(new PointCloud);
    int res_is_temp = 0;
    for (std::size_t i = 1; i < data.size(); ++i)
    {
        source = data[i - 1].cloud;
        target = data[i].cloud;

        // Add visualization data
        showClouds(source, target, vp_1);

        PointCloud::Ptr temp(new PointCloud);
        PCL_INFO("Aligning %s (%zu) with %s (%zu).\n", data[i - 1].f_name.c_str(), static_cast<std::size_t>(source->size()), data[i].f_name.c_str(), static_cast<std::size_t>(target->size()));
        
        //Segment out the plane and use the rest to align
        PointCloud::Ptr plane_src(new PointCloud);
        PointCloud::Ptr rest_src(new PointCloud);
        PointCloud::Ptr plane_tar(new PointCloud);
        PointCloud::Ptr rest_tar(new PointCloud);

        segmentPoint(source, plane_src, rest_src, i);
        segmentPoint(target, plane_tar, rest_tar, i);

        if(i==9){
            res->removePointCloud("Result");
            res->addPointCloud(rest_tar, "Result");
            res->spin();
        }
        
        showClouds(rest_src, rest_tar, vp_2);

        // pairAlign(source, target, temp, pairTransform, true);
        pairAlign(rest_src, rest_tar, source, target, temp, pairTransform, true);

        //transform current pair into the global transform
        pcl::transformPointCloud(*temp, *result, GlobalTransform);

        if(res_is_temp == 0){
            *res_temp = *result;
            res_is_temp = 1;
        }else{
            *res_temp += *result;
        }
        //visualize result of global transform
        res->removePointCloud("Result");
        res->addPointCloud(res_temp, "Result");
        res->spin();
        //update the global transform
        GlobalTransform *= pairTransform;

        //save aligned pair, transformed into the first cloud's frame
        std::stringstream ss;
        ss << "transform_"<< i << ".pcd";
        pcl::io::savePCDFile(ss.str(), *result, true);
    }
}

