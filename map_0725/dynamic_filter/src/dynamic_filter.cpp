#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt.h>               //NDT(正态分布)配准类头文件
#include <pcl/filters/approximate_voxel_grid.h> //滤波类头文件  （使用体素网格过滤器处理的效果比较好）
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cstring>
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr;

void function(PointCloudPtr target_cloud, PointCloudPtr input_cloud, PointCloudPtr cloud_add, PointCloudPtr cloud_static)
{

    PointCloudPtr input_cloud_avg(new pcl::PointCloud<pcl::PointXYZI>);
    PointCloudPtr target_cloud_avg(new pcl::PointCloud<pcl::PointXYZI>);
    PointCloudPtr input_cloud_sor(new pcl::PointCloud<pcl::PointXYZI>);
    PointCloudPtr target_cloud_sor(new pcl::PointCloud<pcl::PointXYZI>);

    PointCloudPtr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    PointCloudPtr cloud_dynamic(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> input_sor;
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> input_avg;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> target_sor;
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> target_avg;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> cloud_add_sor;

    input_avg.setInputCloud(input_cloud);
    input_avg.setLeafSize(1, 1, 1);
    input_avg.filter(*input_cloud_avg);

    target_avg.setInputCloud(target_cloud);
    target_avg.setLeafSize(0.8, 0.8, 0.8);
    target_avg.filter(*target_cloud_avg);

    input_sor.setInputCloud(input_cloud_avg);
    input_sor.setMeanK(100);
    input_sor.setStddevMulThresh(0.3);
    input_sor.filter(*input_cloud_sor);

    target_sor.setInputCloud(target_cloud_avg);
    target_sor.setMeanK(100);
    target_sor.setStddevMulThresh(0.3);
    target_sor.filter(*target_cloud_sor);

    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

    ndt.setTransformationEpsilon(1); //为终止条件设置最小转换差异

    ndt.setStepSize(1); //为more-thuente线搜索设置最大步长

    ndt.setResolution(8.0); //设置NDT网格网格结构的分辨率（voxelgridcovariance）

    //设置匹配迭代的最大次数，这个参数控制程序运行的最大迭代次数，一般来说这个限制值之前优化程序会在epsilon变换阀值下终止
    //添加最大迭代次数限制能够增加程序的鲁棒性阻止了它在错误的方向上运行时间过长
    ndt.setMaximumIterations(100);

    ndt.setInputSource(input_cloud_sor); //源点云
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(target_cloud_sor); //目标点云

    Eigen::AngleAxisf init_rotation(3.12, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(42, 50, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    ndt.align(*output_cloud, init_guess);
    ROS_INFO("target_cloud red=%zu", target_cloud_sor->size());

    pcl::transformPointCloud(*input_cloud_sor, *output_cloud, ndt.getFinalTransformation());

    ROS_INFO("input_cloud green=%zu", input_cloud_sor->size());
    ROS_INFO("output_cloud blue=%zu", output_cloud->size());

    *cloud_add = *output_cloud + *target_cloud_sor;

    cloud_add_sor.setInputCloud(cloud_add);
    cloud_add_sor.setMeanK(100);
    cloud_add_sor.setStddevMulThresh(0.3);
    cloud_add_sor.filter(*cloud_static);

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_add);
    int K = 100;
    for (size_t i = 0; i < cloud_add->size(); ++i)
    {
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        cloud_add->points[i].intensity = 0;
        if (kdtree.nearestKSearch(cloud_add->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j)
            {
                cloud_add->points[i].intensity += pointNKNSquaredDistance[j];
            }
        }
    }

    return;

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
        target_color(target_cloud_sor, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZI>(target_cloud_sor, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "target cloud");

    // // Coloring and visualizing transformed input cloud (green).
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    //     input_color(cloud2, 0, 255, 0);
    // viewer_final->addPointCloud<pcl::PointXYZ>(cloud2, input_color, "input cloud");
    // viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
    //                                                1, "input cloud");

    //Coloring and visualizing transformed transform cloud (blue).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
        output_color(output_cloud, 0, 0, 255);
    viewer_final->addPointCloud<pcl::PointXYZI>(output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "output cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_sample");
    ros::NodeHandle nh;
    string target_param, input_param, cloud_add_param, cloud_static_param;
    ros::param::get("~target_param", target_param);
    ros::param::get("~input_param", input_param);
    ros::param::get("~cloud_add_param", cloud_add_param);
    ros::param::get("~cloud_static_param", cloud_static_param);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(target_param, *target_cloud) == -1)
    {
        ROS_ERROR("Couldn't read file target_cloud \n");
        std::cerr << target_param << std::endl;
        return -1;
    }
    ROS_INFO("load target_cloud = %zu", target_cloud->size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_param, *input_cloud) == -1)
    {
        ROS_ERROR("Couldn't read file input_cloud.pcd \n");
        std::cerr << input_param << std::endl;
        return -1;
    }
    ROS_INFO("load input_cloud = %zu", input_cloud->size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_add(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_static(new pcl::PointCloud<pcl::PointXYZI>);
    function(target_cloud, input_cloud, cloud_add, cloud_static);

    //把PointCloud对象数据存储在 test.pcd文件中
    pcl::io::savePCDFileASCII(cloud_add_param, *cloud_add);
    //打印输出存储的点云数据
    std::cerr << "Saved " << cloud_add->size() << " data points to cloud_add.pcd." << std::endl;

    pcl::io::savePCDFileASCII(cloud_static_param, *cloud_static);

    //打印输出存储的点云数据
    std::cerr << "Saved " << cloud_static->size() << " data points to cloud_static.pcd." << std::endl;
    ros::spin();
    return 0;
}
