//
// Created by ty on 20-9-30.
//


#include <iostream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void filterCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered,
        const char *filter_name, double limitMin, double limitMax){
    pcl::PassThrough<PointT> passFilter;
    passFilter.setInputCloud(cloud);
    passFilter.setFilterFieldName(filter_name);
    passFilter.setFilterLimits(limitMin, limitMax);
    passFilter.setFilterLimitsNegative(false);
    passFilter.filter(*cloud_filtered);
}
// 降采样 ...
void downsampling(pcl::PointCloud<PointT>::Ptr &input_cloud, pcl::PointCloud<PointT>::Ptr &output_cloud,
                  const float voxel_grid_size) {
    pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
    vox_grid.setInputCloud(input_cloud);
    // 设置叶子节点的大小lx, ly, lz
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    vox_grid.filter(*tempCloud);
    output_cloud = tempCloud;
}


/**
 * 将现有点云进行优化，取出目标区域
 */
int main(int argc, char *argv[]) {

    PointCloud::Ptr cloud(new PointCloud);
    pcl::io::loadPCDFile("./output1/table_scene_0.pcd", *cloud);
	//pcl::io::loadPCDFile("./output/table_scene_0_ds.pcd", *cloud);

    PointCloud::Ptr cloud_filtered(new PointCloud);

    // 直通滤波
    filterCloud(cloud, cloud_filtered, "x", -0.20, 0.40);
    filterCloud(cloud_filtered, cloud_filtered, "y", -0.40, 0.40);
    filterCloud(cloud_filtered, cloud_filtered, "z", 0.70, 1.15);

    // 降采样 ...
    downsampling(cloud_filtered, cloud_filtered, 0.003f);

    // 保留模板盒子区域
    PointCloud::Ptr cloud_copy(new PointCloud(*cloud_filtered));
    PointCloud::Ptr cloud_template(new PointCloud);
    filterCloud(cloud_copy, cloud_template, "x", 0.118007 , 0.241143);
    filterCloud(cloud_template, cloud_template, "y", 0.210088, 0.307214);
    filterCloud(cloud_template, cloud_template, "z", 0.70, 1.078);


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->addPointCloud(cloud_filtered, "cloud_filterd");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_hander(cloud_template, 0, 255, 0);
    viewer->addPointCloud(cloud_template, color_hander, "cloud_template");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_template");

    viewer->addCoordinateSystem(1.0);

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    // 退出的时候，保存两个点云
    // 场景点云
   // pcl::io::savePCDFile("target_scene.pcd", *cloud_filtered, true);
    pcl::io::savePCDFile("target_scene_0.pcd", *cloud_filtered, true);
    // 模板点云
//    pcl::io::savePCDFile("target_template.pcd", *cloud_template, true);
}