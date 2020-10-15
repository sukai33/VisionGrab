//
// Created by ty on 20-10-7.
//


#include <iostream>

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>
#include <opencv/cxeigen.hpp>


using namespace std;
using namespace cv;

string template2target_Rt_filepath = "./template2target_Rt.xml";

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// 表面法相量的信息
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
//特征直方图
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PCLHandler;
// 特征点云  获取法线 与 征描述子信息特征直方图特 类
class FeatureCloud{

private:
    // 保存点云数据
    PointCloud::Ptr xyz_;
    // 表面法向量信息
    SurfaceNormals::Ptr normals_;
    // 特征描述子信息
    LocalFeatures::Ptr features_; // 快速点特征直方图
    // 邻域搜索方法
    SearchMethod::Ptr search_method_xyz_; // KDTree

    float normal_radius_;   // 法向量构建时，邻域参考半径
    float feature_radius_;  // 特征描述子构建时，邻域参考半径  要比normal_radius_大一些或相等,不能小

public:
    // 初始化
    FeatureCloud() : search_method_xyz_(new SearchMethod), normal_radius_(0.02f), feature_radius_(0.02f) {}

    PointCloud::Ptr getPointCloud(){
        return xyz_;
    };
    //获取面法向量信息
    SurfaceNormals::Ptr getSurfaceNormals(){
        return normals_;
    };
    //获取特征描述子信息 快速点特征直方图
    LocalFeatures::Ptr getLocalFeatures(){
        return features_;
    };
    // 计算表面法线 xyz_:输入点云  normals_:输出得到表面法向量信息
    void computeSurfaceNormals(){
        // 创建 表面法线堆内存对象  储存表面法线的对象
        normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
        //创建计算表面法线估算的对象   输入:pcl::PointXYZ点  输出：pcl::Normal 法线特征
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nor;
        // 设置输入点云
        nor.setInputCloud(xyz_);
        //邻域搜索方法 KDTree
        nor.setSearchMethod(search_method_xyz_);
        // 设置搜索半经
        nor.setRadiusSearch(normal_radius_);
        // 输出法线
        nor.compute(*normals_);
    }
    // 计算特征描述子 xyz_:输入点云    features_: 输出得到特征描述子信息 快速点特征直方图
    void computeLocalFeatures(){
        // 创建 特征描述子对象  储存特征描述子的对象
        features_ = LocalFeatures::Ptr(new LocalFeatures);
        //创建特征描述子估算的对象   输入:pcl::PointXYZ点  输入: pcl::Normal法线   输出：pcl::features_ 特征描述子
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud(xyz_);
        fpfh_est.setInputNormals(normals_);
        //邻域搜索方法 KDTree
        fpfh_est.setSearchMethod(search_method_xyz_);
        fpfh_est.setRadiusSearch(feature_radius_);
        fpfh_est.compute(*features_);
    }
    // 处理输入点云 计算表面法线及计算特征描述子
    void processInput(){
        // 计算表面法线 xyz_:输入点云  normals_:输出得到表面法向量信息
        computeSurfaceNormals();
        // 计算特征描述子 xyz_:输入点云    features_: 输出得到特征描述子信息 快速点特征直方图
        computeLocalFeatures();
    }

    // 通过点云文件路径 加载点云
    void loadInputCloud(const string &pcd_filename){
        // 创建点云
        xyz_ = PointCloud::Ptr(new PointCloud);
        // 通过点云文件路径得到点云数据
        pcl::io::loadPCDFile(pcd_filename, *xyz_);
        // 处理输入点云得到: 表面法线normals_ 及 特征描述子features_
        processInput();
    }

    void setInputCloud(PointCloud::Ptr cloud) {
        //设置输入点云
        xyz_ = cloud;
        // 处理输入点云
        processInput();
    }

};
/**
 *
 *  匹配器，执行模板匹配
 * 此对象专门用来进行刚体变换匹配
 * 匹配成功的模板点云 + 模板变换到目标的变换矩阵
 */
class TemplateAligment{

private:
    // FeatureCloud特征点云  获取点云的法线 与 征描述子信息特征直方图特
    vector<FeatureCloud> templates_;
    // 特征点云  获取点云的法线 与 征描述子信息特征直方图特
    FeatureCloud target_;
    // 初始化 实现了RANSAC姿态估计对象 SAC-IA估算对象 在这个setTargetCloud方法中初始化; 参数1:场景输入点云的类型 参数2:模板的输入点云的类型 参数3:输出点特征的直方图的类型:FPFHSignature33的估算器
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    // 最小采样距离
    float min_sample_distance_;
    // 最大的响应距离,如果发现两个点太远了,认为它没有在模型内部
    float max_correspondence_distance_;
    // 最大迭代次数
    float max_iterations_;

public:

    TemplateAligment()
            : min_sample_distance_(0.05f),
              max_correspondence_distance_(0.01f * 0.01f),
              max_iterations_(500) {
        // 给SAC-IA估算对象设置最小采样距离
        sac_ia_.setMinSampleDistance(min_sample_distance_);
        // 给SAC-IA估算对象设置最大的响应距离,如果发现两个点太远了,认为它没有在模型内部
        sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
        // 给SAC-IA估算对象设置最大迭代次数
        sac_ia_.setMaximumIterations(max_iterations_);
    }
    // 添加模板点云
    void addTemplateCloud(FeatureCloud &cloud) {
        templates_.push_back(cloud);
    }
    // 给估算对象SAC-IA初始化,同时传入目标点云数据及目标点云特征描述子
    void setTargetCloud(FeatureCloud &target_cloud){
        target_ = target_cloud;
        // 给估算对象SAC-IA初始化 设置target cloud ; 设置目标点云 target_cloud.getPointCloud()获取点云 xyz_
        sac_ia_.setInputTarget(target_cloud.getPointCloud());
        // 设置目标点云特征描述子
        sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
    };
     // 对所有模板进行匹配，得到结果results
    struct Result {
        // 模板对应的匹配分数
        float fitness_score;

        // 转换矩阵
        Eigen::Matrix4f transformation;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /**
     * 3.
     * 核心代码:
     * 执行匹配，并将结果设置给 result
     * @param result
     * @param template_cloud
     */
    void align(FeatureCloud &template_cloud, TemplateAligment::Result &result){
        // 设置输入源 输入点云
        sac_ia_.setInputSource(template_cloud.getPointCloud());
        // 设置输入点云特征
        sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

        PointCloud registration_output;
        // 执行配准
        sac_ia_.align(registration_output);

        // 给result设置数据 分数
        result.fitness_score = (float) sac_ia_.getFitnessScore(max_correspondence_distance_);
        // 对应的变换矩阵  模板匹配到的变换矩阵
        result.transformation = sac_ia_.getFinalTransformation();

    };
    // 2. 对所有模板进行匹配，得到结果results
    void alignAll(std::vector<Result, Eigen::aligned_allocator<Result>>& results){
        results.resize(templates_.size());
        for (int i = 0; i < templates_.size(); ++i) {
              /** 3.
                * 核心代码:
                * 执行匹配，并将结果设置给 result
                * @param result
                * @param template_cloud
                */
            align(templates_[i], results[i]);
        }
    };

    /**
     *  1.
     * 在templates_中查找与target_匹配结果最佳的点云索引
     * 根据匹配分数判断（越低越好）
     * 查找最优解
     */
    int findBestAligment(TemplateAligment::Result &result){
        // 对所有模板进行匹配，得到结果results
        std::vector<Result, Eigen::aligned_allocator<Result>> results;
        //2. 对所有模板进行匹配，得到结果results
        alignAll(results);

        // 对结果进行循环（匹配分数），得到分数最佳的result对象，返回对应的索引
        int best_template_index = 0;
        float lowest_score = results[0].fitness_score;
        //排序
        for (int i = 0; i < results.size(); ++i) {
            Result &result = results[i];
            // 取分数最小的
            if (result.fitness_score < lowest_score) {
                lowest_score = result.fitness_score;
                best_template_index = i;
            }
        }
        // 最佳匹配值
        result = results[best_template_index];
        // 最佳匹配值得索引
        return best_template_index;
    };
};

// 直通滤波
void filterCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered,
        const char *filter_name, double limitMin, double limitMax){
    pcl::PassThrough<pcl::PointXYZ> passFilter;
    passFilter.setInputCloud(cloud);
    passFilter.setFilterFieldName(filter_name);
    passFilter.setFilterLimits(limitMin, limitMax);
    passFilter.setFilterLimitsNegative(false);
    passFilter.filter(*cloud_filtered);
}
// 降采样
void downsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                  const float voxel_grid_size) {
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(input_cloud);
    // 设置叶子节点的大小lx, ly, lz
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    vox_grid.filter(*tempCloud);
    output_cloud = tempCloud;
}



/**
 *
 * 输入： 模板点云文件（一个或多个），目标场景点云
 *
 * 输出： 匹配成功的模板点云 + 模板变换到目标的变换矩阵
 * 启动 加参数 : ./data1/object_templates.txt ./data1/person.pcd  人脸
 * 调用格式： ./05-TemplateAlignment ./data1/object_templates.txt ./data1/person.pcd
             ./data/object_templates.txt ./data/target_scene.pcd
 *
 * 优化：
 *  1. 实时根据相机生成场景点云 （直通滤波，降采样）
 *  2. 制作同一个物体的不同角度模板
 *  3. 把多个模板融合成同一个完整的点云（ICP、NDT）
 *  /data1  pcl_viewer person.pcd object_template_*pcd
     按 u 标注尺
     按 1 切换颜色
     按 2 x方向颜色
     按 3 y方向颜色
     按 4 z方向颜色
 * 
 */
int main(int argc, char *argv[]) {


    if (argc < 3){
        std::cerr << "请把模板点云和场景点云作为参数" << std::endl;
        return -1;
    }

    // 把所有的模板点云保存到列表中
    std::vector<FeatureCloud> object_templates;
    std::ifstream input_stream(argv[1]);//  读取文件第一个参数  ./data1/object_templates.txt
     // 把文件中读取的每一行数据赋值给 pcd_filename
    std::string pcd_filename;
    while (input_stream.good()) {//判断.txt中是否有数据
        // 读取文件里的每一行对应的pcd文件内容，存到object_templates中
        std::getline(input_stream, pcd_filename);
        if (pcd_filename.empty() || pcd_filename.at(0) == '#') {
            continue;
        }
        // 加载特征点云, 并且计算表面法向量 & FPFH直方图
        FeatureCloud template_cloud;
        // 把文件中读取的每一行数据加载特征点云, 并且计算表面法向量
        template_cloud.loadInputCloud(pcd_filename);
        // 数据点云加入到集合中
        object_templates.push_back(template_cloud);
    }
    input_stream.close();

    // 加载场景点云
    PointCloud::Ptr cloud(new PointCloud);
    pcl::io::loadPCDFile(argv[2], *cloud);
    // 直通滤波
    filterCloud(cloud, cloud, "z", 0, 1.10);
    filterCloud(cloud, cloud, "x", -1.0, 0.37);

    // 降采样
    downsampling(cloud, cloud, 0.005f);

    // 把目标点云封装成FeatureCloud
    FeatureCloud target_cloud;
    target_cloud.setInputCloud(cloud);

    // 创建匹配器，执行模板匹配
    TemplateAligment template_align;
    // 遍历所有的模板点云object_templates集合  std::vector<FeatureCloud> object_templates;
    for (FeatureCloud object_templte: object_templates) {
        // 添加模板点云
        template_align.addTemplateCloud(object_templte);
    }

    // 设置目标点云
    template_align.setTargetCloud(target_cloud);
    //得到最佳匹配模板
    TemplateAligment::Result result;
    // 执行配准
    // 返回最佳模板配准结果索引
    int best_index = template_align.findBestAligment(result);
    // 返回最佳模板配准结果对象
    FeatureCloud best_template = object_templates[best_index];

    // 最佳匹配的变换，匹配分数 低于0.00002较好
    std::cout << "score: " << result.fitness_score << " index: " << best_index << std::endl;
    std::cout << "变换矩阵：\n" << result.transformation << std::endl;

    Eigen::Matrix3f rotation = result.transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = result.transformation.block<3, 1>(0, 3);
    // 旋转矩阵转欧拉角. 2:z 1:y 0:x  按zyx顺序旋转
    Eigen::Vector3f eulerAngles = rotation.eulerAngles(2, 1, 0);
    // eulerAngles.transpose()  转置 一行3列
    std::cout << "yaw(z), pitch(y), roll(x): " << eulerAngles.transpose() * 180 / M_PI << std::endl;
    std::cout << "translation: " << translation.transpose() << std::endl;

    // 对最佳匹配的模板执行以上变换
    PointCloud::Ptr transformed_template_cloud(new PointCloud);
    //*best_template.getPointCloud() 模板点云 通过 result.transformation 变化矩阵   转换成 transformed_template_cloud点云[改变模板点云的位姿]
    pcl::transformPointCloud(*best_template.getPointCloud(), *transformed_template_cloud, result.transformation);

    // 保存变换矩阵  string template2target_Rt_filepath = "./template2target_Rt.xml";
    FileStorage fs(template2target_Rt_filepath, FileStorage::WRITE);
    cv::Mat RMat, tMat;
    // eigen矩阵转mat
    cv::eigen2cv(rotation, RMat);
    cv::eigen2cv(translation, tMat);
    fs << "rotation" << RMat;
    fs << "translation" << tMat;
    fs.release();

    pcl::visualization::PCLVisualizer viewer("3D viewer");
    // 显示目标点云
    PCLHandler cloud_handler(cloud, 255, 100, 100);
    viewer.addPointCloud(cloud, cloud_handler, "cloud");

    // 显示匹配成功的模板点云(最佳匹配)
//    FeatureCloud &featureCloud = object_templates.at(0);
    PCLHandler template_cloud_handler(best_template.getPointCloud(), 100, 255, 255);
    viewer.addPointCloud(best_template.getPointCloud(), template_cloud_handler, "template_cloud");

    // 变换之后的模板点云（最佳匹配）
    PCLHandler transformed_template_cloud_handler(transformed_template_cloud, 255, 255, 255);
    viewer.addPointCloud(transformed_template_cloud, transformed_template_cloud_handler, "transformed_template_cloud");
    //显示坐标
    viewer.addCoordinateSystem(0.5);
    while (!viewer.wasStopped()) {//如果viewer没有停止
        viewer.spinOnce();
    }
}