//
// Created by ty on 20-9-30.
//


#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <CoordinateUtil.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
//输出的模板品平移旋转文件数据的路径  输出模板的旋转与平移的xml文件
const string outputTemplateRtPath = "./output/template_Rt.xml";

pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::PCLVisualizer::Ptr viewer;
pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_search(new pcl::search::KdTree<pcl::PointXYZ>);
//把选的点放在一个队列里,集合里
vector<cv::Point3d> pointQueue;
void
pp_callback(const pcl::visualization::PointPickingEvent &event, void *cookie) {
    //获取鼠标点中的索引
    int idx = event.getPointIndex();
    if (idx == -1)
        return;

    kd_search->setInputCloud(xyzcloud);

    // Return the correct index in the cloud instead of the index on the screen
    //返回一个正确的真实的索引替代刚才鼠标点的那个
    std::vector<int> indices(1);
    std::vector<float> distances(1);

    // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
    pcl::PointXYZ picked_pt;
    //event查找下刚才点的xyz,picked_pt点赋个值
    event.getPoint(picked_pt.x, picked_pt.y, picked_pt.z);
    //找到一个点picked_pt之后,在这个picked_pt点周围找1个点,结果放indices中,距离放distances中 [因为自己点的点不准,他帮你找邻近的一个点]
    kd_search->nearestKSearch(picked_pt, 1, indices, distances);
    //这个就是picked_pt我们想要的点
    PCL_INFO ("Point index picked: %d (real: %d) - [%f, %f, %f]\n", idx, indices[0], picked_pt.x, picked_pt.y,
              picked_pt.z);
    //拿到这个点的索引
    idx = indices[0];

    std::stringstream ss;
    ss << idx;
    if (viewer) {
        pcl::PointXYZ pos;
        event.getPoint(pos.x, pos.y, pos.z);
        // 在用户点的位置，添加一个文字 内容是点的索引 ,pos位置,0.002字体缩放大小,1.0 1.0 1.0 代表白色,前面ss.str()是文字内容后面ss.str()是标签
        viewer->addText3D<pcl::PointXYZ>(ss.str(), pos, 0.002, 1.0, 1.0, 1.0, ss.str());
        //把选的点放在一个队列里,集合里
        pointQueue.emplace_back(pos.x, pos.y, pos.z);

        std::cout << "目前共有点数："<< pointQueue.size() << std::endl;
    }
}


Mat output_RMat;
Mat output_tMat;
//space空格清除所有点及形状,z 清除上一个点 ,a通过点绘制坐标系,w保存
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *){
    //屏蔽键盘弹起
    if (!event.keyUp()) {
        return;
    }
    //获取键盘按下的内容
    string keySym = event.getKeySym();
//    std::cout << "sym: " << keySym << " code: " << event.getKeyCode() << std::endl;
    if (keySym == "space") {
        std::cout << "清除所有点及形状" << std::endl;
        pointQueue.clear();
        viewer->removeAllShapes();
    } else if (keySym == "z" || keySym == "Z") {
        std::cout << "清除上一个点，目前共有点数："<< pointQueue.size() << std::endl;
        pointQueue.pop_back();
    } else if (keySym == "a" || keySym == "A") {

        if (pointQueue.size() == 3) {
            //封装的一个工具类  pointQueue: 选的点的队列,集合  viewer:视图,在内部画添加了一个坐标系  output_RMat:旋转矩阵输出 output_tMat:平移矩阵输出
            CoordinateUtil::drawCoordinate(pointQueue, viewer, output_RMat, output_tMat);
            std::cout << "通过3个点绘制坐标系" << std::endl;
            pointQueue.clear();
        } else if (pointQueue.size() >= 4) {
            CoordinateUtil::drawCoordinate(pointQueue, viewer, output_RMat, output_tMat);
            std::cout << "通过4个点绘制坐标系" << std::endl;
            pointQueue.clear();
        } else {
            std::cout << "点个数必须>=3 ：" << pointQueue.size() << std::endl;
        }
    } else if (keySym == "w" || keySym == "W") {

        if (output_RMat.empty()) {
            std::cout << "还未得到变换矩阵" << std::endl;
            return;
        }
        //输出的模板品平移旋转文件数据的路径
        //const string outputTemplateRtPath = "./output/template_Rt.xml";
        FileStorage rtFs(outputTemplateRtPath, FileStorage::WRITE);
        rtFs << "rotation" << output_RMat;
        rtFs << "translation" << output_tMat;
        rtFs.release();
        std::cout << "变换矩阵已保存：" << outputTemplateRtPath << std::endl;
    }
}


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
/**
 * 通过点云构建抓取位姿
 *
 * input:
 *      点云文件
 *
 * output:
 *      目标物体姿态坐标系（旋转+平移） 平移:夹抓抓的中心位置 旋转:夹抓竖着抓还是横这抓
 *
 *    传入的参数  ./output/table_scene_0_ds.pcd
 *    控制台命令  ./03-TemplateRtMaker ./output/table_scene_0_ds.pcd
 *    控制台降采样命令 pcl_voxel_grid table_scene_0_ds.pcd table_scene_0_ds.pcd -leaf 0.003,0.003,0.003
 *    控制台显示点云图 pcl_viewer table_scene_0_ds.pcd
 *    控制台显示点云图 pcl_viewer -use_point_picking table_scene_0_ds.pcd    [shift + 鼠标左键可获取点云坐标]
 
 *    查看历史操作的命令 history
 */
int main(int argc, char *argv[]) {

    if (argc < 2) {
        PCL_ERROR("请使用点云文件作为参数\n");
        return -1;
    }

    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud_filtered(new PointCloud);

    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1) {
        PCL_ERROR("点云加载失败： %s \n", argv[0]);
        return -1;
    }

    PCL_INFO("点云加载成功，总数量： %d \n", cloud->points.size());

    // 直通滤波 (得到桌子区域)
    filterCloud(cloud, cloud_filtered, "x", -0.20, 0.40);
    filterCloud(cloud_filtered, cloud_filtered, "y", -0.40, 0.40);
    filterCloud(cloud_filtered, cloud_filtered, "z", 0.70, 1.15);
   //点云拷贝复制  把cloud_filtered 拷贝到xyzcloud中
    pcl::copyPointCloud(*cloud_filtered, *xyzcloud);

    // 0.003 降采样
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // 注册点选择回调函数 构建坐标系 鼠标选点回调   shift+左键选点  cloud_filtered:过滤后的点云做参数
    viewer->registerPointPickingCallback(&pp_callback, static_cast<void *>(&cloud_filtered));
    //键盘的回调 当用户键盘输入发生的时候执行 键盘按下与弹起都会执行
    viewer->registerKeyboardCallback(&keyboardEventOccurred);

    viewer->addPointCloud(cloud_filtered, "cloud_filterd");

    viewer->addCoordinateSystem(1.0);

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}