//
// Created by ty on 20-9-30.
//

// 顺序不能错
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <cstdlib>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <KinectCamera.h>
#include <iostream>

using namespace std;
using namespace cv;

const int ACTION_SPACE = 32;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
//string camera_in_params_path = "./calibration_in_params.yml";
string camera_in_params_path = "./assets/calibration_in_params.yml";
//const string output_dir = "./output";
const string output_dir = "./output1";

float qnan_ = std::numeric_limits<float>::quiet_NaN();
Eigen::Matrix<float, 1920, 1> colmap;
Eigen::Matrix<float, 1080, 1> rowmap;
const short w = 1920, h = 1080;
void prepareMake3D(const double cx, const double cy,
                   const double fx, const double fy) {
//    const int w = 512;
//    const int h = 424;
    float *pm1 = colmap.data();
    float *pm2 = rowmap.data();
    for (int i = 0; i < w; i++) {
        *pm1++ = (i - cx + 0.5) / fx;
    }
    for (int i = 0; i < h; i++) {
        *pm2++ = (i - cy + 0.5) / fy;
    }
}

/***
 * 通过彩色图和深度图实时生成点云图
 * @param colorMat
 * @param depthMat
 * @param cloud
 */
void updateCloud(Mat &colorMat, Mat &depthMat, PointCloud::Ptr &cloud) {
    const float *itD0 = (float *) depthMat.ptr();
    const char *itRGB0 = (char *) colorMat.ptr();

    if (cloud->size() != w * h)
        cloud->resize(w * h);


    pcl::PointXYZRGB *itP = &cloud->points[0];
    bool is_dense = true;

    for (size_t y = 0; y < h; ++y) {

        const unsigned int offset = y * w;
        const float *itD = itD0 + offset;
        const char *itRGB = itRGB0 + offset * 4;
        const float dy = rowmap(y);

        for (size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4) {
            const float depth_value = *itD / 1000.0f;

            if (!isnan(depth_value) && abs(depth_value) >= 0.0001) {

                const float rx = colmap(x) * depth_value;
                const float ry = dy * depth_value;
                itP->z = depth_value;
                itP->x = rx;
                itP->y = ry;

                itP->b = itRGB[0];
                itP->g = itRGB[1];
                itP->r = itRGB[2];
            } else {
                itP->z = qnan_;
                itP->x = qnan_;
                itP->y = qnan_;

                itP->b = qnan_;
                itP->g = qnan_;
                itP->r = qnan_;
                is_dense = false;
            }
        }
    }
    cloud->is_dense = is_dense;

}

/**
 * 采集彩色图片，采集深度图片
 *
 * 根据内参，实时生成点云图
 */
int main(int argc, char *argv[]) {

    KinectCamera *camera = new KinectCamera();

    if (!camera->isOpened()) {
        std::cerr << "未发现Kinect2相机" << std::endl;
        return -1;
    }

    Mat cameraMatrix = Mat_<double>(3, 3);
    //string camera_in_params_path = "./assets/calibration_in_params.yml";
    FileStorage fs(camera_in_params_path, FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    // 比如 焦距 f=35mm  dx * dy每个像素的物理尺寸为单位为mm,dx像素宽度，dy像素高度分别表示每个像素在水平u和竖直v方向上的实际物理尺寸（单位mm），即每个感光芯片的实际大小。
    //最高分辨率6000×4000，传感器尺寸22.3×14.9mm ; 焦距f=35mm (光心到图像平面的距离)
    // dx单个像素宽度=整个传感器尺寸宽度/分辨率宽
    // dy单个像素高度=整个传感器尺寸高度/分辨率高
    // fx:x轴方向等价的像素个数 ; fx = f(焦距) / dx(单个像素宽度 像素宽度像素的物理尺寸) = 35(焦距) / (22.3(整个传感器尺寸宽度)/6000(分辨率宽)) = 9417.040358744
    // fy:y轴方向等价的像素个数 ; fy = f(焦距) / dy(单个像素高度 像素高度像素的物理尺寸) = 35(焦距) / (14.9(整个传感器尺寸高度)/4000(分辨率高)) = 9395.973154362
    double fx = cameraMatrix.at<double>(0, 0);// x轴方向等价的像素个数; fx =焦距f光心到图像平面的距离) / (dx像素宽度像素的物理尺寸/分辨率宽度6000[像素数量])
    double fy = cameraMatrix.at<double>(1, 1);// y轴方向等价的像素个数; fy =焦距f光心到图像平面的距离) / (dy像素高度像素的物理尺寸/分辨率高度4000[像素数量])
    // 相机像素坐标系中心 相机佳能80D及镜头参数如下：最高分辨率6000×4000
    //宽: 6000 相机像素坐标系中心cx=u0= 6000 / 2 = 3000
    //高: 4000 相机像素坐标系中心cy=v0= 4000 / 2 = 2000
    double cx = cameraMatrix.at<double>(0, 2);//相机像素坐标系中心cx=u0
    double cy = cameraMatrix.at<double>(1, 2);//相机像素坐标系中心cy=v0

    prepareMake3D(cx, cy, fx, fy);

    // 初始化PCL可视化类
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3dViewer"));
    viewer->setBackgroundColor(0, 0, 0);
    PointCloud::Ptr cloud(new PointCloud());

    viewer->addPointCloud<PointT>(cloud, "sample_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_cloud");

    int index = 0;
    while (true) {
        Mat colorMat, depthMat;
        camera->capture(colorMat, depthMat);

        int action = waitKey(30) & 0xFF;
        if (action == 'q' || action == 27) {
            break;
        } else if (action == ACTION_SPACE) {
            std::cout << "开始保存数据" << std::endl;
            // 保存彩图，深度图
            stringstream color_ss;
            color_ss << output_dir << "/camera_color_" << index << ".jpg";
            bool rst = imwrite(color_ss.str(), colorMat);
            if (!rst) {
                std::cerr << "目标目录不存在，或不可写入" << output_dir << std::endl;
                break;
            }

            stringstream depth_ss;
            depth_ss << output_dir << "/camera_depth_" << index << ".xml";
            FileStorage fs(depth_ss.str(), FileStorage::WRITE);
            fs << "depth" << depthMat;
            fs.release();

            // 生成点云
            stringstream cloud_ss;
            cloud_ss << output_dir << "/table_scene_" << index << ".pcd";
            pcl::io::savePCDFile(cloud_ss.str(), *cloud, true);
            index ++;

            std::cout << "生成彩色图，深度图，点云图成功！" << std::endl;
            bitwise_not(colorMat, colorMat);
        }

        if (!viewer->wasStopped()) {
            // 更新点云
            updateCloud(colorMat, depthMat, cloud);

            viewer->updatePointCloud(cloud, "sample_cloud");

            viewer->spinOnce();
        }

        resize(colorMat, colorMat, Size(), 0.5, 0.5);
        resize(depthMat, depthMat, Size(), 0.5, 0.5);
        imshow("rgb", colorMat);
        imshow("depth", depthMat / 4500);
    }

    destroyAllWindows();
    camera->release();

}