//
// Created by ty on 20-9-30.
//

#ifndef VISIONGRAB_CLOUDMAKER_H
#define VISIONGRAB_CLOUDMAKER_H

#include <opencv2/opencv.hpp>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

/**
 * 相机内参
 *
 * 合成点云（彩色图，深度图）
 */
class CloudMaker {

public:
    Mat cameraMatrix;
    explicit CloudMaker(Mat& cameraMatrix) {
        this->cameraMatrix = move(cameraMatrix);
    }

    /**
     * 将彩色和深度图转成点云图
     * @param color
     * @param depth
     * @param cloud
     */
    void convert(Mat &color, Mat &depth, PointCloud::Ptr &cloud){
        // 比如 焦距 f=35mm  dx * dy每个像素的物理尺寸为单位为mm,dx像素宽度，dy像素高度分别表示每个像素在水平u和竖直v方向上的实际物理尺寸（单位mm），即每个感光芯片的实际大小。
        //最高分辨率6000×4000，传感器尺寸22.3×14.9mm ; 焦距f=35mm (光心到图像平面的距离)
        // dx单个像素宽度=整个传感器尺寸宽度/分辨率宽
        // dy单个像素高度=整个传感器尺寸高度/分辨率高
        // fx:x轴方向等价的像素个数 ; fx = f(焦距) / dx(单个像素宽度 像素宽度像素的物理尺寸) = 35(焦距) / (22.3(整个传感器尺寸宽度)/6000(分辨率宽)) = 9417.040358744
        // fy:y轴方向等价的像素个数 ; fy = f(焦距) / dy(单个像素高度 像素高度像素的物理尺寸) = 35(焦距) / (14.9(整个传感器尺寸高度)/4000(分辨率高)) = 9395.973154362
        double fx = cameraMatrix.at<double>(0, 0);// x轴方向等价的像素个数; fx =焦距f光心到图像平面的距离) / (dx像素宽度像素的物理尺寸/分辨率宽度6000[像素数量])
        double fy = cameraMatrix.at<double>(1, 1);// y轴方向等价的像素个数; fy =焦距f光心到图像平面的距离) / (dy像素高度像素的物理尺寸/分辨率高度4000[像素数量])
        // 相机像素坐标系中心 相机佳能80D及镜头参数如下：最高分辨率6000×4000
        // cx图像中心附近的点 分辨率宽x: 6000 相机像素坐标系中心cx=u0= 6000 / 2 = 3000
        double cx = cameraMatrix.at<double>(0, 2);//相机像素坐标系中心cx=u0
        //cx图像中心附近的点 分辨率高y: 4000 相机像素坐标系中心cy=v0= 4000 / 2 = 2000
        double cy = cameraMatrix.at<double>(1, 2);//相机像素坐标系中心cy=v0

        std::cout << "fx: " << fx << std::endl;
        std::cout << "fy: " << fy << std::endl;
        std::cout << "cx: " << cx << std::endl;
        std::cout << "cy: " << cy << std::endl;

        for (int v = 0; v < depth.rows; ++v) {
            for (int u = 0; u < depth.cols; ++u) {
                // 获取深度图中对应 的值单位(MM)   float类型需要与深度相机取得深度值类型 cv_32FC1 一致,  cv_32FC1=4个字节的 float存取(4*8=32)  不能用ushort;数据类型不对会出金字塔点云问题
                float d = depth.ptr<float>(v)[u];
                //强转成单位(米)
                float depth_value = float(d) / 1000.0f;
                 //isnan(depth_value):判断是否是非数字  abs(depth_value):值是否非常小, 都舍弃掉
                if (isnan(depth_value) || abs(depth_value) < 0.0001) {
                    continue;
                }

                // d存在且有意义， 给点云添加一个点 计算好的单位都是(米)
                PointT p;
                p.z = depth_value;
                p.x = (u - cx) * p.z / fx;
                p.y = (v - cy) * p.z / fy;

                // 按照bgr顺序从彩色图中取颜色值; Vec3b:这个类型是不对的用Vec4b
                Vec4b bgr = color.at<Vec4b>(v, u);
                p.b = bgr[0];
                p.g = bgr[1];
                p.r = bgr[2];

                cloud->points.push_back(p);
            }
        }
        // 配置点云参数  无序点云
        cloud->height = 1;
        cloud->width = cloud->points.size();
        // 是否是密集点云
        cloud->is_dense = false;
        std::cout << "cloud.size: " << cloud->points.size() << std::endl;

    };
};

#endif //VISIONGRAB_CLOUDMAKER_H
