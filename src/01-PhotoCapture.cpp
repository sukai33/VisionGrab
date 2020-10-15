//
// Created by ty on 20-9-30.
//


#include <iostream>
#include <opencv2/opencv.hpp>
#include <KinectCamera.h>
#include <CloudMaker.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;

const int ACTION_SPACE = 32;

//string camera_in_params_path = "./calibration_in_params.yml";
string camera_in_params_path = "./assets/calibration_in_params.yml";
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
    FileStorage fs(camera_in_params_path, FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
//    double fx = cameraMatrix.at<double>(0, 0);
//    double fy = cameraMatrix.at<double>(1, 1);
//    double cx = cameraMatrix.at<double>(0, 2);
//    double cy = cameraMatrix.at<double>(1, 2);

    PointCloud::Ptr cloud(new PointCloud());
    CloudMaker cloudMaker(cameraMatrix);
    while (true) {
        Mat colorMat, depthMat;
        camera->capture(colorMat, depthMat);

        int action = waitKey(30) & 0xFF;
        if (action == 'q' || action == 27) {
            break;
        } else if (action == ACTION_SPACE) {
            // 保存彩图，深度图

            // 生成点云  colorMat:彩色图 depthMat:深度图  cloud:点云
            cloudMaker.convert(colorMat, depthMat, cloud);
            // 保存点云到文件  //默认保存可视化 ASCII的值,true:保存成Binary 二进制 savePCDFileBinary 不可读的,更加高效的读写
            pcl::io::savePCDFile("./output/pcd_cloud.pcd", *cloud, true);

            //pcl::io::savePCDFileASCII("./output/pcd_ascii.pcd", cloud);//保存成 ASCII 可读的
            //pcl::io::savePCDFileBinary("./output/pcd_binary.pcd", cloud);//保存成Binary 二进制 不可读的,更加高效的读写
            //pcl::io::savePCDFileBinaryCompressed("./output/pcd_binary_compressed.pcd", cloud);
            //默认保存可视化 ASCII的值,true:保存成Binary 二进制 不可读的,更加高效的读写
            //pcl::io::savePCDFile("./output/pcd_save.pcd", cloud, true); // savePCDFileBinary


            std::cout << "点云保存成功" << std::endl;
        }
        //缩小原图
        resize(colorMat, colorMat, Size(), 0.5, 0.5);
        resize(depthMat, depthMat, Size(), 0.5, 0.5);
        imshow("rgb", colorMat);
        imshow("depth", depthMat / 4500);
    }

    destroyAllWindows();
    camera->release();

}