//
// Created by ty on 20-9-26.
//

#include "KinectCamera.h"


int KinectCamera::init() {


    // 判断已连接的设备个数
    if (freenect2.enumerateDevices() == 0) {
        std::cerr << "未发现Kinect2设备" << std::endl;
        return -1;
    }

    const string &serial = freenect2.getDefaultDeviceSerialNumber();

    if (serial.empty()) {
        std::cerr << "设备序列号不存在" << std::endl;
        return -1;
    }

    std::cout << "设备序列号: " << serial << std::endl;

    // 创建pipeline接收并处理数据
//    auto *pipeline = new libfreenect2::CpuPacketPipeline();
    auto pipeline = new libfreenect2::OpenGLPacketPipeline();

    // 根据序列号开启设备，并设定数据处理通道
    device = freenect2.openDevice(serial, pipeline);

    // 创建数据接收器（监听器），给设备执行监听;
    device->setColorFrameListener(&listener);
    device->setIrAndDepthFrameListener(&listener);

    device->start();

    registration = new libfreenect2::Registration(device->getIrCameraParams(),
                                                  device->getColorCameraParams());
    return 0;
}

void KinectCamera::capture(Mat& outputMat) {

    listener.waitForNewFrame(frames);

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
//    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    Mat colorMat = Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
    // 图像进行镜像处理
    flip(colorMat, colorMat, 1);

    colorMat.copyTo(outputMat);

    listener.release(frames);

}

void KinectCamera::capture(Mat &outputColorMat, Mat &outputDepthMat) {

    listener.waitForNewFrame(frames);

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];   // 1920x1080
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth]; // 512x424

    // ------------------------------- 彩色图 1920x1080
    Mat colorMat = Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
    // 图像进行镜像处理
    flip(colorMat, colorMat, 1);
    colorMat.copyTo(outputColorMat);

    // ------------------------------- 深度图
    // 执行配准，将彩色图映射到深度图上。
    registration->apply(rgb, depth, &undistorted, &registered, true, &bigdepth);
    // 深度图（去畸变）undistorted 512x424
//    Mat undistortedMat = Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
    // 彩色图（对齐后）registered 512x424 有可能有的位置深度未获取到，可能会有黑色空隙
//    Mat registeredMat = Mat(registered.height, registered.width, CV_8UC4, registered.data);
    // 大深度图（对齐后）1920x1082 -> 1920x1080
    Mat bigdepthMat = Mat(bigdepth.height, bigdepth.width, CV_32FC1, bigdepth.data);

    // 左右翻转
    flip(bigdepthMat, bigdepthMat, 1);

    // 把深度图中INF，NAN值设置为0
    for (int row = 0; row < bigdepthMat.rows; ++row) {
        for (int col = 0; col < bigdepthMat.cols; ++col) {
            float d = bigdepthMat.at<float>(row, col);
            if (fpclassify(d) == FP_INFINITE || fpclassify(d) == NAN) {
                bigdepthMat.at<float>(row, col) = 0;
            }
        }
    }
    // 对深度图，去掉首行末行
    Mat bigdepthMat_;
    bigdepthMat_ = bigdepthMat(Rect(0, 1, bigdepthMat.cols, bigdepthMat.rows - 2));
    bigdepthMat_.copyTo(outputDepthMat);

    listener.release(frames);
}


void KinectCamera::release() {
    device->stop();
    device->close();
}

KinectCamera::~KinectCamera() {

}


