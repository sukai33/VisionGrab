# 实践课程目标

## 相机标定

制作外参标定图片&机械臂信息采集器

根据采集到的数据进行外参标定

进阶：制作Qt界面版内外参标定工具

高阶：使用之前保存的机械臂位姿，自动重新标定



## 基础代码实现

### 眼在手外

- 内参标定（根据相机） CameraIntrinsicCalibration.cpp
- 相机封装（Kinect2） KinectCamera.h + KinectCamera.cpp
- 内参标定（根据文件） CameraIntrinsicFromFiles.cpp
- 图片+机械臂位姿采集    HandToEyeCapture.cpp
- 图片+机械臂位姿自动采集 HandToEyeCaptureAuto.cpp
- 外参标定（眼在手外） HandToEyeCalibration.cpp
- 外参验证（眼在手外） HandToEyeVerify.cpp

### 眼在手上

- 内参标定（根据文件）CameraIntrinsicFromFiles.cpp
- 相机封装（Astra）  AstraCamera.h + AstraCamera.cpp
- 图片+机械臂位姿采集    HandInEyeCapture.cpp
- 图片+机械臂位姿自动采集 HandInEyeCaptureAuto.cpp
- 外参标定（眼在手上） HandInEyeCalibration.cpp
- 外参验证（眼在手上） HandInEyeVerify.cpp


### 其他

- Aubo机械臂封装
- xml文件操作工具

### OpenCV官方参考

- 内参标定

> opencv-3.4.9/samples/cpp/calibration.cpp
> opencv-3.4.9/build/bin/example_cpp_calibration

- 外参标定

> opencv-3.4.9/modules/calib3d/test/test_calibration_hand_eye.cpp

## 

## 通过模板技术抓取目标

VisionTargetGrab

**基础版：**

1. 制作模板点云
2. 确定抓取位姿

3. 测试抓取模板

4. 获取目标点云

5. 进行模板匹配

6. 执行目标抓取


**进阶版：**

1. 制作Qt界面版模板制作工具
2. 制作Qt界面版加载指定模板，进行目标匹配抓取

**高阶版：**

1. 安全位置判定
2. 准备多个模板，提高模板匹配姿态识别度
3. 将盒子抓取到指定位置放置
4. 不间断抓取多个盒子
5. 设置间隔，实时进行模板匹配
6. 设置目标位置抓取动态延时（跟踪物体）
7. 自动避障

## 控制机械臂完成四棱锥

规划四棱锥的底边和高度（例如底边宽20cm，高60cm）

使用UR模拟器完成机械臂运动

进阶：使用Aubo和UR硬件完成运动

