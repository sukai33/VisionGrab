//
// Created by ty on 20-9-30.
//

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv/cxeigen.hpp>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <Robot.h>
#include <Rotation3DUtils.h>
#include <Gripper.h>

using namespace std;
using namespace cv;
// 模板位姿 机械臂要抓取的位姿  竖抓还是横抓
const string templateRtPath = "./output/template_Rt.xml";
//外参文件 确定相机坐标 机械臂坐标
const string ex_calib_file_path = "./assets/calibration_ex_params.yml";
// 目标盒子所在的位置及姿态
const string template2target_Rt_filepath = "./template2target_Rt.xml";

Gripper* gripper;
//机械臂要抓取的位姿  竖抓还是横抓   1. 获取模板在相机下的位姿 ①  无序分拣第6
Mat getTemplateMat() {
    Mat template_RMat = Mat_<double>(3, 3);
    Mat template_tMat = Mat_<double>(3, 1);
    // ./output/template_Rt.xml 模板位姿 机械臂要抓取的位姿  竖抓还是横抓
    FileStorage rtFs(templateRtPath, FileStorage::READ);
    rtFs["rotation"] >> template_RMat;
    rtFs["translation"] >> template_tMat;
    rtFs.release();
    //VisionGrab\src\util\Rotation3DUtils.h 将旋转矩阵和平移向量转成4x4的齐次变换矩阵 输入: template_RMat:旋转矩阵  template_tMat:平移向量
    return toHomogeneousMat(template_RMat, template_tMat);
}

// 目标盒子所在的位置及姿态 4. 取出模板到目标的位姿 ④   无序分拣第6
Mat getTemplate2TargetMat() {
    Mat template_RMat = Mat_<double>(3, 3);
    Mat template_tMat = Mat_<double>(3, 1);
    // 目标盒子所在的位置及姿态 "./template2target_Rt.xml"
    FileStorage rtFs(template2target_Rt_filepath, FileStorage::READ);
    rtFs["rotation"] >> template_RMat;
    rtFs["translation"] >> template_tMat;
    rtFs.release();
    //VisionGrab\src\util\Rotation3DUtils.h 将旋转矩阵和平移向量转成4x4的齐次变换矩阵 输入: template_RMat:旋转矩阵  template_tMat:平移向量
    return toHomogeneousMat(template_RMat, template_tMat);
}

//②外参文件 确定相机坐标 机械臂坐标  2. 读取相机的外参       ②
//外参文件 确定相机坐标 机械臂坐标  无序分拣第6
//ex_calib_file_path = "./assets/calibration_ex_params.yml";
Mat getExMat() {
    double Angle = 0, AxisX = 0, AxisY = 0, AxisZ = 0;
    double TranslationX = 0, TranslationY = 0, TranslationZ = 0;
    //外参文件 确定相机坐标 机械臂坐标
    FileStorage fs(ex_calib_file_path, FileStorage::READ);

    fs["Angle"] >> Angle;
    fs["AxisX"] >> AxisX;
    fs["AxisY"] >> AxisY;
    fs["AxisZ"] >> AxisZ;
    fs["TranslationX"] >> TranslationX;
    fs["TranslationY"] >> TranslationY;
    fs["TranslationZ"] >> TranslationZ;

    // 将轴角对 -> 旋转矩阵 + 平移矩阵 -> 齐次矩阵
    Eigen::Vector3d axisMatrix(AxisX, AxisY, AxisZ);
    Eigen::AngleAxisd angleAxisd(Angle, axisMatrix);

    const Eigen::AngleAxis<double>::Matrix3 &eigenR = angleAxisd.toRotationMatrix();

    Mat cvR = Mat_<double>(3, 3);
    cv::eigen2cv(eigenR, cvR);

    Mat t = (Mat_<double>(3, 1) << TranslationX / 1000, TranslationY / 1000, TranslationZ / 1000);
    //VisionGrab\src\util\Rotation3DUtils.h 将旋转矩阵和平移向量转成4x4的齐次变换矩阵 输入: template_RMat:旋转矩阵  template_tMat:平移向量
    Mat exMat = toHomogeneousMat(cvR, t);
    return exMat;
}

// ③ 3. 创建工具的位姿
Mat getToolMat() {
    // 3. 创建工具的位姿       ③  4x4的齐次变换矩阵
    double tool_x = 0, tool_y = 0, tool_z = 220;
    Mat_<double> toolMat = (Mat_<double>(4, 4) <<
            1, 0, 0, tool_x / 1000,
            0, 1, 0, tool_y / 1000,
            0, 0, 1, tool_z / 1000,
            0, 0, 0, 1
    );

    return toolMat;

}

//机械臂执行
void move2template() {

    // 1. 获取模板在相机下的位姿 ①  4x4的齐次变换矩阵
    Mat templateMat = getTemplateMat();
    // 2. 读取相机的外参       ②  4x4的齐次变换矩阵
    Mat extMat = getExMat();
    // 3. 创建工具的位姿       ③  4x4的齐次变换矩阵
    double tool_x = 0, tool_y = 0, tool_z = 220;
    Mat_<double> toolMat = (Mat_<double>(4, 4) <<
            1, 0, 0, tool_x / 1000,
            0, 1, 0, tool_y / 1000,
            0, 0, 1, tool_z / 1000,
            0, 0, 0, 1
    );


    // 3. 创建工具的位姿       ③ 4x4的齐次变换矩阵的逆矩阵
    //VisionGrab\src\util\Rotation3DUtils.h homogeneousInverse(toolMat) 求齐次矩阵的逆矩阵
    const Mat &toolMatInv = homogeneousInverse(toolMat);
    // 4. 取出模板到目标的位姿 ④ 4x4的齐次变换矩阵
    Mat template2target = getTemplate2TargetMat();
    // 1. 获取模板在相机下的位姿 ①
    std::cout << "获取模板在相机下的位姿templateMat: \n" << templateMat << std::endl;
    // 2. 读取相机的外参       ②
    std::cout << "读取相机的外参 extMat: \n" << extMat << std::endl;
    // 3. 创建工具的位姿       ③
    std::cout << "创建工具的位姿 toolMatInv: \n" << toolMatInv << std::endl;
    // 4. 取出模板到目标的位姿 ④
    std::cout << "取出模板到目标的位姿 template2target: \n" << template2target << std::endl;

    // 4. 计算目标位姿  = ② * ④ * ① * inv(③)
    // ④ * ①得到真实盒子的位姿
    Mat_<double> finalMat = extMat * template2target * templateMat * toolMatInv;
    //VisionGrab\src\util\Rotation3DUtils.h
    //将4x4的变换矩阵转成xyzrpy位姿 *pose = new double[6]{t.at<double>(0),t.at<double>(1),t.at<double>(2),rpy[0],rpy[1],rpy[2]};
   // 机械臂运动轨迹
    double *pose = convert2pose(finalMat);

    // ----------------------------------------- 定义目标上方10cm的位置
    Mat_<double> toolMatUp;
    //3. 复制工具的位姿 toolMat -> toolMatUp     ③
    toolMat.copyTo(toolMatUp);
    toolMatUp.at<double>(2, 3) += 0.1f; // += 10cm  向上抬起10里米
    //3. 工具的位姿 toolMat -> toolMatUp     ③
    //VisionGrab\src\util\Rotation3DUtils.h homogeneousInverse(toolMat) 求齐次矩阵的逆矩阵
    const Mat &toolMatUpInv = homogeneousInverse(toolMatUp);
    Mat_<double> finalMatUp = extMat * template2target * templateMat * toolMatUpInv;
    //将4x4的变换矩阵转成xyzrpy位姿 *pose = new double[6]{t.at<double>(0),t.at<double>(1),t.at<double>(2),rpy[0],rpy[1],rpy[2]};
    double *poseUp = convert2pose(finalMatUp);


    cout << "pose x: " << pose[0] << " y: " << pose[1] << " z: " << pose[2];
    cout << " r: "<< pose[3] * RA2DE << " p: " << pose[4] * RA2DE << " y: " << pose[5] * RA2DE << endl;


    cout << "poseUP x: " << poseUp[0] << " y: " << poseUp[1] << " z: " << poseUp[2];
    cout << " r: "<< poseUp[3] * RA2DE << " p: " << poseUp[4] * RA2DE << " y: " << poseUp[5] * RA2DE << endl;

//    if (true) { return; }
    // 可以计算抓取姿态z轴和机械臂Z轴负方向(0, 0, -1)的夹角 < 60。

    // ----------------------------------------- 定义目标上方10cm的位置
    // 记录当前位置
    aubo_robot_namespace::JointParam jointParam;
    // 获取当前机械臂的各角度 记录当前位置
    Robot::getInstance()->robotServiceGetJointAngleInfo(jointParam);

    // 打开夹爪
    gripper->gripperRelease();
    usleep(2 * 1000 * 1000);

    // MoveJ目标上方,xyz,rpy
    int rst = Robot::getInstance()->moveJwithPose(poseUp, true);
    if (rst != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "运动到目标上方失败：" << rst << std::endl;
        return;
    }
    std::cout << "运动到上方Ok" << std::endl;
    usleep(5 * 1000 * 1000);

    // MoveL下降到位
    rst = Robot::getInstance()->moveL(pose, true);
    if (rst != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "下降目标失败：" << rst << std::endl;
        return;
    }
    std::cout << "下降到目标Ok" << std::endl;

    // 闭合夹爪
    gripper->gripperCatch(500, 200);
    //休眠2秒
    usleep(2 * 1000 * 1000);

    // MoveL回到上方
    rst = Robot::getInstance()->moveL(poseUp, true);
    // 判断运动是否成功
    if (rst != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "回到上方失败：" << rst << std::endl;
        return;
    }
    std::cout << "回到上方Ok" << std::endl;

    // MoveJ到初始位置
    rst = Robot::getInstance()->moveJ(jointParam.jointPos, true);
    if (rst != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "回到初始位置失败：" << rst << std::endl;
        return;
    }
    std::cout << "回到初始位置Ok" << std::endl;

    usleep(3 * 1000 * 1000);
    // 打开夹爪
    gripper->gripperRelease();

}
/**
 * 尝试根据已有的数据抓取目标
 * 物品在任何位置抓取,自动匹配模板 1. 首先执行05-TemplateAlignment.cpp 得到变化矩阵,而后执行06-GrabTarget.cpp进行抓取
 * 抓取的物品位姿改变需要重新执行 05-TemplateAlignment.cpp 得到变化矩阵,或把05-TemplateAlignment.cpp 集成进这个06-GrabTarget.cpp
 * 类中实现品在任何位置抓取
 *通过 04-TestGrabTemplate.cpp 改版而来
 */
int main(int argc, char *argv[]) {

    // sudo chmod 777 /dev/ttyUSB0

    gripper = new Gripper("/dev/ttyUSB0");

    char *ip = "192.168.36.25";
    int port = 8899;
    int rst = Robot::getInstance()->connect(ip, port);
    if (rst != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "连接机械臂失败， ip: " << ip << std::endl;
        return -1;
    }
    Robot::getInstance()->setOffset(-0.002, 0, 0); // 设置偏移量

    move2template(); // 阻塞式移动

    Robot::getInstance()->disConnect();





}