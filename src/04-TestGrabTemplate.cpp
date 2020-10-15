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

const string templateRtPath = "./output/template_Rt.xml";
const string ex_calib_file_path = "./assets/calibration_ex_params.yml";

Gripper* gripper;

Mat getTemplateMat() {
    Mat template_RMat = Mat_<double>(3, 3);
    Mat template_tMat = Mat_<double>(3, 1);

    FileStorage rtFs(templateRtPath, FileStorage::READ);
    rtFs["rotation"] >> template_RMat;
    rtFs["translation"] >> template_tMat;
    rtFs.release();
    return toHomogeneousMat(template_RMat, template_tMat);
}


Mat getExMat() {
    double Angle = 0, AxisX = 0, AxisY = 0, AxisZ = 0;
    double TranslationX = 0, TranslationY = 0, TranslationZ = 0;

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

void move2template() {

    // 1. 获取模板在相机下的位姿 ①
    Mat templateMat = getTemplateMat();
    // 2. 读取相机的外参       ②
    Mat extMat = getExMat();
    // 3. 创建工具的位姿       ③
    double tool_x = 0, tool_y = 0, tool_z = 220;
    Mat_<double> toolMat = (Mat_<double>(4, 4) <<
            1, 0, 0, tool_x / 1000,
            0, 1, 0, tool_y / 1000,
            0, 0, 1, tool_z / 1000,
            0, 0, 0, 1
    );

    const Mat &toolMatInv = homogeneousInverse(toolMat);

    std::cout << "templateMat: \n" << templateMat << std::endl;
    std::cout << "extMat: \n" << extMat << std::endl;
    std::cout << "toolMatInv: \n" << toolMatInv << std::endl;

    // 4. 计算目标位姿 ④ = ② * ① * inv(③)
    Mat_<double> finalMat = extMat * templateMat * toolMatInv;
    double *pose = convert2pose(finalMat);

    // ----------------------------------------- 定义目标上方10cm的位置
    Mat_<double> toolMatUp;
    toolMat.copyTo(toolMatUp);
    toolMatUp.at<double>(2, 3) += 0.1f; // += 10cm
    const Mat &toolMatUpInv = homogeneousInverse(toolMatUp);
    Mat_<double> finalMatUp = extMat * templateMat * toolMatUpInv;
    //VisionGrab\src\util\Rotation3DUtils.h
    // 将4x4的变换矩阵转成xyzrpy位姿 *pose = new double[6]{t.at<double>(0),t.at<double>(1),t.at<double>(2),rpy[0],rpy[1],rpy[2]};
    // 机械臂运动轨迹
    double *poseUp = convert2pose(finalMatUp);


    cout << "pose x: " << pose[0] << " y: " << pose[1] << " z: " << pose[2];
    cout << " r: "<< pose[3] * RA2DE << " p: " << pose[4] * RA2DE << " y: " << pose[5] * RA2DE << endl;


    cout << "poseUP x: " << poseUp[0] << " y: " << poseUp[1] << " z: " << poseUp[2];
    cout << " r: "<< poseUp[3] * RA2DE << " p: " << poseUp[4] * RA2DE << " y: " << poseUp[5] * RA2DE << endl;

//    if (true) { return; }

    // ----------------------------------------- 定义目标上方10cm的位置
    // 记录当前位置
    aubo_robot_namespace::JointParam jointParam;
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
    usleep(1 * 1000 * 1000);

    // MoveL下降到位
    rst = Robot::getInstance()->moveL(pose, true);
    if (rst != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cerr << "下降目标失败：" << rst << std::endl;
        return;
    }
    std::cout << "下降到目标Ok" << std::endl;

    // 闭合夹爪
    gripper->gripperCatch(500, 200);
    usleep(2 * 1000 * 1000);

    // MoveL回到上方
    rst = Robot::getInstance()->moveL(poseUp, true);
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
 * 物品在特定位置抓取
 */
int main(int argc, char *argv[]) {

    // sudo chmod 777 /dev/ttyUSB0

    gripper = new Gripper("/dev/ttyUSB0");

    char *ip = "192.168.36.21";
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