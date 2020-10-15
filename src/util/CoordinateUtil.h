/**
 * @Author: PoplarTang
 * @CreateTime: 2019-11-28
 * @Description: 
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace cv;
/**
 * 通过 选择点云上的点 转成抓取的位姿 3点或4点都可以
 */
class CoordinateUtil {


public:

    static vector<double> calcRPY(vector<double> vec, vector<double> rVec, Point3d &normalVec) {
        // 1.x方向, 并归一化
        Eigen::Vector3d xVec(vec[0], vec[1], vec[2]);//cb 向量
        xVec = xVec.normalized();//归一化

        // 2.给定参考方向，此方向根据实际情况指定
        Eigen::Vector3d refVec(rVec[0], rVec[1], rVec[2]);//参考向量ca

        // 3.通过叉乘，计算normal向量作为z轴
        Eigen::Vector3d norVec = xVec.cross(refVec);//得到法向量在轴 垂直与cb的向量
        norVec = norVec.normalized();//归一化
        normalVec = Point3d(norVec[0], norVec[1], norVec[2]);

        /// 4.通过叉乘，计算y轴向量
        Eigen::Vector3d yVec = norVec.cross(xVec);//z轴叉乘x轴得到y轴向量
        yVec = yVec.normalized();//归一化

        /// 5.根据三个坐标轴，可以构建一个旋转矩阵，
        /// 输入依次是
        /// x轴向量
        /// y轴向量
        /// z轴向量(法向量)
        //xVec列向量->转至成行向量xVec.transpose()  打印出来
        std::cout << "xVec:\t" << xVec.transpose() << std::endl;
        std::cout << "yVec:\t" << yVec.transpose() << std::endl;
        std::cout << "norVec:\t" << norVec.transpose() << std::endl;
        // 构建出旋转矩阵 xVec[0] xVec[1] xVec[2] = 1 莫是1; 做了归一化了
        Eigen::Matrix3d Rot;
        Rot <<  xVec[0], yVec[0], norVec[0],
                xVec[1], yVec[1], norVec[1],
                xVec[2], yVec[2], norVec[2];

        ///最后，需要注意的是，此处得到的RPY，是绕动坐标系变换的，顺序是z,y,x
        ///而UR驱动中,moveL的输入里，RPY是按x,y,z顺序输入的
        // 旋转矩阵转欧拉角
        // Rot.eulerAngles(2, 1, 0); zyx结果  不同的旋转顺序 先按z旋转后按y旋转在按x旋转
        // Rot.eulerAngles(2, 1, 2); zyz结果  不同的旋转顺序
        // Rot.eulerAngles(0, 1, 0); xyx结果  不同的旋转顺序
        Eigen::Vector3d euler_angles = Rot.eulerAngles(2, 1, 0);
        cout << "RotationMatrix2euler result is:" << endl;
        cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << endl;
        std::vector<double> result{euler_angles[0], euler_angles[1], euler_angles[2]};
//    result = normalize(rpy);
        return result;

    };

/**
欧拉角计算对应的旋转矩阵


**/
    static Mat eulerAnglesToRotationMatrix(Vec3f &theta) {
        // 计算旋转矩阵的X分量
        Mat R_x = (Mat_<double>(3, 3) <<
                                      1, 0, 0,
                0, cos(theta[0]), -sin(theta[0]),
                0, sin(theta[0]), cos(theta[0])
        );

        // 计算旋转矩阵的Y分量
        Mat R_y = (Mat_<double>(3, 3) <<
                                      cos(theta[1]), 0, sin(theta[1]),
                0, 1, 0,
                -sin(theta[1]), 0, cos(theta[1])
        );

        // 计算旋转矩阵的Z分量
        Mat R_z = (Mat_<double>(3, 3) <<
                                      cos(theta[2]), -sin(theta[2]), 0,
                sin(theta[2]), cos(theta[2]), 0,
                0, 0, 1
        );

        // 合并
        Mat R = R_z * R_y * R_x;
        return R;
    }

// 4点方案.  [O中心, C右下, B左下, A左上]
    static void getPoseFrom4Points(const vector<Point3d> &points3d, vector<double> &xyz, vector<double> &rpy) {// [O, C, B, A]
        Point3d center = points3d[0];     // O
        Point3d cVec = points3d[1];       // C
        Point3d cb = points3d[2] - cVec;   // CB
        Point3d ca = points3d[3] - cVec;   // CA
        vector<double> xVec {cb.x, cb.y, cb.z};     // 确定目标坐标系的x轴向量
        vector<double> rVec {ca.x, ca.y, ca.z};    // 选取一个和xy平面内的参考向量

        Point3d normalVec;
        rpy = calcRPY(xVec, rVec, normalVec);  // 根据x轴和参考向量确定一个坐标系
        //normalVec法向量是指长度为1米的向量
        center = center + (normalVec * 0.02);//抓取的中心位置向下平移2厘米,normalVec法向量是指长度为1米的向量 [中心位置向下平移2厘米:不能抓表面的皮吧]
        xyz = vector<double>{center.x, center.y, center.z};

//    vector<double> target{center.x, center.y, center.z, rpy[2], rpy[1], rpy[0]};
        cout << "center: " << center << endl;
    }
// 3点方案   C右，B左，A上
    static void getPoseFrom3Points(const vector<Point3d> &points3d, vector<double> &xyz, vector<double> &rpy) {// [C, B, A]
        Point3d center = (points3d[0] + points3d[1]) / 2;     // O
        Point3d cVec = points3d[0];       // C 点的值
        Point3d cb = points3d[1] - cVec;   // CB 的向量
        Point3d ca = points3d[2] - cVec;   // CA 的向量
        vector<double> xVec{cb.x, cb.y, cb.z};     // 确定目标坐标系的x轴向量
        vector<double> rVec{ca.x, ca.y, ca.z};    // 选取一个和xy平面内的参考向量

        xyz = vector<double>{center.x, center.y, center.z};

        Point3d normalVec;//法向量
        rpy = calcRPY(xVec, rVec, normalVec);  // 根据x轴和参考向量确定一个坐标系
//    vector<double> target{center.x, center.y, center.z, rpy[2], rpy[1], rpy[0]};
        cout << "center: " << center << endl;
    }

    static void drawCoordinate(const vector<Point3d> &points3d,
                               pcl::visualization::PCLVisualizer::Ptr viewer,
                               Mat &output_RMat,
                               Mat &output_tMat)  {// 4. 构建坐标系得到变化矩阵T0，转成RPY进行抓取测试
        printf("vector<Point3d> points3d{");
        for (auto point3d: points3d) {
            printf("Point3d(%f,%f,%f),", point3d.x, point3d.y, point3d.z);
        }
        printf("};\n");

        vector<double> xyz, rpy;

        if (points3d.size() == 3) {
            getPoseFrom3Points(points3d, xyz, rpy);
        } else if (points3d.size() >= 4) {
            getPoseFrom4Points(points3d, xyz, rpy);
        }

        Vec3f euler(rpy[2], rpy[1], rpy[0]);
        //欧拉角转旋转矩阵
        const Mat &matrix = eulerAnglesToRotationMatrix(euler);

        std::cout << "euler2matrix: \n" << matrix << std::endl;

        output_RMat = matrix;
        output_tMat = (Mat_<double>(3, 1)  << xyz[0], xyz[1], xyz[2]);

        // 绘制坐标系中心点
        cv::Vec3d centerPoint{xyz[0], xyz[1], xyz[2]};
        // 根据物体中点的向量，平移旋转后的坐标系
        // x项量
        cv::Vec3d pointX{matrix.at<double>(0, 0), matrix.at<double>(1, 0), matrix.at<double>(2, 0)};
        //y向量
        cv::Vec3d pointY{matrix.at<double>(0, 1), matrix.at<double>(1, 1), matrix.at<double>(2, 1)};
        //z向量
        cv::Vec3d pointZ{matrix.at<double>(0, 2), matrix.at<double>(1, 2), matrix.at<double>(2, 2)};

        pointX += centerPoint;
        pointY += centerPoint;
        pointZ += centerPoint;

        viewer->removeShape("10");
        viewer->removeShape("20");
        viewer->removeShape("30");
        // 绘制x，y，z
        pcl::PointXYZ cPoint(centerPoint[0], centerPoint[1], centerPoint[2]);
        pcl::PointXYZ x_axis(pointX[0], pointX[1], pointX[2]);
        pcl::PointXYZ y_axis(pointY[0], pointY[1], pointY[2]);
        pcl::PointXYZ z_axis(pointZ[0], pointZ[1], pointZ[2]);
        //画坐标系
        viewer->addLine(cPoint, x_axis, 1.0f, 0.0f, 0.0f, to_string(10));
        viewer->addLine(cPoint, y_axis, 0.0f, 1.0f, 0.0f, to_string(20));
        viewer->addLine(cPoint, z_axis, 0.0f, 0.0f, 1.0f, to_string(30));

    }
};