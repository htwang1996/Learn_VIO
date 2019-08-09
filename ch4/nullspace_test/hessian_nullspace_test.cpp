//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <Eigen/Eigenvalues>
#include <iomanip>

using namespace std;
// 构建一个结构体， 里面包含R（世界到相机）,Q,T
struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;
};



int main()
{
    int featureNums = 2;  // 20个特征点te
    int poseNums = 2;      // 10个位姿
    int diem = poseNums * 6 + featureNums * 3;  // pose包含方向位置6自由度，feature只有位置3自由度
    double fx = 1.;         // 这里的fx，fy表征的是缩放比例
    double fy = 1.;
    Eigen::MatrixXd H(diem,diem);  //这个就是最终要拼起来的H矩阵
    H.setZero();

    // 定义相机的位置
    std::vector<Pose> camera_pose;
    double radius = 8;
    for(int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / ( poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // 随机数生成三维特征点
    std::default_random_engine generator;
    std::vector<Eigen::Vector3d> points;
    for(int j = 0; j < featureNums; ++j)
    {
        std::uniform_real_distribution<double> xy_rand(-4, 4.0);
        std::uniform_real_distribution<double> z_rand(8., 10.);
        double tx = xy_rand(generator);
        double ty = xy_rand(generator);
        double tz = z_rand(generator);

        Eigen::Vector3d Pw(tx, ty, tz);
        points.push_back(Pw);

        // 对于每次生成的特征点j，执行下述操作
        for (int i = 0; i < poseNums; ++i) {
            Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
            Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

            // 这里的x,y,z是特征点相对当前相机的pose_i的坐标
            double x = Pc.x();
            double y = Pc.y();
            double z = Pc.z();
            double z_2 = z * z;

            // 这里残差的jacobian求导，一定是跟重投影误差有关系，根据重投影误差来定义的。
            // 都是链是求导得来的 详情请见《十四讲》的7.7.3
            // 精彩，这就是实现版本的十四讲
            Eigen::Matrix<double,2,3> jacobian_uv_Pc;
            jacobian_uv_Pc<< fx/z, 0 , -x * fx/z_2,
                    0, fy/z, -y * fy/z_2;

            Eigen::Matrix<double,2,3> jacobian_Pj = jacobian_uv_Pc * Rcw;
            Eigen::Matrix<double,2,6> jacobian_Ti;
            jacobian_Ti << -x* y * fx/z_2, (1+ x*x/z_2)*fx, -y/z*fx, fx/z, 0 , -x * fx/z_2,
                            -(1+y*y/z_2)*fy, x*y/z_2 * fy, x/z * fy, 0,fy/z, -y * fy/z_2;


            // H矩阵有四块，我们填充其中两块
            H.block(i*6,i*6,6,6) += jacobian_Ti.transpose() * jacobian_Ti; // 左上 表示跟自己
            H.block(j*3 + 6*poseNums,j*3 + 6*poseNums,3,3) +=jacobian_Pj.transpose() * jacobian_Pj; // 右下
            H.block(i*6,j*3 + 6*poseNums, 6,3) += jacobian_Ti.transpose() * jacobian_Pj;    // 表示观测
            H.block(j*3 + 6*poseNums, i*6 , 3,6) += jacobian_Pj.transpose() * jacobian_Ti;
        }
    }

    // std::cout << setprecision(2)<< H << std::endl;


    // 最后是0我还没理解原因
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << svd.singularValues() <<std::endl;
    return 0;
}
