#ifndef ARM_POSE_SOLVER_H_
#define ARM_POSE_SOLVER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

struct ArmParams {
    int l1, l2, l3;
};

struct JointAngles {
    double theta1, theta2, theta3;
};

//三自由度机械臂解算器
class ArmPoseSolver3Dof
{
public:
    ArmPoseSolver3Dof(const int& l1, const int& l2, const int& l3);
    ~ArmPoseSolver3Dof();

    JointAngles pose_calculation(const int& x, const int& y);
    JointAngles inverse_kinematics(const ArmParams& params, double x, double y, double z);
private:
    ArmParams arm_params;
};

ArmPoseSolver3Dof::ArmPoseSolver3Dof(const int& l1, const int& l2, const int& l3)
{
    //机械臂的基座为L1
    arm_params = {l1, l2, l3};
}

ArmPoseSolver3Dof::~ArmPoseSolver3Dof()
{
}

JointAngles ArmPoseSolver3Dof::pose_calculation(const int& x, const int& y)
{
    double head_angle = 0.785;
    double head_angle_comp = M_PI - head_angle;

    double fy = 547.40127;
    double fx = 544.81663;

    int h = 137 - 33;

    double tan_b = (x - 320)/ fx;
    // double angel_rad_b = atan(tan_b);
    double target_location_x = tan_b * (h / sin(head_angle));
    double tan_a = (y - 240)/ fy;
    double angel_rad_a = atan(tan_a);
    double target_location_y = sin(angel_rad_a) * h / sin(head_angle_comp - angel_rad_a);
    double target_location_z = target_location_y / tan(tan_a);

    Eigen::Vector3d v_3d(target_location_x, target_location_y, target_location_z);

    //相机坐标系到头部舵机坐标系的位置变化（正对机器人，x轴往左，y轴往下，z轴往机器人前方）
    Eigen::Vector3d v_a(0, 4, 19);

    //头部舵机坐标系到机器人左手手臂坐标系的位置变化（预备状态手臂平举，手部朝前。正对机器人x轴往机器人前方，y轴往下，z轴往右）
    Eigen::Vector3d v_b(7, -53, -53);

    //相机坐标系到头部舵机坐标系的旋转变化（沿x轴旋转45°）
    Eigen::AngleAxisd rotation_vector_x(- head_angle, Eigen::Vector3d(1,0,0));
    Eigen::Matrix3d rotation_matrix_x = rotation_vector_x.toRotationMatrix();

    //头部舵机坐标系到左手手臂坐标系的旋转变化（沿y轴旋转90°）
    Eigen::AngleAxisd rotation_vector_y(M_PI / 2, Eigen::Vector3d(0,1,0));
    Eigen::Matrix3d rotation_matrix_y = rotation_vector_y.toRotationMatrix();

    Eigen::Vector3d result = rotation_matrix_y * ((rotation_matrix_x * v_3d) + v_a) + v_b;

    // ArmParams params = {28, 60, 85};

    JointAngles angles = inverse_kinematics(arm_params, result.x(), result.y(), result.z());

    std::cout << "6: " << 500 - ((angles.theta3 * 180 / 3.1459) * 1000 / 240) << std::endl;
    std::cout << "7: " << 500 - ((angles.theta2  * 180 / 3.1459  - 90) *  1000 / 240) << std::endl;
    std::cout << "8: " << 350 + ((angles.theta1 * 180 / 3.1459) *  1000 / 240 ) << std::endl;
    return angles;
}

JointAngles ArmPoseSolver3Dof::inverse_kinematics(const ArmParams& params, double x, double y, double z) {
    JointAngles angles;

    angles.theta1 = std::atan2(y, x);

    double r = std::sqrt(x*x + y*y);
    double s = z - params.l1;

    double D = (r*r + s*s - params.l2*params.l2 - params.l3*params.l3) / (2 * params.l2 * params.l3);
    angles.theta3 = std::atan2(-std::sqrt(1 - D*D), D);

    angles.theta2 = std::atan2(s, r) - std::atan2(params.l3 * std::sin(angles.theta3), params.l2 + params.l3 * std::cos(angles.theta3));

    return angles;
}

#endif //ARM_POSE_SOLVER_H_