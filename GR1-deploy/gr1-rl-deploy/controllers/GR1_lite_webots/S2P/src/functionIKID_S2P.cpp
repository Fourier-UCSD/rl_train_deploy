#include "functionIKID_S2P.h"
#include "myNeuralNetworkFunction.h"
#include "myNeuralNetworkFunctionRight.h"
#include <vector>

using namespace Eigen;

functionIKID_S2P::functionIKID_S2P(){
    //初始化左脚踝
    // 初始化位置参数、ankle偏置角度、连杆长度
    initial_ra1 << 0, 25, 260;
    initial_ra2 << 0, -25, 205;
    initial_rb1 << -53.95, 25, 270.68;
    initial_rb2 << -53.95, -25, 215.68;
    initial_rc1 << -53.95, 25, 10.68;
    initial_rc2 << -53.95, -25, 10.68;

    initial_offset = 11.2;
    initial_offset_radian = initial_offset * M_PI / 180.0;

    length_bar1 = 55;
    length_bar2 = 55;
    length_rod1 = 260;
    length_rod2 = 205;

    // ankle旋转角度
    ankle_roll_angle = 5.0;
    ankle_pitch_angle = 10.0;
    ankle_roll_radian = ankle_roll_angle * M_PI / 180.0;
    ankle_pitch_radian = ankle_pitch_angle * M_PI / 180.0;

    // ankle角速度
    ankle_roll_velocity = 1;
    ankle_pitch_velocity = 0;
    total_ankle_velocity << ankle_roll_velocity, ankle_pitch_velocity;

    // ankle力矩
    ankle_roll_torque = 0;
    ankle_pitch_torque = 10;
    total_ankle_torque << ankle_roll_torque, ankle_pitch_torque;

    // 方向向量
    s11 << 0, 1, 0;
    s21 << 0, 1, 0;

    // 电机增量角度(以初始启动位置为0位)
    motor_long_angle_fd = 60;
    motor_short_angle_fd = 60;
    motor_long_radian_fd = motor_long_angle_fd * M_PI / 180.0;
    motor_short_radian_fd = motor_short_angle_fd * M_PI / 180.0;
    total_motor_radian_fd[0] = motor_long_radian_fd;
    total_motor_radian_fd[1] = motor_short_radian_fd;

    // 电机速度
    motor_long_velocity_fd = 0.0;
    motor_short_velocity_fd = 10.0;
    total_motor_velocity_fd << motor_long_velocity_fd, motor_short_velocity_fd;

    // 电机力矩
    motor_long_torque_fd = 0.0;
    motor_short_torque_fd = 10.0;
    total_motor_torque_fd << motor_long_torque_fd, motor_short_torque_fd;

    //存储输出值
    ankle_radian_output << 0.0, 0.0;
    ankle_velocity_output << 0.0, 0.0;
    ankle_torque_output << 0.0, 0.0;

    motor_radian_output << 0.0, 0.0;
    motor_velocity_output << 0.0, 0.0;
    motor_torque_output << 0.0, 0.0;


    //初始化右脚踝
    // 初始化位置参数、ankle偏置角度、连杆长度
    initial_ra1_right << 0, 25, 205;
    initial_ra2_right << 0, -25, 260;
    initial_rb1_right << -53.95, 25, 215.68;
    initial_rb2_right << -53.95, -25, 270.68;
    initial_rc1_right << -53.95, 25, 10.68;
    initial_rc2_right << -53.95, -25, 10.68;

    initial_offset_right = 11.2;
    initial_offset_radian_right = initial_offset_right * M_PI / 180.0;

    length_bar1_right = 55;
    length_bar2_right = 55;
    length_rod1_right = 205;
    length_rod2_right = 260;

    // ankle旋转角度
    ankle_roll_angle_right = 5.0;
    ankle_pitch_angle_right = 10.0;
    ankle_roll_radian_right = ankle_roll_angle_right * M_PI / 180.0;
    ankle_pitch_radian_right = ankle_pitch_angle_right * M_PI / 180.0;

    // ankle角速度
    ankle_roll_velocity_right = 1;
    ankle_pitch_velocity_right = 0;
    total_ankle_velocity_right << ankle_roll_velocity_right, ankle_pitch_velocity_right;

    // ankle力矩
    ankle_roll_torque_right = 0;
    ankle_pitch_torque_right = 10;
    total_ankle_torque_right << ankle_roll_torque_right, ankle_pitch_torque_right;

    // 电机增量角度(以初始启动位置为0位)
    motor_short_angle_fd_right = 60;
    motor_long_angle_fd_right = 60;
    motor_short_radian_fd_right = motor_short_angle_fd_right * M_PI / 180.0;
    motor_long_radian_fd_right = motor_long_angle_fd_right * M_PI / 180.0;
    total_motor_radian_fd_right[0] = motor_short_radian_fd_right;
    total_motor_radian_fd_right[1] = motor_long_radian_fd_right;

    // 电机速度
    motor_short_velocity_fd_right = 0.0;
    motor_long_velocity_fd_right = 10.0;
    total_motor_velocity_fd_right << motor_short_velocity_fd_right, motor_long_velocity_fd_right;

    // 电机力矩
    motor_short_torque_fd_right = 0.0;
    motor_long_torque_fd_right = 10.0;
    total_motor_torque_fd_right << motor_short_torque_fd_right, motor_long_torque_fd_right;

    //存储输出值
    ankle_radian_output_right << 0.0, 0.0;
    ankle_velocity_output_right << 0.0, 0.0;
    ankle_torque_output_right << 0.0, 0.0;

    motor_radian_output_right << 0.0, 0.0;
    motor_velocity_output_right << 0.0, 0.0;
    motor_torque_output_right << 0.0, 0.0;
}

functionIKID_S2P::~functionIKID_S2P(){}

// 定义旋转函数 radian_to_rotation_matrix
Matrix3d functionIKID_S2P::radian_to_rotation_matrix(double roll, double pitch) {
    Matrix3d rotation_matrix;
    rotation_matrix = AngleAxisd(0, Vector3d::UnitZ())
                    * AngleAxisd(pitch, Vector3d::UnitY())
                    * AngleAxisd(roll, Vector3d::UnitX()); 
    return rotation_matrix;
}

// 定义旋转函数 single_radian_rotation_matrix
Matrix3d functionIKID_S2P::single_radian_rotation_matrix(double pitch) {
    Matrix3d rotation_matrix;
    rotation_matrix = AngleAxisd(pitch, Vector3d::UnitY());
    return rotation_matrix;
}

// 两个脚踝正运动学接口
// 设置电机属性值
bool functionIKID_S2P::setEst(const VectorXd & qEst,
                            const VectorXd & qDotEst,
                            const VectorXd & qTorEst)
{
    motor_long_radian_fd = qEst(0);
    motor_short_radian_fd = qEst(1);
    total_motor_radian_fd[0] = motor_long_radian_fd;
    total_motor_radian_fd[1] = motor_short_radian_fd;
    motor_short_radian_fd_right = qEst(3);
    motor_long_radian_fd_right = qEst(2);
    total_motor_radian_fd_right[0] = motor_short_radian_fd_right;
    total_motor_radian_fd_right[1] = motor_long_radian_fd_right;
    
    total_motor_velocity_fd << qDotEst(0), qDotEst(1);
    total_motor_velocity_fd_right << qDotEst(3), qDotEst(2);

    total_motor_torque_fd << qTorEst(0), qTorEst(1);
    total_motor_torque_fd_right << qTorEst(3), qTorEst(2);

    return 0;

}

bool functionIKID_S2P::calcFK(){
    ankle_radian_output = left_forward_position();
    ankle_radian_output_right = right_forward_position();

    return 0;
}

bool functionIKID_S2P::calcIK(){
    ankle_velocity_output = left_forward_velocity();
    ankle_velocity_output_right = right_forward_velocity();

    ankle_torque_output = left_forward_torque();
    ankle_torque_output_right = right_forward_torque();
    
    return 0;
}

// 输出踝关节状态
bool functionIKID_S2P::getAnkleState(VectorXd & ankleOrienEst,
                                   VectorXd & ankleOmegaEst,
                                   VectorXd & ankleTorEst)
{
    ankleOrienEst << ankle_radian_output(1), ankle_radian_output(0), ankle_radian_output_right(1), ankle_radian_output_right(0);
    ankleOmegaEst << ankle_velocity_output(1), ankle_velocity_output(0), ankle_velocity_output_right(1), ankle_velocity_output_right(0);
    ankleTorEst << ankle_torque_output(1), ankle_torque_output(0), ankle_torque_output_right(1), ankle_torque_output_right(0);
    
    // std::cout << "ankleOrienEst: " << ankleOrienEst << std::endl;
    // std::cout << "ankleOmegaEst: " << ankleOmegaEst << std::endl;
    // std::cout << "ankleTorEst: " << ankleTorEst << std::endl;

    return 0;
}

// 两个脚踝逆运动学接口
// 设置踝关节属性值
bool functionIKID_S2P::setDes(const VectorXd & ankleOrienRef,
                            const VectorXd & ankleOmegaRef)
{
    ankle_roll_radian = ankleOrienRef(1);
    ankle_pitch_radian = ankleOrienRef(0);
    ankle_roll_radian_right = ankleOrienRef(3);
    ankle_pitch_radian_right = ankleOrienRef(2);

    total_ankle_velocity << ankleOmegaRef(1), ankleOmegaRef(0);
    total_ankle_velocity_right << ankleOmegaRef(3), ankleOmegaRef(2);

    return 0;
}

bool functionIKID_S2P::setDesTorque(const VectorXd & tauSDes)
{
    total_ankle_torque << tauSDes(1), tauSDes(0);
    total_ankle_torque_right << tauSDes(3), tauSDes(2);

    return 0;
}

bool functionIKID_S2P::calcJointPosRef()
{
    motor_radian_output = left_inverse_position();
    motor_radian_output_right = right_inverse_position();

    return 0;
}

bool functionIKID_S2P::calcJointTorDes()
{
    motor_velocity_output = left_inverse_velocity();
    motor_velocity_output_right = right_inverse_velocity();

    motor_torque_output = left_inverse_torque();
    motor_torque_output_right = right_inverse_torque();

    return 0;
}

// 输出电机状态
bool functionIKID_S2P::getDes(VectorXd & qDes, VectorXd & qDotDes, VectorXd & tauDes)
{
    qDes << motor_radian_output(0), motor_radian_output(1), motor_radian_output_right(1), motor_radian_output_right(0);
    qDotDes << motor_velocity_output(0), motor_velocity_output(1), motor_velocity_output_right(1), motor_velocity_output_right(0);
    tauDes << motor_torque_output(0), motor_torque_output(1), motor_torque_output_right(1), motor_torque_output_right(0);

    // std::cout << "qDes: " << qDes << std::endl;
    // std::cout << "qDotDes: " << qDotDes << std::endl;
    // std::cout << "tauDes: " << tauDes << std::endl;

    return 0;
}


// 左脚踝正逆运动学
Vector2d functionIKID_S2P::left_inverse_position() {
    // 根据角度得到旋转矩阵
    Matrix3d total_rotation_matrix = radian_to_rotation_matrix(ankle_roll_radian, ankle_pitch_radian);
    // 旋转后的关节点
    Vector3d target_rc1 = total_rotation_matrix * initial_rc1;
    Vector3d target_rc2 = total_rotation_matrix * initial_rc2;
    Vector3d target_ra1 = initial_ra1;
    Vector3d target_ra2 = initial_ra2;

    // 得到计算公式的元素
    double a1 = target_rc1(0) - target_ra1(0);
    double a2 = target_rc2(0) - target_ra2(0);
    double b1 = target_ra1(2) - target_rc1(2);
    double b2 = target_ra2(2) - target_rc2(2);

    // 计算二阶范数
    double norm_1 = (target_rc1 - target_ra1).norm();
    double norm_2 = (target_rc2 - target_ra2).norm();

    double c1 = (length_rod1 * length_rod1 - length_bar1 * length_bar1 - norm_1 * norm_1) / (2 * length_bar1);
    double c2 = (length_rod2 * length_rod2 - length_bar2 * length_bar2 - norm_2 * norm_2) / (2 * length_bar2);

    // 计算电机增量角度值
    double motor_long_radian = asin((b1 * c1 + sqrt(b1 * b1 * c1 * c1 - (a1 * a1 + b1 * b1) * (c1 * c1 - a1 * a1))) /
                                    (a1 * a1 + b1 * b1)) - initial_offset_radian;
    double motor_short_radian = asin((b2 * c2 + sqrt(b2 * b2 * c2 * c2 - (a2 * a2 + b2 * b2) * (c2 * c2 - a2 * a2))) /
                                     (a2 * a2 + b2 * b2)) - initial_offset_radian;
    
    Vector2d vector_motor_radian = {motor_long_radian, motor_short_radian};
    // 打印结果
    // std::cout << "Motor Long Angle (radian): " << motor_long_radian << std::endl;
    // std::cout << "Motor Short Angle (radian): " << motor_short_radian << std::endl;

    return vector_motor_radian;
}

Matrix2d functionIKID_S2P::left_inverse_Jacobian(){
     // 根据角度得到旋转矩阵
    Matrix3d total_rotation_matrix = radian_to_rotation_matrix(ankle_roll_radian, ankle_pitch_radian);

    // 旋转后的关节点
    Vector3d target_rc1 = total_rotation_matrix * initial_rc1;
    Vector3d target_rc2 = total_rotation_matrix * initial_rc2;
    Vector3d target_ra1 = initial_ra1;
    Vector3d target_ra2 = initial_ra2;

    // 得到计算公式的元素
    double a1 = target_rc1(0) - target_ra1(0);
    double a2 = target_rc2(0) - target_ra2(0);
    double b1 = target_ra1(2) - target_rc1(2);
    double b2 = target_ra2(2) - target_rc2(2);

    // 计算二阶范数
    double norm_1 = (target_rc1 - target_ra1).norm();
    double norm_2 = (target_rc2 - target_ra2).norm();

    double c1 = (length_rod1 * length_rod1 - length_bar1 * length_bar1 - norm_1 * norm_1) / (2 * length_bar1);
    double c2 = (length_rod2 * length_rod2 - length_bar2 * length_bar2 - norm_2 * norm_2) / (2 * length_bar2);

    // 计算电机角度值
    double motor_long_radian = asin((b1 * c1 + sqrt(b1 * b1 * c1 * c1 - (a1 * a1 + b1 * b1) * (c1 * c1 - a1 * a1))) /
                                    (a1 * a1 + b1 * b1)) - initial_offset_radian;
    double motor_short_radian = asin((b2 * c2 + sqrt(b2 * b2 * c2 * c2 - (a2 * a2 + b2 * b2) * (c2 * c2 - a2 * a2))) /
                                     (a2 * a2 + b2 * b2)) - initial_offset_radian;

    // 根据角度得到旋转矩阵
    Matrix3d long_rotation_matrix = single_radian_rotation_matrix(motor_long_radian);
    Matrix3d short_rotation_matrix = single_radian_rotation_matrix(motor_short_radian);

    // 踝关节目标位置参数
    Vector3d target_rb1 = target_ra1 + long_rotation_matrix * (initial_rb1 - initial_ra1);
    Vector3d target_rb2 = target_ra2 + short_rotation_matrix * (initial_rb2 - initial_ra2);

    // bar和rod的向量表示
    Vector3d r_bar1 = target_rb1 - target_ra1;
    Vector3d r_bar2 = target_rb2 - target_ra2;
    Vector3d r_rod1 = target_rc1 - target_rb1;
    Vector3d r_rod2 = target_rc2 - target_rb2;

    // 雅可比矩阵的组成部分
    // Jx
    Matrix<double, 2, 6> Jx;
    Jx << r_rod1.transpose(), (target_rc1.cross(r_rod1)).transpose(),
          r_rod2.transpose(), (target_rc2.cross(r_rod2)).transpose();

    // J_theta
    Matrix2d J_theta;
    J_theta << s11.dot(r_bar1.cross(r_rod1)), 0,
               0, s21.dot(r_bar2.cross(r_rod2));

    // G_matrix
    Matrix<double, 6, 2> G_matrix;
    G_matrix << 0, 0,
                0, 0,
                0, 0,
                cos(ankle_pitch_radian), 0,
                0, 1,
                -sin(ankle_pitch_radian), 0;

    //计算雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = J_theta.inverse() * Jx * G_matrix;

    // 打印结果
    // std::cout << "Jacobian Matrix: " << Jacobian_matrix << std::endl;

    return Jacobian_matrix;

}

Vector2d functionIKID_S2P::left_inverse_velocity(){
    //获取雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = left_inverse_Jacobian();

    // 使用雅可比矩阵计算速度关系
    Vector2d motor_velocity = Jacobian_matrix * total_ankle_velocity;
    Vector2d vector_motor_velocity = {motor_velocity(0), motor_velocity(1)};
    
    // 打印结果
    // std::cout << "Motor Long Velocity: " << motor_velocity(0) << std::endl;
    // std::cout << "Motor Short Velocity: " << motor_velocity(1) << std::endl;

    return vector_motor_velocity;
}

Vector2d functionIKID_S2P::left_inverse_torque(){
    //获取雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = left_inverse_Jacobian();

    // 对雅可比矩阵先求逆再求转置，计算力矩关系
    Vector2d motor_torque = Jacobian_matrix.inverse().transpose() * total_ankle_torque;
    Vector2d vector_motor_torque = {motor_torque(0), motor_torque(1)};
    
    // 打印结果
    // std::cout << "Motor Long Torque: " << motor_torque(0) << std::endl;
    // std::cout << "Motor Short Torque: " << motor_torque(1) << std::endl;

    return vector_motor_torque;
}

Vector2d functionIKID_S2P::left_forward_position(){
    // 输入电机角度，通过神经网络计算踝关节角度
    double ankle_radian_fd[2];
    myNeuralNetworkFunction(total_motor_radian_fd, ankle_radian_fd);
    // std::cout << "total_motor_radian_fd[0]: " << total_motor_radian_fd[0] << std::endl;
    // std::cout << "ankle_roll_radian: " << ankle_radian_fd[0] << std::endl;
    // std::cout << "ankle_pitch_radian: " << ankle_radian_fd[1] << std::endl;
    Vector2d vector_ankle_radian_fd = {ankle_radian_fd[0], ankle_radian_fd[1]};

    return vector_ankle_radian_fd;
}

Matrix2d functionIKID_S2P::left_forward_Jacobian(){
     // 从电机角度，得到ankle关节的角度
    Vector2d total_ankle_radian = left_forward_position();
    double ankle_roll_radian_fd = total_ankle_radian(0);
    double ankle_pitch_radian_fd = total_ankle_radian(1);

    // 计算雅可比矩阵
    // 根据角度得到旋转矩阵
    Matrix3d total_rotation_matrix = radian_to_rotation_matrix(ankle_roll_radian_fd, ankle_pitch_radian_fd);

    // 踝关节目标位置参数
    Vector3d target_rc1 = total_rotation_matrix * initial_rc1;
    Vector3d target_rc2 = total_rotation_matrix * initial_rc2;
    Vector3d target_ra1 = initial_ra1;
    Vector3d target_ra2 = initial_ra2;

    // 根据角度得到旋转矩阵
    Matrix3d motor_long_rotation_matrix = single_radian_rotation_matrix(motor_long_radian_fd);
    Matrix3d motor_short_rotation_matrix = single_radian_rotation_matrix(motor_short_radian_fd);

    // 踝关节目标位置参数
    Vector3d target_rb1 = target_ra1 + motor_long_rotation_matrix * (initial_rb1 - initial_ra1);
    Vector3d target_rb2 = target_ra2 + motor_short_rotation_matrix * (initial_rb2 - initial_ra2);

    // bar和rod的向量表示
    Vector3d r_bar1 = target_rb1 - target_ra1;
    Vector3d r_bar2 = target_rb2 - target_ra2;
    Vector3d r_rod1 = target_rc1 - target_rb1;
    Vector3d r_rod2 = target_rc2 - target_rb2;

    // Jx
    Matrix<double, 2, 6> Jx;
    Jx << r_rod1.transpose(), (target_rc1.cross(r_rod1)).transpose(),
          r_rod2.transpose(), (target_rc2.cross(r_rod2)).transpose();

    // J_theta
    Matrix2d J_theta;
    J_theta << s11.dot(r_bar1.cross(r_rod1)), 0,
               0, s21.dot(r_bar2.cross(r_rod2));

    // G_matrix
    Matrix<double, 6, 2> G_matrix;
    G_matrix << 0, 0,
                0, 0,
                0, 0,
                cos(ankle_pitch_radian_fd), 0,
                0, 1,
                -sin(ankle_pitch_radian_fd), 0;

    //计算雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = J_theta.inverse() * Jx * G_matrix;

    // 打印结果
    // std::cout << "Jacobian Matrix: " << Jacobian_matrix << std::endl;

    return Jacobian_matrix;
}

Vector2d functionIKID_S2P::left_forward_velocity(){
    //获取雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = left_forward_Jacobian();

    // 对雅可比矩阵求逆，计算速度关系
    Vector2d ankle_velocity = Jacobian_matrix.inverse() * total_motor_velocity_fd;
    Vector2d vector_ankle_velocity_fd = {ankle_velocity(0), ankle_velocity(1)};
    // 打印结果
    // std::cout << "Ankle Roll Velocity: " << ankle_velocity(0) << std::endl;
    // std::cout << "Ankle Pitch Velocity: " << ankle_velocity(1) << std::endl;

    return vector_ankle_velocity_fd;
}

Vector2d functionIKID_S2P::left_forward_torque(){
    //获取雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = left_forward_Jacobian();

    // 对雅可比矩阵求逆再求转置再求逆，计算速度关系
    Vector2d ankle_torque = Jacobian_matrix.inverse().transpose().inverse() * total_motor_torque_fd;
    Vector2d vector_ankle_torque_fd = {ankle_torque(0), ankle_torque(1)};
    // 打印结果
    // std::cout << "Ankle Roll Torque: " << ankle_torque(0) << std::endl;
    // std::cout << "Ankle Pitch Torque: " << ankle_torque(1) << std::endl;

    return vector_ankle_torque_fd;
}

// 右脚踝正逆运动学
Vector2d functionIKID_S2P::right_inverse_position() {
    // 根据角度得到旋转矩阵
    Matrix3d total_rotation_matrix_right = radian_to_rotation_matrix(ankle_roll_radian_right, ankle_pitch_radian_right);
    // 旋转后的关节点
    Vector3d target_rc1_right = total_rotation_matrix_right * initial_rc1_right;
    Vector3d target_rc2_right = total_rotation_matrix_right * initial_rc2_right;
    Vector3d target_ra1_right = initial_ra1_right;
    Vector3d target_ra2_right = initial_ra2_right;

    // 得到计算公式的元素
    double a1 = target_rc1_right(0) - target_ra1_right(0);
    double a2 = target_rc2_right(0) - target_ra2_right(0);
    double b1 = target_ra1_right(2) - target_rc1_right(2);
    double b2 = target_ra2_right(2) - target_rc2_right(2);

    // 计算二阶范数
    double norm_1 = (target_rc1_right - target_ra1_right).norm();
    double norm_2 = (target_rc2_right - target_ra2_right).norm();

    double c1 = (length_rod1_right * length_rod1_right - length_bar1_right * length_bar1_right - norm_1 * norm_1) / (2 * length_bar1_right);
    double c2 = (length_rod2_right * length_rod2_right - length_bar2_right * length_bar2_right - norm_2 * norm_2) / (2 * length_bar2_right);

    // 计算电机增量角度值
    double motor_short_radian_right = asin((b1 * c1 + sqrt(b1 * b1 * c1 * c1 - (a1 * a1 + b1 * b1) * (c1 * c1 - a1 * a1))) /
                                    (a1 * a1 + b1 * b1)) - initial_offset_radian_right;
    double motor_long_radian_right = asin((b2 * c2 + sqrt(b2 * b2 * c2 * c2 - (a2 * a2 + b2 * b2) * (c2 * c2 - a2 * a2))) /
                                     (a2 * a2 + b2 * b2)) - initial_offset_radian_right;

    Vector2d vector_motor_radian_right = {motor_short_radian_right, motor_long_radian_right};
    
    // 打印结果
    // std::cout << "Motor short Radian (right): " << motor_short_radian_right << std::endl;
    // std::cout << "Motor long Radian (right): " << motor_long_radian_right << std::endl;

    return vector_motor_radian_right;
}

Matrix2d functionIKID_S2P::right_inverse_Jacobian(){
     // 根据角度得到旋转矩阵
    Matrix3d total_rotation_matrix_right = radian_to_rotation_matrix(ankle_roll_radian_right, ankle_pitch_radian_right);

    // 旋转后的关节点
    Vector3d target_rc1_right = total_rotation_matrix_right * initial_rc1_right;
    Vector3d target_rc2_right = total_rotation_matrix_right * initial_rc2_right;
    Vector3d target_ra1_right = initial_ra1_right;
    Vector3d target_ra2_right = initial_ra2_right;

    // 得到计算公式的元素
    double a1 = target_rc1_right(0) - target_ra1_right(0);
    double a2 = target_rc2_right(0) - target_ra2_right(0);
    double b1 = target_ra1_right(2) - target_rc1_right(2);
    double b2 = target_ra2_right(2) - target_rc2_right(2);

    // 计算二阶范数
    double norm_1 = (target_rc1_right - target_ra1_right).norm();
    double norm_2 = (target_rc2_right - target_ra2_right).norm();

    double c1 = (length_rod1_right * length_rod1_right - length_bar1_right * length_bar1_right - norm_1 * norm_1) / (2 * length_bar1_right);
    double c2 = (length_rod2_right * length_rod2_right - length_bar2_right * length_bar2_right - norm_2 * norm_2) / (2 * length_bar2_right);

    // 计算电机角度值
    double motor_short_radian_right = asin((b1 * c1 + sqrt(b1 * b1 * c1 * c1 - (a1 * a1 + b1 * b1) * (c1 * c1 - a1 * a1))) /
                                    (a1 * a1 + b1 * b1)) - initial_offset_radian_right;
    double motor_long_radian_right = asin((b2 * c2 + sqrt(b2 * b2 * c2 * c2 - (a2 * a2 + b2 * b2) * (c2 * c2 - a2 * a2))) /
                                     (a2 * a2 + b2 * b2)) - initial_offset_radian_right;

    // 根据角度得到旋转矩阵
    Matrix3d short_rotation_matrix_right = single_radian_rotation_matrix(motor_short_radian_right);
    Matrix3d long_rotation_matrix_right = single_radian_rotation_matrix(motor_long_radian_right);

    // 踝关节目标位置参数
    Vector3d target_rb1_right = target_ra1_right + short_rotation_matrix_right * (initial_rb1_right - initial_ra1_right);
    Vector3d target_rb2_right = target_ra2_right + long_rotation_matrix_right * (initial_rb2_right - initial_ra2_right);

    // bar和rod的向量表示
    Vector3d r_bar1_right = target_rb1_right - target_ra1_right;
    Vector3d r_bar2_right = target_rb2_right - target_ra2_right;
    Vector3d r_rod1_right = target_rc1_right - target_rb1_right;
    Vector3d r_rod2_right = target_rc2_right - target_rb2_right;

    // 雅可比矩阵的组成部分
    // Jx
    Matrix<double, 2, 6> Jx;
    Jx << r_rod1_right.transpose(), (target_rc1_right.cross(r_rod1_right)).transpose(),
          r_rod2_right.transpose(), (target_rc2_right.cross(r_rod2_right)).transpose();

    // J_theta
    Matrix2d J_theta;
    J_theta << s11.dot(r_bar1_right.cross(r_rod1_right)), 0,
               0, s21.dot(r_bar2_right.cross(r_rod2_right));

    // G_matrix
    Matrix<double, 6, 2> G_matrix;
    G_matrix << 0, 0,
                0, 0,
                0, 0,
                cos(ankle_pitch_radian_right), 0,
                0, 1,
                -sin(ankle_pitch_radian_right), 0;

    //计算雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = J_theta.inverse() * Jx * G_matrix;

    // 打印结果
    // std::cout << "Jacobian Matrix: " << Jacobian_matrix << std::endl;

    return Jacobian_matrix;

}

Vector2d functionIKID_S2P::right_inverse_velocity(){
    //获取雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = right_inverse_Jacobian();

    // 使用雅可比矩阵计算速度关系
    Vector2d motor_velocity_right = Jacobian_matrix * total_ankle_velocity_right;
    Vector2d vector_motor_velocity_right = {motor_velocity_right(0), motor_velocity_right(1)};
    
    // 打印结果
    // std::cout << "Motor short right Velocity: " << motor_velocity_right(0) << std::endl;
    // std::cout << "Motor long right Velocity: " << motor_velocity_right(1) << std::endl;

    return vector_motor_velocity_right;
}

Vector2d functionIKID_S2P::right_inverse_torque(){
    //获取雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = right_inverse_Jacobian();

    // 对雅可比矩阵先求逆再求转置，计算力矩关系
    Vector2d motor_torque_right = Jacobian_matrix.inverse().transpose() * total_ankle_torque_right;
    Vector2d vector_motor_torque_right = {motor_torque_right(0), motor_torque_right(1)};

    // 打印结果
    // std::cout << "Motor short Torque Right: " << motor_torque_right(0) << std::endl;
    // std::cout << "Motor long Torque Right: " << motor_torque_right(1) << std::endl;

    return vector_motor_torque_right;
}

Vector2d functionIKID_S2P::right_forward_position(){
    // 输入电机角度，通过神经网络计算踝关节角度
    double ankle_radian_fd_right[2];
    myNeuralNetworkFunctionRight(total_motor_radian_fd_right, ankle_radian_fd_right);
    // std::cout << "total_motor_radian_fd[0]: " << total_motor_radian_fd[0] << std::endl;
    // std::cout << "ankle_roll_radian: " << ankle_radian_fd_right[0] << std::endl;
    // std::cout << "ankle_pitch_radian: " << ankle_radian_fd_right[1] << std::endl;
    Vector2d vector_ankle_radian_fd_right = {ankle_radian_fd_right[0], ankle_radian_fd_right[1]};

    return vector_ankle_radian_fd_right;
}

Matrix2d functionIKID_S2P::right_forward_Jacobian(){
     // 从电机角度，得到ankle关节的角度
    Vector2d total_ankle_radian_right = right_forward_position();
    double ankle_roll_radian_fd_right = total_ankle_radian_right(0);
    double ankle_pitch_radian_fd_right = total_ankle_radian_right(1);

    // 计算雅可比矩阵
    // 根据角度得到旋转矩阵
    Matrix3d total_rotation_matrix_right = radian_to_rotation_matrix(ankle_roll_radian_fd_right, ankle_pitch_radian_fd_right);

    // 踝关节目标位置参数
    Vector3d target_rc1_right = total_rotation_matrix_right * initial_rc1_right;
    Vector3d target_rc2_right = total_rotation_matrix_right * initial_rc2_right;
    Vector3d target_ra1_right = initial_ra1_right;
    Vector3d target_ra2_right = initial_ra2_right;

    // 根据角度得到旋转矩阵
    Matrix3d motor_short_rotation_matrix_right = single_radian_rotation_matrix(motor_short_radian_fd_right);
    Matrix3d motor_long_rotation_matrix_right = single_radian_rotation_matrix(motor_long_radian_fd_right);

    // 踝关节目标位置参数
    Vector3d target_rb1_right = target_ra1_right + motor_short_rotation_matrix_right * (initial_rb1_right - initial_ra1_right);
    Vector3d target_rb2_right = target_ra2_right + motor_long_rotation_matrix_right * (initial_rb2_right - initial_ra2_right);

    // bar和rod的向量表示
    Vector3d r_bar1_right = target_rb1_right - target_ra1_right;
    Vector3d r_bar2_right = target_rb2_right - target_ra2_right;
    Vector3d r_rod1_right = target_rc1_right - target_rb1_right;
    Vector3d r_rod2_right = target_rc2_right - target_rb2_right;

    // Jx
    Matrix<double, 2, 6> Jx;
    Jx << r_rod1_right.transpose(), (target_rc1_right.cross(r_rod1_right)).transpose(),
          r_rod2_right.transpose(), (target_rc2_right.cross(r_rod2_right)).transpose();

    // J_theta
    Matrix2d J_theta;
    J_theta << s11.dot(r_bar1_right.cross(r_rod1_right)), 0,
               0, s21.dot(r_bar2_right.cross(r_rod2_right));

    // G_matrix
    Matrix<double, 6, 2> G_matrix;
    G_matrix << 0, 0,
                0, 0,
                0, 0,
                cos(ankle_pitch_radian_fd_right), 0,
                0, 1,
                -sin(ankle_pitch_radian_fd_right), 0;

    //计算雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = J_theta.inverse() * Jx * G_matrix;

    // 打印结果
    // std::cout << "Jacobian Matrix: " << Jacobian_matrix << std::endl;

    return Jacobian_matrix;
}

Vector2d functionIKID_S2P::right_forward_velocity(){
    //获取雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = right_forward_Jacobian();

    // 对雅可比矩阵求逆，计算速度关系
    Vector2d ankle_velocity_right = Jacobian_matrix.inverse() * total_motor_velocity_fd_right;
    Vector2d vector_ankle_velocity_fd_right = {ankle_velocity_right(0), ankle_velocity_right(1)};
    // 打印结果
    // std::cout << "Ankle Roll Velocity: " << ankle_velocity_right(0) << std::endl;
    // std::cout << "Ankle Pitch Velocity: " << ankle_velocity_right(1) << std::endl;

    return vector_ankle_velocity_fd_right;
}

Vector2d functionIKID_S2P::right_forward_torque(){
    //获取雅克比矩阵
    Matrix2d Jacobian_matrix;
    Jacobian_matrix = right_forward_Jacobian();

    // 对雅可比矩阵求逆再求转置再求逆，计算速度关系
    Vector2d ankle_torque_right = Jacobian_matrix.inverse().transpose().inverse() * total_motor_torque_fd_right;
    Vector2d vector_ankle_torque_fd_right = {ankle_torque_right(0), ankle_torque_right(1)};
    // 打印结果
    // std::cout << "Ankle Roll Torque: " << ankle_torque_right(0) << std::endl;
    // std::cout << "Ankle Pitch Torque: " << ankle_torque_right(1) << std::endl;

    return vector_ankle_torque_fd_right;
}













