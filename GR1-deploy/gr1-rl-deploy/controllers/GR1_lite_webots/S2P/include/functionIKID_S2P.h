#ifndef P_A
#define P_A

#include <Eigen/Dense> // 请确保已安装Eigen库
#include <cmath>
#include <iostream>
#include <vector>

using namespace Eigen;

class functionIKID_S2P {
  public:
    functionIKID_S2P();
    ~functionIKID_S2P();

    Eigen::Matrix3d radian_to_rotation_matrix(double roll, double pitch);
    Eigen::Matrix3d single_radian_rotation_matrix(double pitch);

    bool setEst(const VectorXd &qEst,
                const VectorXd &qDotEst,
                const VectorXd &qTorEst);
    bool calcFK();
    bool calcIK();
    bool getAnkleState(VectorXd &ankleOrienEst,
                       VectorXd &ankleOmegaEst,
                       VectorXd &ankleTorEst);

    bool setDes(const VectorXd &ankleOrienRef,
                const VectorXd &ankleOmegaRef);
    bool setDesTorque(const VectorXd &tauSDes);
    bool calcJointPosRef();
    bool calcJointTorDes();
    bool getDes(VectorXd &qDes, VectorXd &qDotDes, VectorXd &tauDes);

    // 左踝关节
    Eigen::Vector2d left_inverse_position();
    Eigen::Matrix2d left_inverse_Jacobian();
    Eigen::Vector2d left_inverse_velocity();
    Eigen::Vector2d left_inverse_torque();

    Eigen::Vector2d left_forward_position();
    Eigen::Matrix2d left_forward_Jacobian();
    Eigen::Vector2d left_forward_velocity();
    Eigen::Vector2d left_forward_torque();

    Eigen::Vector3d initial_ra1;
    Eigen::Vector3d initial_ra2;
    Eigen::Vector3d initial_rb1;
    Eigen::Vector3d initial_rb2;
    Eigen::Vector3d initial_rc1;
    Eigen::Vector3d initial_rc2;

    double initial_offset;
    double initial_offset_radian;

    double length_bar1;
    double length_bar2;
    double length_rod1;
    double length_rod2;

    double ankle_roll_angle;
    double ankle_pitch_angle;
    double ankle_roll_radian;
    double ankle_pitch_radian;

    double ankle_roll_velocity;
    double ankle_pitch_velocity;
    Eigen::Vector2d total_ankle_velocity;

    double ankle_roll_torque;
    double ankle_pitch_torque;
    Eigen::Vector2d total_ankle_torque;

    Eigen::Vector3d s11;
    Eigen::Vector3d s21;

    double motor_long_angle_fd;
    double motor_short_angle_fd;
    double motor_long_radian_fd;
    double motor_short_radian_fd;
    double total_motor_radian_fd[2];

    double motor_long_velocity_fd;
    double motor_short_velocity_fd;
    Eigen::Vector2d total_motor_velocity_fd;

    double motor_long_torque_fd;
    double motor_short_torque_fd;
    Eigen::Vector2d total_motor_torque_fd;

    Eigen::Vector2d ankle_radian_output;
    Eigen::Vector2d ankle_velocity_output;
    Eigen::Vector2d ankle_torque_output;

    Eigen::Vector2d motor_radian_output;
    Eigen::Vector2d motor_velocity_output;
    Eigen::Vector2d motor_torque_output;

    // 右踝关节
    Eigen::Vector2d right_inverse_position();
    Eigen::Matrix2d right_inverse_Jacobian();
    Eigen::Vector2d right_inverse_velocity();
    Eigen::Vector2d right_inverse_torque();

    Eigen::Vector2d right_forward_position();
    Eigen::Matrix2d right_forward_Jacobian();
    Eigen::Vector2d right_forward_velocity();
    Eigen::Vector2d right_forward_torque();

    Eigen::Vector3d initial_ra1_right;
    Eigen::Vector3d initial_ra2_right;
    Eigen::Vector3d initial_rb1_right;
    Eigen::Vector3d initial_rb2_right;
    Eigen::Vector3d initial_rc1_right;
    Eigen::Vector3d initial_rc2_right;

    double initial_offset_right;
    double initial_offset_radian_right;

    double length_bar1_right;
    double length_bar2_right;
    double length_rod1_right;
    double length_rod2_right;

    double ankle_roll_angle_right;
    double ankle_pitch_angle_right;
    double ankle_roll_radian_right;
    double ankle_pitch_radian_right;

    double ankle_roll_velocity_right;
    double ankle_pitch_velocity_right;
    Eigen::Vector2d total_ankle_velocity_right;

    double ankle_roll_torque_right;
    double ankle_pitch_torque_right;
    Eigen::Vector2d total_ankle_torque_right;

    double motor_short_angle_fd_right;
    double motor_long_angle_fd_right;
    double motor_short_radian_fd_right;
    double motor_long_radian_fd_right;
    double total_motor_radian_fd_right[2];

    double motor_short_velocity_fd_right;
    double motor_long_velocity_fd_right;
    Eigen::Vector2d total_motor_velocity_fd_right;

    double motor_short_torque_fd_right;
    double motor_long_torque_fd_right;
    Eigen::Vector2d total_motor_torque_fd_right;

    Eigen::Vector2d ankle_radian_output_right;
    Eigen::Vector2d ankle_velocity_output_right;
    Eigen::Vector2d ankle_torque_output_right;

    Eigen::Vector2d motor_radian_output_right;
    Eigen::Vector2d motor_velocity_output_right;
    Eigen::Vector2d motor_torque_output_right;
};
#endif
