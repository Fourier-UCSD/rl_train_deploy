#include "../include/basicfunction.h"
#include <iostream>
// #include "basicfunction.h"
namespace basicfunction {
void MatrixToEuler_ZYX(Eigen::Matrix3d R, Eigen::Vector3d &Euler) {
    Euler = R.eulerAngles(2, 1, 0);
}

void MatrixToEuler_XYZ(Eigen::Matrix3d R, Eigen::Vector3d &Euler) {
    Euler = R.eulerAngles(0, 1, 2);
}

void Euler_XYZToMatrix(Eigen::Matrix3d &R, Eigen::Vector3d euler_a) {
    R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
// add 2022.07.26
void Euler_ZYXToMatrix(Eigen::Matrix3d &R, Eigen::Vector3d euler_a) {
    R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d RotX(double x) {
    return Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).toRotationMatrix();
}
Eigen::Matrix3d RotY(double y) {
    return Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()).toRotationMatrix();
}
Eigen::Matrix3d RotZ(double z) {
    return Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
Eigen::Matrix3d VelProjectionMatrix_EulerXYZ(Eigen::Vector3d Euler) {
    Eigen::Matrix3d R;
    R << 1., 0., sin(Euler[1]),
        0., cos(Euler[0]), -cos(Euler[1]) * sin(Euler[0]),
        0., sin(Euler[0]), cos(Euler[0]) * cos(Euler[1]);

    return R;
}

void euleraddoffset(Eigen::Vector3d &euler) {
    // std::cout<<"euler_a_pre: "<<std::endl<<euler.transpose()<<std::endl;
    Eigen::Matrix3d R_a;
    basicfunction::Euler_XYZToMatrix(R_a, euler);
    Eigen::Vector3d delt(0.785, 0.785, 0.785);
    Eigen::Matrix3d R_delt;
    basicfunction::Euler_XYZToMatrix(R_delt, delt);
    R_a = R_a * R_delt;
    basicfunction::MatrixToEuler_XYZ(R_a, euler);
    // std::cout<<"euler_a_after: "<<std::endl<<euler.transpose()<<std::endl;
}

void eulersuboffset(Eigen::Vector3d &euler) {
    // std::cout<<"euler_a_pre: "<<std::endl<<euler.transpose()<<std::endl;
    Eigen::Matrix3d R_a;
    basicfunction::Euler_XYZToMatrix(R_a, euler);
    // std::cout<<"R_a: "<<std::endl<<R_a<<std::endl;
    Eigen::Vector3d delt(0.785, 0.785, 0.785);
    Eigen::Matrix3d R_delt;
    basicfunction::Euler_XYZToMatrix(R_delt, delt);
    R_a = R_a * R_delt.inverse();
    // std::cout<<"R_a: "<<std::endl<<R_a<<std::endl;
    basicfunction::MatrixToEuler_XYZ(R_a, euler);
    // std::cout<<"euler_a_after: "<<std::endl<<euler.transpose()<<std::endl;
}

Eigen::Matrix3d skew(Eigen::Vector3d r) {
    Eigen::Matrix3d skew;
    skew << 0, -r(2), r(1),
        r(2), 0, -r(0),
        -r(1), r(0), 0;
    return skew;
}

void matrixtoeulerxyz_(Eigen::Matrix3d R, Eigen::Vector3d &euler) {
    euler.setZero();
    // y (-pi/2 pi/2)
    euler(1) = asin(R(0, 2));
    // z [-pi pi]
    double sinz = -R(0, 1) / cos(euler(1));
    double cosz = R(0, 0) / cos(euler(1));
    euler(2) = atan2(sinz, cosz);
    // x [-pi pi]
    double sinx = -R(1, 2) / cos(euler(1));
    double cosx = R(2, 2) / cos(euler(1));
    euler(0) = atan2(sinx, cosx);
}

double intervaltime(struct timeval ts, struct timeval te) {
    return (te.tv_sec - ts.tv_sec) + (te.tv_usec - ts.tv_usec) / 1000000.0;
}

Eigen::VectorXd gait_phase(double timer,
                           double gait_cycle_,
                           double left_theta_offest_,
                           double right_theta_offest_,
                           double left_phase_ratio_,
                           double right_phase_ratio_,
                           double &left_phase,
                           double &right_phase) {
    Eigen::VectorXd res = Eigen::VectorXd::Zero(6);
    left_phase = (timer / gait_cycle_ + left_theta_offest_) - floor(timer / gait_cycle_ + left_theta_offest_);
    right_phase = (timer / gait_cycle_ + right_theta_offest_) - floor(timer / gait_cycle_ + right_theta_offest_);

    res(0) = sin(2.0 * M_PI * left_phase);
    res(1) = sin(2.0 * M_PI * right_phase);
    res(2) = cos(2.0 * M_PI * left_phase);
    res(3) = cos(2.0 * M_PI * right_phase);
    res(4) = left_phase_ratio_;
    res(5) = right_phase_ratio_;
    return res;
}

void copytoinputs(torch::Tensor &inputdata, Eigen::VectorXd inputs_nlp) {
    int num = 175;
    float *input_array = new float[num];
    for (int i = 0; i < num; i++) {
        input_array[i] = inputs_nlp(i);
    }
    inputdata = torch::from_blob(input_array, {num});
}
void copytoinputs(torch::Tensor &inputdata, Eigen::VectorXd inputs_nlp, int num) {
    float *input_array = new float[num];
    for (int i = 0; i < num; i++) {
        input_array[i] = inputs_nlp(i);
    }
    inputdata = torch::from_blob(input_array, {num});
}

void clip(Eigen::VectorXd &in_, double lb, double ub) {
    for (int i = 0; i < in_.size(); i++) {
        if (in_[i] < lb) {
            in_[i] = lb;
        }
        if (in_[i] > ub) {
            in_[i] = ub;
        }
    }
}

double clip(double a, double lb, double ub) {
    double b = a;
    if (a < lb) {
        b = lb;
    }
    if (a > ub) {
        b = ub;
    }
    return b;
}

} // namespace basicfunction
