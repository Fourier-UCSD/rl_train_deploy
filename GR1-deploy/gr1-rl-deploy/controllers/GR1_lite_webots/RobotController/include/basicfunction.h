#ifndef BASICFUNCTION_H
#define BASICFUNCTION_H
#include <Eigen/Dense>
#include <math.h>
#include <sys/time.h>
#include <torch/script.h>
namespace basicfunction {
// rotation matrix to zyx euler
void MatrixToEuler_ZYX(Eigen::Matrix3d R, Eigen::Vector3d &Euler);
void MatrixToEuler_XYZ(Eigen::Matrix3d R, Eigen::Vector3d &Euler);
void Euler_XYZToMatrix(Eigen::Matrix3d &R, Eigen::Vector3d euler_a);
void Euler_ZYXToMatrix(Eigen::Matrix3d &R, Eigen::Vector3d euler_a);
Eigen::Matrix3d VelProjectionMatrix_EulerXYZ(Eigen::Vector3d Euler);
Eigen::Matrix3d RotX(double x);
Eigen::Matrix3d RotY(double y);
Eigen::Matrix3d RotZ(double z);
void euleraddoffset(Eigen::Vector3d &euler);
void eulersuboffset(Eigen::Vector3d &euler);
// skew
Eigen::Matrix3d skew(Eigen::Vector3d r);
void matrixtoeulerxyz_(Eigen::Matrix3d R, Eigen::Vector3d &euler);
/**
 * @brief return interval time unit:s
 *
 * @param ts
 * @param te
 * @return double
 */
double intervaltime(struct timeval ts, struct timeval te);

Eigen::VectorXd gait_phase(double timer,
                           double gait_cycle_,
                           double left_theta_offest_,
                           double right_theta_offest_,
                           double left_phase_ratio_,
                           double right_phase_ratio_,
                           double &left_phase,
                           double &right_phase);
void copytoinputs(torch::Tensor &inputdata, Eigen::VectorXd inputs_nlp);
void copytoinputs(torch::Tensor &inputdata, Eigen::VectorXd inputs_nlp, int num);
void clip(Eigen::VectorXd &in_, double lb, double ub);
double clip(double a, double lb, double ub);
} // namespace basicfunction

#endif // BASICFUNCTION_H
