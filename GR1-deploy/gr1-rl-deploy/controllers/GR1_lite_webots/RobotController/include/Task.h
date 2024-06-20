#ifndef TASK_H
#define TASK_H
#include "controller_lib.h"
#include <vector>
 #include <Eigen/StdVector>
/**
 * @brief The task_type enum
 */
enum task_type {general_task = 1, com_task, contact_task, joint_task};
/**
 * @brief The task_direction enum
 */
enum task_direction {task_x_theta=0, task_y_theta, task_z_theta, task_x, task_y, task_z};
/**
 * @brief The Task class
 */
class Task
{
public:
    // construct function
    Task();
    ~Task();
    // task description
    // task priority: 1, 2, ...
    int priority;
    // task ID
    int task_id;
    // task type
    enum task_type type;
    // task dimesion
    int dim;
    // task direction:
    std::vector<task_direction> task_selection_matrix;
    // weight
    Eigen::Matrix<double, Eigen::Dynamic,1> weight;
    // contact state for contact task: false for non contact, true for contact
    bool contact_state_d;
    // task reference frame: 0 for local frame; 1 for world frame
    int frame;

    // task current state:  x, x_dot, x_ddot, F dim:4*n
    /**
     * @brief X_a
     * general_task: x; x_dot; x_ddot; F; 4*dim
     * joint_task: q_a; q_dot; q_ddot; tau;
     * com_task: [theta=0, x]; h; h_dot; F;
     */
    Eigen::MatrixXd X_a;
    // task desired quantity:  x, x_dot, x_ddot , F dim:4*n
    Eigen::MatrixXd X_d;
    // task command quantity:  x, x_dot, x_ddot, F dim:4*n
    Eigen::MatrixXd X_c;
    // task jacobi
    /**
     * @brief jacobi
     * com_task: H
     */
    Eigen::MatrixXd jacobi;
    // task jacobi_dot
    /**
     * @brief jacobi_dot
     * com_task: H_dot
     */
    Eigen::MatrixXd jacobi_dot_q_dot;
//    // task equality constraints
//    Eigen::MatrixXd A_eq;
//    Eigen::VectorXd b_eq;
    // task inequality constraints
    Eigen::MatrixXd A_ineq_x;
    Eigen::VectorXd b_ineq_x;

    Eigen::MatrixXd A_ineq_x_dot;
    Eigen::VectorXd b_ineq_x_dot;

    Eigen::MatrixXd A_ineq_F;
    Eigen::VectorXd b_ineq_F;
    // task controller
    Controller_Lib * controller;
    // sensor id: 0 for none sensor
    int sensor_id;
    // joint id for rbdl
    /**
     * @brief joint_id
     * for com_task, joint_id and T_offset are same as the floating base
     */
    unsigned int joint_id;
    // homogeneous coordinates transformation offset
    Eigen::Matrix4d T_offset;
    /**
     * @brief mass
     * for com_task
     */
//    double mass;
    Eigen::Matrix<double,6,6> IG;

};

#endif // TASK_H
