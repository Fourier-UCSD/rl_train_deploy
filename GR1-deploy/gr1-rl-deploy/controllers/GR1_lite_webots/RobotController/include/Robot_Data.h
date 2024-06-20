#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <vector>
#include "Task.h"
#include "Sensor.h"
const double _INF = 1.0e20;
/**
 * @brief The robot_type enum
 */
enum robot_type
{
    Fixed_Base_Open_Chain = 1,
    Float_Base_Open_Chain,
    Mobile_Wheel_Open_Chain
};
/**
 * @brief The Wbc_Solver_type enum
 */
enum Wbc_Solver_type
{
    hqp_kinematics = 1,
    hqp_dynamics,
    WBIC,
    WQP
};
/**
 * @brief The Robot_Data class
 */
class Robot_Data
{
public:
    // construct function
    Robot_Data();
    ~Robot_Data();
    // robot model
    RigidBodyDynamics::Model *robot_model = nullptr;
    // body id
    std::vector<uint> id_body;
    std::vector<RigidBodyDynamics::Body *> robot_body;
    std::vector<RigidBodyDynamics::Joint *> robot_joint;
    // task card set
    int npriority;
    int ntask;
    std::vector<Task *> task_card_set;
    // sensor set
    std::vector<Sensor *> sensor_set;
    // tau external
    Eigen::VectorXd tau_ext;
    // total dof
    int ndof = 0;
    // wheel dof
    int ndof_wheel = 0;
    double radius_wheel;
    double length_wheel;
    // limits : if ub = lb = 0 -> no bounds
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_lbound;
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_ubound;
    Eigen::Matrix<double, Eigen::Dynamic, 1> qd_bound;
    Eigen::Matrix<double, Eigen::Dynamic, 1> tau_bound;
    // if mobile wheel robot, q = [q_float, q_wheel, other(q_arm...) ]
    // robot current state :
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_a;
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_dot_a;
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_ddot_a;
    Eigen::Matrix<double, Eigen::Dynamic, 1> tau_a;
    // robot desired quantity
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_d;
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_dot_d;
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_ddot_d;
    Eigen::Matrix<double, Eigen::Dynamic, 1> tau_d;
    // robot command quantity
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_c;
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_dot_c;
    Eigen::Matrix<double, Eigen::Dynamic, 1> q_ddot_c;
    Eigen::Matrix<double, Eigen::Dynamic, 1> tau_c;
    // robot system paras
    // control cycle time units: /s
    double dt;
    // robot type
    enum robot_type robottype;
    // solver type
    enum Wbc_Solver_type wbcsolver;
    // desired net contect wrench
    Eigen::Matrix<double, Eigen::Dynamic, 1> net_contact_force;
    // desired contact force : left and right ankle force from Landing_MPC
    Eigen::MatrixXd contactforce;
    // contact state
    Eigen::MatrixXd C;
    //
    int sign;
    // gravity
    Eigen::Matrix<double, 6, 1> G;
    // PD gains
    Eigen::VectorXd q_factor;
    Eigen::VectorXd q_dot_factor;

    // NLP
    bool nlp_init = false;

    // Landing mpc todo
    int body_task_id;
    int com_task_id;
    int left_foot_id;
    int right_foot_id;
    // wbic qp weight
    // Eigen::MatrixXd Q1; // for GRF
    // Eigen::MatrixXd Q2; // for qb
    // data from jason for Q1 and Q2
    double wq1;
    double wq2;
    double wq3;
    double vq1;
    double vq2;
    double vq3;
    double mq1;
    double mq2;
    double mq3;
    double fq1;
    double fq2;
    double fq3;
    Eigen::MatrixXd mpcforceref;
    // imu init matrix
    Eigen::Matrix3d imu_init;
    // WF1 WF2
    Eigen::MatrixXd WF1;
    Eigen::MatrixXd WF2;
    // stance foot index
    int stance_index = 1; // 1: right leg stance; 0: left leg stance
    int touch_index = 1;  // 1: right foot touch; 2: right foot fullly touch down; 3: left foot touch; 4: left foot fully touch down.
    int touch_index_pre = 1;
    int flag_ankle = 0;

    // Updated state estimator variables
    int touch_type = 1;
    int foot_type = 0;

    // ground reaction force
    Eigen::VectorXd grf = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd grf_pre = Eigen::VectorXd::Zero(12);

    double MG{460.0};
    double time{0.0};     // run time (sec) for current behavior
    double t{0.0};        // passed-time (sec), since last Phase switch
    double t_switch{0.0}; // last TD time
    double s{0.0};        // the time-variant parameter: s = clamp(t/Ts, 0, 1)
    double T{0.4};
    double Td{0.4};
    double Td2{0.3};
    double Tc{0.02};
    int step{0};
    double sL;
    double sR;
    double sc;
    double sfL;
    double sfR;
    double sfc;
    double t_ftd{0.5};

    double grf_lb{50.0};
    double grf_ub{0.9 * 460};

    Eigen::VectorXd imuAcc = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd pFoot_td = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd pFootb_td = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd pFootb_tgt = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd pFootb_tgt2 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd pFootb_ini = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd pFootb_end = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd vFoot_td = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd vFootb_td = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd vFootb_tgt = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd vFootb_ini = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd vFootb_end = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd pTorso_td = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd pTorso_tgt = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd pTorso_ini = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd pTorso_end = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd vTorso_td = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd vTorso_tgt = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd vTorso_ini = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd vTorso_end = Eigen::VectorXd::Zero(3);

    Eigen::Matrix3d rTorso_d = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rTorso = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d rTorso_td = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rTorso_tgt = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d rFoot_td = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rFoot_tgt = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d rFoot_l = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rFoot_r = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rFoot_ltd = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rFoot_rtd = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rFoot_d = Eigen::Matrix3d::Identity();

    Eigen::VectorXd tau_lb = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd tau_ub = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd GRF_ub = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd GRF_lb = Eigen::VectorXd::Zero(12);

    //
    Eigen::VectorXd temp = Eigen::VectorXd::Zero(9);
    Eigen::VectorXd temp2 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd qdotcmd_temp = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd temp_kalman = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd temp_worldacc = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd qCmd_td = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd qDotCmd_td = Eigen::VectorXd::Zero(12);

    Eigen::Vector3d vCmd = Eigen::Vector3d::Zero();
    double vy = 0.0;
    double vyaw = 0.0;
    Eigen::Vector3d vCmd_joystick = Eigen::Vector3d::Zero();
    Eigen::Vector3d vCmd_offset_joystick = Eigen::Vector3d::Zero();
    Eigen::Vector3d pCmd_joystick = Eigen::Vector3d::Zero();
    Eigen::Vector3d pCmd_joystick_last = Eigen::Vector3d::Zero();
    Eigen::Vector3d rCmd_joystick = Eigen::Vector3d::Zero();
    Eigen::Vector3d rCmd_joystick_last = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_com = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_com_filt = Eigen::Vector3d::Zero();
    double avg_vx = 0.0;

    Eigen::Vector3d foot_odometer = Eigen::Vector3d::Zero();
    Eigen::Vector3d odometer = Eigen::Vector3d::Zero();
    Eigen::Vector3d odometer_d = Eigen::Vector3d::Zero();
    Eigen::Vector3d odometer_avg = Eigen::Vector3d::Zero();

    // walk to stand flag
    bool prestand = false;
    //
    Eigen::VectorXd xStand_init = Eigen::VectorXd::Zero(6);

    // swing arm phase
    Eigen::Vector2d pArm_tgt = Eigen::Vector2d::Zero();
    Eigen::Vector2d vArm_tgt = Eigen::Vector2d::Zero();
    Eigen::Vector2d pArm_td = Eigen::Vector2d::Zero();
    Eigen::Vector2d vArm_td = Eigen::Vector2d::Zero();

    // rl state machine
    Eigen::Vector2d t_rl = Eigen::Vector2d::Zero();
    Eigen::Vector2d st_rl = Eigen::Vector2d::Ones();
    Eigen::Vector2d st_rl_pre = Eigen::Vector2d::Ones();
    Eigen::Vector2d tsw_rl = Eigen::Vector2d::Zero();
    // waist
    Eigen::VectorXd q_waist_c = Eigen::VectorXd::Zero(3);
    Eigen::Matrix<double, 3, 1> q_a_Waist;
    Eigen::Matrix<double, 3, 1> q_dot_a_Waist;
    Eigen::Matrix<double, 3, 1> tau_a_Waist;
    Eigen::VectorXd q_dot_c_Waist = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd q_ddot_c_Waist = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd tau_c_Waist = Eigen::VectorXd::Zero(3);
    // Arm
    Eigen::Matrix<double, 8, 1> q_a_Arm;
    Eigen::Matrix<double, 8, 1> q_dot_a_Arm;
    Eigen::Matrix<double, 8, 1> tau_a_Arm;
    Eigen::VectorXd q_c_Arm = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd q_dot_c_Arm = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd q_ddot_c_Arm = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd tau_c_Arm = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd q_factor_Waist = Eigen::VectorXd::Ones(3);
    Eigen::VectorXd q_dot_factor_Waist = Eigen::VectorXd::Ones(3);
    Eigen::VectorXd q_factor_Arm = Eigen::VectorXd::Ones(8);
    Eigen::VectorXd q_dot_factor_Arm = Eigen::VectorXd::Ones(8);
};

#endif // ROBOT_DATA_H
