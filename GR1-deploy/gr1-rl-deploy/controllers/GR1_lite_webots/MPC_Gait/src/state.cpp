#include "../include/state.h"
#include "../include/GaitGenerator.h"
#include <sys/time.h>
// #include "state.h"
// #include "GaitGenerator.h"
// Start /////////////////////////////////////////////
void Start::init() {
    std::cout << "Start::init() " << std::endl;
    stateName = "Start";
    addEventTrans("gotoZero", "Zero");
    addEventTrans("gotoNLP", "NLP");
    // addEventTrans("gotoStand", "Stand");
    // addEventTrans("gotoTestmpc", "Testmpc");
    // addEventTrans("gotoJump", "Jump");
    // addEventTrans("gotoWalk", "Walk");
    // addEventTrans("gotoStep", "Step");
}

void Start::onEnter() {
    std::cout << "Start::onEnter() " << std::endl;
    GaitGenerator *gait = static_cast<GaitGenerator *>(app);
    gait->robot_controller_._robot_data->q_c = gait->robot_controller_._robot_data->q_a;
    gait->robot_controller_._robot_data->q_dot_c = gait->robot_controller_._robot_data->q_dot_a;
    gait->robot_controller_._robot_data->q_ddot_c = gait->robot_controller_._robot_data->q_ddot_a;
    gait->robot_controller_._robot_data->tau_c = gait->robot_controller_._robot_data->tau_a;

    // gait->setevent("gotoNLP");
}

void Start::run() {
    // std::cout  << "Start::run() " <<std::endl;
    GaitGenerator *gait = static_cast<GaitGenerator *>(app);
}

void Start::onExit() {
    std::cout << "Start::onExit" << std::endl;
}

// Zero //////////////////////////////////////////////
void Zero::init() {
    std::cout << "Zero::init() " << std::endl;
    stateName = "Zero";
    addEventTrans("gotoNLP", "NLP");
    GaitGenerator *gait = static_cast<GaitGenerator *>(app);
    timer = 0.0;
    //
    qa = Eigen::MatrixXd::Zero(18, 1);
    qa_dot = Eigen::MatrixXd::Zero(18, 1);
    qd = Eigen::MatrixXd::Zero(18, 1);
    ;
    qd_dot = Eigen::MatrixXd::Zero(18, 1);
    //
    avr_v = 0.1;
    totaltime = 0;

    qCmd = Eigen::VectorXd::Zero(12);
    qDotCmd = Eigen::VectorXd::Zero(12);
    xStand = Eigen::VectorXd::Zero(6);
    //
    qaWaist = Eigen::MatrixXd::Zero(3, 1);
    qa_dotWaist = Eigen::MatrixXd::Zero(3, 1);
    qdWaist = Eigen::MatrixXd::Zero(3, 1);
    qd_dotWaist = Eigen::MatrixXd::Zero(3, 1);
    //
    qaArm = Eigen::MatrixXd::Zero(8, 1);
    qa_dotArm = Eigen::MatrixXd::Zero(8, 1);
    qdArm = Eigen::MatrixXd::Zero(8, 1);
    qd_dotArm = Eigen::MatrixXd::Zero(8, 1);

    return;
}

void Zero::onEnter() {
    std::cout << "Zero::onEnter() " << std::endl;
    GaitGenerator *gait = static_cast<GaitGenerator *>(app);

    timer = 0.0;

    // update
    qa = gait->robot_controller_._robot_data->q_a;
    qa_dot = gait->robot_controller_._robot_data->q_dot_a;
    qaWaist = gait->robot_controller_._robot_data->q_a_Waist;
    qa_dotWaist = gait->robot_controller_._robot_data->q_dot_a_Waist;
    qaArm = gait->robot_controller_._robot_data->q_a_Arm;
    qa_dotArm = gait->robot_controller_._robot_data->q_dot_a_Arm;
    // init position
    xStand(0) = 0.01;
    xStand(1) = 0.12;
    xStand(2) = -0.7;
    xStand(3) = 0.01;
    xStand(4) = -0.12;
    xStand(5) = -0.7;
    footflag = true;
    wkSpace2Joint(xStand, qCmd, qDotCmd, footflag);
    qd.setZero();
    // qd.block(6,0,12,1) = qCmd;
    qd.block(6, 0, 12, 1) = gait->default_dof_pos.head(12);
    qd_dot.setZero();

    qdWaist.setZero();
    qd_dotWaist.setZero();

    qdArm = gait->default_dof_pos.tail(8);
    qd_dotArm.setZero();
    // init total time
    avr_v = 0.2;
    double deltq = max(abs((qd - qa).block(6, 0, 12, 1).maxCoeff()), abs((qd - qa).block(6, 0, 12, 1).minCoeff()));
    totaltime = deltq / avr_v;
    std::cout << "qd: " << qd.transpose() << std::endl;
    std::cout << "qa: " << qa.transpose() << std::endl;
    std::cout << "qa_dot: " << qa_dot.transpose() << std::endl;
    std::cout << "zero_tol: " << totaltime << std::endl;

    // data log
    foutData.open("Zero.txt", std::ios::out);
    dataL = Eigen::VectorXd::Zero(100);
    gait->setevent("");
    // start nlp
    gait->inputdata_nlp.setZero();
    gait->nlpmodel->set_inputdata(gait->inputdata_nlp);
}

void Zero::run() {
    // std::cout << "Zero::run() " <<std::endl;
    GaitGenerator *gait = static_cast<GaitGenerator *>(app);
    // std::cout<<"error!"<<std::endl;
    auto robotdata = gait->robot_controller_._robot_data;
    robotdata->time = timer;
    gait->robot_controller_._estimation_operator->task_state_update_x_a(robotdata);
    // run
    Eigen::MatrixXd O = Eigen::MatrixXd::Zero(18, 1);
    double delth = 0.07;
    double T_squad = 2.0;
    double omega = 2.0 * M_PI / T_squad;
    double n = 30;
    Eigen::VectorXd FR = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd FL = Eigen::VectorXd::Zero(6);
    if (timer < totaltime) {
        gait->FifthPoly(qa, qa_dot, O, qd, qd_dot, O, totaltime, timer,
                        robotdata->q_c,
                        robotdata->q_dot_c,
                        robotdata->q_ddot_c);
        gait->FifthPoly(qaWaist, qa_dotWaist, O, qdWaist, qd_dotWaist, O, totaltime, timer, robotdata->q_waist_c, robotdata->q_dot_c_Waist,
                        robotdata->q_ddot_c_Waist);
        gait->FifthPoly(qaArm, qa_dotArm, O, qdArm, qd_dotArm, O, totaltime, timer, robotdata->q_c_Arm, robotdata->q_dot_c_Arm,
                        robotdata->q_ddot_c_Arm);
        // std::cout<<"timer: "<< timer<<std::endl;
        // std::cout<<"qa: "<< qa.transpose()<<std::endl;
        // std::cout<<"qd: "<< qd.transpose()<<std::endl;
        // std::cout<<"qc: "<< robotdata->q_c.transpose()<<std::endl;
        robotdata->tau_c.setZero();
        robotdata->tau_c_Waist.setZero();
        robotdata->tau_c_Arm.setZero();
    } else {
        // std::cout<<"end!"<<std::endl;
        robotdata->q_c.block(6, 0, 12, 1) = qd.block(6, 0, 12, 1);
        robotdata->q_dot_c.block(6, 0, 12, 1).setZero();
        robotdata->tau_c.setZero();
        robotdata->q_waist_c = qdWaist;
        robotdata->q_dot_c_Waist.setZero();
        robotdata->tau_c_Waist.setZero();
        robotdata->q_c_Arm = qdArm;
        robotdata->q_dot_c_Arm.setZero();
        robotdata->tau_c_Arm.setZero();
    }

    // PD gains
    robotdata->q_factor = Eigen::VectorXd::Ones(robotdata->ndof - 6, 1);
    robotdata->q_dot_factor = Eigen::VectorXd::Ones(robotdata->ndof - 6, 1);
    robotdata->q_factor_Waist = Eigen::VectorXd::Ones(3, 1);
    robotdata->q_dot_factor_Waist = Eigen::VectorXd::Ones(3, 1);
    robotdata->q_factor_Arm = 1. * Eigen::VectorXd::Ones(8, 1);
    robotdata->q_dot_factor_Arm = 1. * Eigen::VectorXd::Ones(8, 1);

    timer += robotdata->dt;
    // std::cout<<"error!"<<std::endl;
    // updata event
    if (timer < totaltime) {
        gait->setevent("");
        // std::cout<<"totaltime: "<<totaltime<<std::endl;
    } else {
    }
    // log data
    dataL[0] = timer;
    dataL.block(1, 0, 18, 1) = robotdata->q_a;
    dataL.block(1, 0, 18, 1) = robotdata->q_c;
    dataL.block(19, 0, 18, 1) = robotdata->q_dot_c;
    dataL.block(37, 0, 18, 1) = robotdata->tau_a;
    dataL.block(55, 0, 12, 1) = robotdata->grf;
    dataL(67) = robotdata->stance_index;
    dataL.block(68, 0, 6, 1) = robotdata->temp_kalman;
    dataL.block(74, 0, 2, 1) = robotdata->st_rl;
    dataL.block(76, 0, 2, 1) = robotdata->tsw_rl;
    dataLog(dataL, foutData);
    // std::cout<<"error?"<<std::endl;
}

void Zero::onExit() {
    std::cout << "Zero::onExit()" << std::endl;
    GaitGenerator *gait = static_cast<GaitGenerator *>(app);
    foutData.close();
}

// NLP //////////////////////////////////////////////////////////////
void NLP::init() {
    std::cout << "NLP::init() " << std::endl;

    stateName = "NLP";
    addEventTrans("gotoZero", "Zero");
    timer = 0.0;
    count = 0;
    // init
    q_init = Eigen::VectorXd::Zero(18);
    //
    q_c = Eigen::VectorXd::Zero(18);
    q_dot_c = Eigen::VectorXd::Zero(18);
    q_ddot_c = Eigen::VectorXd::Zero(18);
    tau_c = Eigen::VectorXd::Zero(18);
    omega_filter = new LowPassFilter(30, 0.707, 0.0025, 3);
    omega_filter_lin_vel = new LowPassFilter(10, 0.707, 0.01, 3);
    omega_filter_dof_vel = new LowPassFilter(10, 0.707, 0.01, 23);
    return;
}

void NLP::onEnter() {
    std::cout << std::endl
              << "NLP::onEnter()" << std::endl;
    GaitGenerator *gait = static_cast<GaitGenerator *>(app);
    auto robotdata = gait->robot_controller_._robot_data;
    timer = 0.0;
    count = 0;
    robotdata->tsw_rl.setZero();
    gait->robot_controller_._estimation_operator->task_state_update_x_a_walk(robotdata);
    //-----------------------------------------//
    q_c = Eigen::VectorXd::Zero(18);
    q_dot_c = Eigen::VectorXd::Zero(18);
    q_ddot_c = Eigen::VectorXd::Zero(18);
    tau_c = Eigen::VectorXd::Zero(18);
    q_init = robotdata->q_a;
    robotdata->nlp_init = true;
    inputdata_nlp_tmp = Eigen::VectorXd::Zero(91);

    // inputdata
    // run
    gait->inputdata_nlp.setZero();
    // data log
    foutData.open("NLP.txt", std::ios::out);
    dataL = Eigen::VectorXd::Zero(300);

    gait->setevent("");
    gait->action_last.setZero();
}

void NLP::run() {
    // std::cout << "NLP::run()" << std::endl;
    GaitGenerator *gait = static_cast<GaitGenerator *>(app);
    auto robotdata = gait->robot_controller_._robot_data;

    // run
    // update task actual state
    robotdata->time = timer;
    gait->robot_controller_._estimation_operator->task_state_update_x_a_walk(robotdata);

    // q_c = q_init;
    // jotstick
    gait->command = robotdata->vCmd_joystick;

    Eigen::Matrix3d Rb_w = Eigen::Matrix3d::Identity();
    basicfunction::Euler_XYZToMatrix(Rb_w, robotdata->q_a.segment(3, 3));
    // Eigen::VectorXd linvel_noise = Eigen::VectorXd::Random(1)*0.01*gait->obs_scales_lin_vel;
    // self.base_lin_vel
    gait->inputdata_nlp.segment(0, 3) = Rb_w.transpose() * robotdata->q_dot_a.segment(0, 3) * gait->obs_scales_lin_vel;
    // gait->inputdata_nlp.segment(0,3) = omega_filter_lin_vel->mFilter(gait->inputdata_nlp.segment(0,3));
    // gait->inputdata_nlp(1) = gait->inputdata_nlp(1) + linvel_noise(0);
    Eigen::Vector3d rpy = robotdata->q_a.segment(3, 3);
    Eigen::Matrix3d R_xyz_omega = Eigen::Matrix3d::Identity();
    R_xyz_omega.row(1) = basicfunction::RotX(rpy(0)).row(1);
    R_xyz_omega.row(2) = (basicfunction::RotX(rpy(0)) * basicfunction::RotY(rpy(1))).row(2);
    R_xyz_omega.row(3) = (basicfunction::RotX(rpy(0)) * basicfunction::RotY(rpy(1)) * basicfunction::RotZ(rpy(2))).row(3);
    Eigen::VectorXd angvel_noise = Eigen::VectorXd::Random(3) * 0.0 * gait->obs_scales_ang_vel;
    // self.base_ang_vel
    gait->inputdata_nlp.segment(3, 3) = Rb_w.transpose() * R_xyz_omega * robotdata->q_dot_a.segment(3, 3) * gait->obs_scales_ang_vel + angvel_noise;
    // gait->inputdata_nlp.segment(3, 3) = omega_filter->mFilter(gait->inputdata_nlp.segment(3, 3));
    // self.projected_gravity
    gait->inputdata_nlp.segment(6, 3) = -Rb_w.transpose().col(2);
    // self.commands
    gait->inputdata_nlp(9) = gait->command[0] * gait->command_scales[0];
    gait->inputdata_nlp(10) = gait->command[1] * gait->command_scales[1];
    gait->inputdata_nlp(11) = gait->command[2] * gait->command_scales[2];
    // (self.dof_pos - self.default_dof_pos)
    gait->inputdata_nlp.segment(12, 12) = (robotdata->q_a.segment(6, 12) - gait->default_dof_pos.head(12)) * gait->obs_scales_dof_pos;
    // std::cout << "leg: " << robotdata->q_a.segment(6, 12) << std::endl;

    gait->inputdata_nlp.segment(24, 3) = (robotdata->q_a_Waist - gait->default_dof_pos.segment(12, 3)) * gait->obs_scales_dof_pos;

    gait->inputdata_nlp.segment(27, 8) = (robotdata->q_a_Arm - gait->default_dof_pos.segment(15, 8)) * gait->obs_scales_dof_pos;
    // std::cout << "arm: " << robotdata->q_a_Arm << std::endl;

    // self.dof_vel
    gait->inputdata_nlp.segment(35, 12) = robotdata->q_dot_a.segment(6, 12) * gait->obs_scales_dof_vel;
    gait->inputdata_nlp.segment(47, 3) = robotdata->q_dot_a_Waist * gait->obs_scales_dof_vel;

    gait->inputdata_nlp.segment(50, 8) = robotdata->q_dot_a_Arm * gait->obs_scales_dof_vel;
    gait->inputdata_nlp.segment(35, 23) = omega_filter_dof_vel->mFilter(gait->inputdata_nlp.segment(35, 23));
    basicfunction::clip(gait->action_last, -100.0, 100.0);
    // self.actions
    gait->inputdata_nlp.segment(58, 23) = gait->action_last;

    // gait->left_theta_offest = timer / gait->gait_cycle + gait->left_theta_offest;
    // gait->right_theta_offest = timer / gait->gait_cycle + gait->right_theta_offest;
    // torch.sin(2 * torch.pi * self.gait_phase),
    //                 torch.cos(2 * torch.pi * self.gait_phase),
    //                 self.phase_ratio,
    gait->inputdata_nlp.segment(81, 6) = basicfunction::gait_phase(timer, gait->gait_cycle,
                                                                   gait->left_theta_offest,
                                                                   gait->right_theta_offest,
                                                                   gait->left_phase_ratio,
                                                                   gait->right_phase_ratio,
                                                                   gait->left_phase,
                                                                   gait->right_phase);
    // std::cout << "left_phase: " << gait->left_phase << std::endl;
    // std::cout << "right_phase: " << gait->right_phase << std::endl;
    // base_height
    gait->inputdata_nlp(87) = basicfunction::clip((robotdata->q_a(2) - 0.85), -1.0, 1.0) * gait->height_scales;
    // gait->inputdata_nlp(79) = (2.0*robotdata->dt/gait->gait_cycle)*gait->inputdata_nlp(0)
    //                         + (1.00 - 2.0*robotdata->dt/gait->gait_cycle)*gait->inputdata_nlp(79);
    // gait->inputdata_nlp(80) = (1.0*robotdata->dt/gait->gait_cycle)*gait->inputdata_nlp(1)
    //                         + (1.00 - 1.0*robotdata->dt/gait->gait_cycle)*gait->inputdata_nlp(80);
    // gait->inputdata_nlp(81) = (1.0*robotdata->dt/gait->gait_cycle)*gait->inputdata_nlp(5)
    //                         + (1.00 - 1.0*robotdata->dt/gait->gait_cycle)*gait->inputdata_nlp(81);

    // gait->inputdata_nlp_encoder.segment(0, 1150) = gait->history_pos;
    // gait->inputdata_nlp_encoder.segment(1150, 1150) = gait->history_vel;
    // gait->inputdata_nlp_encoder.segment(2300, 1150) = gait->history_action;
    // gait->inputdata_nlp_encoder.segment(3450, 150) = gait->history_gravity;

    // std::cout << "gait->inputdata_nlp.segment(81, 6): " << gait->inputdata_nlp.segment(81, 6) << std::endl;

    if (gait->w2s == 1) {
        if (abs(gait->command[0]) == 0 && gait->left_phase == 0.85 && gait->right_phase == 0.35) {

            gait->left_phase_ratio = 0.0;
            gait->right_phase_ratio = 0.0;

        } else {

            gait->left_phase_ratio = 0.35;
            gait->right_phase_ratio = 0.35;
            
        }
    } else {
        if (abs(gait->command[0]) != 0) {
            gait->left_phase_ratio = 0.35;
            gait->right_phase_ratio = 0.35;
        } else {
            gait->left_phase_ratio = 0.0;
            gait->right_phase_ratio = 0.0;
        }
    }

    int fre = 4;
    if (count % fre == 0)

    {
        gait->inputdata_nlp_encoder.segment(0, 3528) = gait->inputdata_nlp_encoder.segment(72, 3528);
        gait->inputdata_nlp_encoder.segment(3528, 23) = gait->inputdata_nlp.segment(12, 23);
        gait->inputdata_nlp_encoder.segment(3551, 23) = gait->inputdata_nlp.segment(35, 23);
        gait->inputdata_nlp_encoder.segment(3574, 23) = gait->action_last;
        gait->inputdata_nlp_encoder.segment(3597, 3) = gait->inputdata_nlp.segment(6, 3);

        // gait->inputdata_nlp_encoder.segment(0, 3675) = gait->inputdata_nlp_encoder.segment(75, 3675);
        // gait->inputdata_nlp_encoder.segment(3675, 23) = gait->inputdata_nlp.segment(12, 23);
        // gait->inputdata_nlp_encoder.segment(3698, 23) = gait->inputdata_nlp.segment(35, 23) / gait->obs_scales_dof_vel;
        // gait->inputdata_nlp_encoder.segment(3721, 23) = gait->action_last;
        // gait->inputdata_nlp_encoder.segment(3744, 3) = gait->inputdata_nlp.segment(6, 3);
        // gait->inputdata_nlp_encoder.segment(3747, 3) = gait->inputdata_nlp.segment(3, 3);

        // gait->inputdata_nlp_encoder.segment(0, 1368) = gait->inputdata_nlp_encoder.segment(72, 1368);
        // gait->inputdata_nlp_encoder.segment(1368, 23) = gait->inputdata_nlp.segment(12, 23);
        // gait->inputdata_nlp_encoder.segment(1391, 23) = gait->inputdata_nlp.segment(35, 23);
        // gait->inputdata_nlp_encoder.segment(1414, 23) = gait->action_last;
        // gait->inputdata_nlp_encoder.segment(1437, 3) = gait->inputdata_nlp.segment(6, 3);

        if (gait->w2s == 1) {
            if (abs(gait->command[0]) == 0 && gait->left_phase == 0.85 && gait->right_phase == 0.35) {
                // gait->outputdata_nlp_encoder = gait->nlpmodel2->act_interface_encoder(gait->inputdata_nlp_encoder);
                // gait->inputdata_nlp(0) = gait->outputdata_nlp_encoder(0);
                // gait->inputdata_nlp(1) = gait->outputdata_nlp_encoder(1);
                // gait->inputdata_nlp(2) = gait->outputdata_nlp_encoder(2);
                // gait->inputdata_nlp(87) = gait->outputdata_nlp_encoder(3);

            }

            else {
                gait->outputdata_nlp_encoder = gait->nlpmodel->act_interface_encoder(gait->inputdata_nlp_encoder);
                gait->inputdata_nlp(0) = gait->outputdata_nlp_encoder(0);
                gait->inputdata_nlp(1) = gait->outputdata_nlp_encoder(1);
                gait->inputdata_nlp(2) = gait->outputdata_nlp_encoder(2);
                gait->inputdata_nlp(87) = gait->outputdata_nlp_encoder(3);
            }
        } else {
            if (abs(gait->command[0]) != 0) {
                gait->outputdata_nlp_encoder = gait->nlpmodel->act_interface_encoder(gait->inputdata_nlp_encoder);
                gait->inputdata_nlp(0) = gait->outputdata_nlp_encoder(0);
                gait->inputdata_nlp(1) = gait->outputdata_nlp_encoder(1);
                gait->inputdata_nlp(2) = gait->outputdata_nlp_encoder(2);
                gait->inputdata_nlp(87) = gait->outputdata_nlp_encoder(3);
            }
        }

        // gait->avg_x_vel.segment(0, 99) = gait->avg_x_vel.segment(1, 99);
        // gait->avg_x_vel(99) = gait->inputdata_nlp(0);
        // gait->avg_y_vel.segment(0, 99) = gait->avg_y_vel.segment(1, 99);
        // gait->avg_y_vel(99) = gait->inputdata_nlp(1);
        // gait->avg_yaw_vel.segment(0, 99) = gait->avg_yaw_vel.segment(1, 99);
        // gait->avg_yaw_vel(99) = gait->inputdata_nlp(5);

        double dt = 1.0 / gait->freqency;
        gait->inputdata_nlp(88) = (2.0 * dt / gait->gait_cycle) * gait->inputdata_nlp(0) + (1.00 - 2.0 * dt / gait->gait_cycle) * gait->inputdata_nlp(88);
        gait->inputdata_nlp(89) = (1.0 * dt / gait->gait_cycle) * gait->inputdata_nlp(1) + (1.00 - 1.0 * dt / gait->gait_cycle) * gait->inputdata_nlp(89);
        gait->inputdata_nlp(90) = (1.0 * dt / gait->gait_cycle) * gait->inputdata_nlp(5) + (1.00 - 1.0 * dt / gait->gait_cycle) * gait->inputdata_nlp(90);

        // gait->inputdata_nlp(88) = gait->avg_x_vel.mean();
        // // std::cout << "gait->avg_x_vel.mean(): " << gait->avg_x_vel.mean() << std::endl;
        // gait->inputdata_nlp(89) = gait->avg_y_vel.mean();
        // gait->inputdata_nlp(90) = gait->avg_yaw_vel.mean();

        // std::cout << "gait->history_pos_obs: " << gait->history_pos_obs.size() << std::endl;
        gait->inputdata_nlp.segment(91, 115) = gait->history_pos_obs;
        gait->inputdata_nlp.segment(206, 115) = gait->history_vel_obs;

        gait->history_pos_obs.segment(0, 92) = gait->history_pos_obs.segment(23, 92);
        gait->history_pos_obs.segment(92, 23) = gait->inputdata_nlp.segment(12, 23);
        gait->history_vel_obs.segment(0, 92) = gait->history_vel_obs.segment(23, 92);
        gait->history_vel_obs.segment(92, 23) = gait->inputdata_nlp.segment(35, 23);

        // std::cout << "gait->inputdata_nlp(88): " << gait->inputdata_nlp(89) << std::endl;

        // gait->inputdata_nlp.segment(0,3) = omega_filter_lin_vel->mFilter(gait->inputdata_nlp.segment(0,3));
        // gait->inputdata_nlp.segment(35, 23) = omega_filter_dof_vel->mFilter(gait->inputdata_nlp.segment(35, 23));
        // gait->inputdata_nlp.segment(0,3) = 0.00*gait->inputdata_nlp.segment(0,3);
        // gait->inputdata_nlp(1) = 0.0*gait->inputdata_nlp(1);
        // std::cout<<"get nlp model command!"<<std::endl;
        // std::cout<<"gait->inputdata_nlp: "<< gait->inputdata_nlp.transpose()<<std::endl;
        if (gait->w2s == 1) {
            if (abs(gait->command[0]) == 0 && gait->left_phase == 0.85 && gait->right_phase == 0.35) {
                gait->nlpmodel2->set_inputdata(gait->inputdata_nlp);
                gait->nlpmodel2->act_interface();
                gait->outputdata_nlp = gait->nlpmodel2->get_outputdata();
                gait->w2s == 0;
            }

            else {
                gait->nlpmodel->set_inputdata(gait->inputdata_nlp);
                gait->nlpmodel->act_interface();
                gait->outputdata_nlp = gait->nlpmodel->get_outputdata();
            }
        } else {

            if (abs(gait->command[0]) != 0) {
                gait->nlpmodel->set_inputdata(gait->inputdata_nlp);
                gait->nlpmodel->act_interface();
                gait->outputdata_nlp = gait->nlpmodel->get_outputdata();
                gait->w2s == 1;
            } else {
                gait->nlpmodel2->set_inputdata(gait->inputdata_nlp);
                gait->nlpmodel2->act_interface();
                gait->outputdata_nlp = gait->nlpmodel2->get_outputdata();
            }
        }
        // gait->nlpmodel->set_inputdata(gait->inputdata_nlp);
        // gait->nlpmodel->act_interface();
        // gait->outputdata_nlp = gait->nlpmodel->get_outputdata();

        // gait->history_pos.segment(0, 1127) = gait->history_pos.segment(23, 1127);
        // gait->history_pos.segment(1127, 23) = gait->inputdata_nlp.segment(12, 23);
        // gait->history_vel.segment(0, 1127) = gait->history_vel.segment(23, 1127);
        // gait->history_vel.segment(1127, 23) = gait->inputdata_nlp.segment(35, 23);
        // gait->history_action.segment(0, 1127) = gait->history_action.segment(23, 1127);
        // gait->history_action.segment(1127, 23) = gait->outputdata_nlp;
        // gait->history_gravity.segment(0, 147) = gait->history_gravity.segment(3, 147);
        // gait->history_gravity.segment(147, 3) = gait->inputdata_nlp.segment(6, 3);

        // std::cout<<"gait->outputdata_nlp: "<<std::endl<<gait->outputdata_nlp.transpose()<<std::endl;
    }
    // gait->outputdata_nlp(14) = 0.0;
    // gait->outputdata_nlp(19) = 0.0;
    Eigen::VectorXd nlpout = (gait->outputdata_nlp * gait->action_scales + gait->default_dof_pos);

    robotdata->q_c.segment(6, 12) = nlpout.head(12);
    robotdata->q_dot_c.setZero();
    robotdata->tau_c.setZero();
    robotdata->q_waist_c = nlpout.segment(12, 3);
    robotdata->q_c_Arm = nlpout.tail(8);
    gait->action_last = gait->outputdata_nlp;
    robotdata->q_factor = Eigen::VectorXd::Ones(robotdata->ndof - 6, 1);
    // std::cout << "aaaa: " << robotdata->ndof << std::endl;
    robotdata->q_dot_factor = Eigen::VectorXd::Ones(robotdata->ndof - 6, 1);
    robotdata->q_factor_Arm = Eigen::VectorXd::Ones(8);
    robotdata->q_factor_Waist = Eigen::VectorXd::Ones(3);
    robotdata->q_dot_factor_Arm = Eigen::VectorXd::Ones(8);
    robotdata->q_dot_factor_Waist = Eigen::VectorXd::Ones(3);

    timer += robotdata->dt;
    count++;
    // updata event

    // std::cout<<"W2S: "<<gait->W2S<<std::endl;
    // std::cout<<"prestand: "<<robotdata->prestand<<std::endl;

    // dataL[0] = timer;
    // dataL.block(1, 0, 18, 1) = robotdata->q_c;
    // dataL.block(19, 0, 8, 1) = robotdata->q_c_Arm;
    // dataL.block(27, 0, 18, 1) = robotdata->q_a;
    // dataL.block(45, 0, 8, 1) = robotdata->q_a_Arm;
    // dataL.block(53, 0, 18, 1) = robotdata->q_dot_a;
    // dataL.block(71, 0, 8, 1) = robotdata->q_dot_a_Arm;
    // dataL.block(79, 0, 18, 1) = robotdata->tau_a;
    // dataL.block(97, 0, 8, 1) = robotdata->tau_a_Arm;
    // dataL.block(105, 0, 12, 1) = robotdata->grf;
    // dataL.block(117, 0, 20, 1) = gait->outputdata_nlp;
    // dataL.block(137, 0, gait->obs_num, 1) = gait->inputdata_nlp;
    // dataL.block(220, 0, 2, 1) = robotdata->st_rl;
    // dataLog(dataL, foutData);
}

void NLP::onExit() {
    std::cout << "NLP::onExit()" << std::endl;
    GaitGenerator *gait = static_cast<GaitGenerator *>(app);
    foutData.close();
}
