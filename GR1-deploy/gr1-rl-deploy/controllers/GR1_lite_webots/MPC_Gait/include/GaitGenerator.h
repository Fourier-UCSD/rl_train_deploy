#ifndef GAITGENERATOR_H_
#define GAITGENERATOR_H_
#include "../../NlpModel/include/NlpModel.h"
#include "../../RobotController/include/Robot_Controller.h"
#include "../../RobotController/include/Robot_Data.h"
#include "XFsmMgr.h"
#include "state.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>
#include <torch/script.h>

// #define WEBOTS
#define REALROBOTS

/**
 * @file GaitGenerator.h
 * @author ren xiaoyu
 * @brief generate the move commands,for example: point to point, tracking a desired speed, tracking a path, etc
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 *
 */
class GaitGenerator {
  public:
    GaitGenerator();
    ~GaitGenerator();
    void init(QString path,  // robotdata json
              QString path2, // mpc json
              double _dt,
              DataPackage *data);
    // the FSM running
    void gait_run(DataPackage *data_);
    // set event
    void setevent(string event_);
    void set_current_fsm_command(string current_command);
    // set desired speed
    void setvelocity(double vx, double vy, double vyaw);
    void setvelocity_offset(double vx_offest, double vy_offset);
    // set desired posiition
    void setxyz(double x, double y, double z);
    void setrollpitch(double roll, double pitch);
    // robotarm
    double getleftamp();
    double getleftphase();
    double getrightamp();
    double getrightphase();
    // for test
    // interp fifth
    void FifthPoly(Eigen::VectorXd p0, Eigen::VectorXd p0_dot, Eigen::VectorXd p0_dotdot,     // start point states
                   Eigen::VectorXd p1, Eigen::VectorXd p1_dot, Eigen::VectorXd p1_dotdot,     // end point states
                   double totalTime,                                                          // total permating time
                   double currenttime,                                                        // current time,from 0 to total time
                   Eigen::VectorXd &pd, Eigen::VectorXd &pd_dot, Eigen::VectorXd &pd_dotdot); // output command
    void FifthPoly(double p0, double p0_dot, double p0_dotdot,                                // start point states
                   double p1, double p1_dot, double p1_dotdot,                                // end point states
                   double totalTime,                                                          // total permating time
                   double currenttime,                                                        // current time,from 0 to total time
                   double &pd, double &pd_dot, double &pd_dotdot);                            // output command
    void Thirdpoly(double p0, double p0_dot, double p1, double p1_dot,
                   double totalTime,   // total permating time
                   double currenttime, // current time,from 0 to total time
                   double &pd, double &pd_dot);
    void LinePoly(double p0, double p1,
                  double totalTime,   // total permating time
                  double currenttime, // current time,from 0 to total time
                  double &pd, double &pd_dot);
    // excitingtrajectory
    void ExcitingTrajectoryFunction(Eigen::VectorXd series_para,
                                    Eigen::VectorXd init_pos,
                                    int dof_num,
                                    int series_num,
                                    double base_freq,
                                    double curtime,
                                    Eigen::VectorXd &pos,
                                    Eigen::VectorXd &vel,
                                    Eigen::VectorXd &acc);

    // unit Quaternion interpolation
    Eigen::Matrix3d QuanteiniontoMatrix(RigidBodyDynamics::Math::Quaternion Q);
    void QuaternionInterp(Eigen::Matrix3d R_start,
                          Eigen::Matrix3d R_end,
                          double totaltime, double currenttime,
                          Eigen::Matrix3d &R_d, Eigen::Vector3d &omiga_d, Eigen::Vector3d &acc_d);

    // robot data
    Robot_Controller robot_controller_;
    string fsmstatename;
    // walk2stand
    bool W2S = false;
    string event;
    //
    string current_fsmstate_command;

  private:
    // state machine
    XFsmMgr fsm;
    // string event;

    Start *startstate;
    Zero *zerostate;
    NLP *nlpstate;

    int initFsm();

  public:
    // set arm cmd
    double leftarm_phase = 0.5;
    double leftarm_amp = 1.0;
    double rightarm_phase = 0.5;
    double rightarm_amp = 1.0;
    // nlp model
    // stand
    // double gait_cycle = 1.0;
    // double left_theta_offest = 0.5;
    // double right_theta_offest = 0.5;
    // double left_phase_ratio = 0.0;
    // double right_phase_ratio = 0.0;
    // run
    // double gait_cycle = 0.7;
    // double left_theta_offest = 0.6;
    // double right_theta_offest = 0.1;
    // double left_phase_ratio = 0.6;
    // double right_phase_ratio = 0.6;
    // walk
    // double gait_cycle = 1.0;
    // double left_theta_offest = 0.4;
    // double right_theta_offest = 0.9;
    // double left_phase_ratio = 0.4;
    // double right_phase_ratio = 0.4;

    // double gait_cycle = 1.0;
    // double left_theta_offest = 0.5;
    // double right_theta_offest = 0.0;
    // double left_phase_ratio = 0.4;
    // double right_phase_ratio = 0.4;
    // hop
    // double gait_cycle = 0.7;
    // double left_theta_offest = 0.3;
    // double right_theta_offest = 0.3;
    // double left_phase_ratio = 0.3;
    // double right_phase_ratio = 0.3;
    double gait_cycle_d = 1.0;
    double gait_cycle = 0.7;
    double left_theta_offest = 0.35;
    double right_theta_offest = 0.85;

    double left_phase = 0.85;
    double right_phase = 0.35;
    int w2s = 0;
    // double left_phase_ratio = 0.35;
    // double right_phase_ratio = 0.35;
    // double left_theta_offest = 0;
    // double right_theta_offest = 0;
    double left_phase_ratio = 0.35;
    double right_phase_ratio = 0.35;
    Eigen::Vector3d command;

    Eigen::Vector3d command_last;
    // perform freq Hz
    double freqency = 100.0;
    Eigen::VectorXd inputdata_nlp;          //
    Eigen::VectorXd inputdata_nlp_encoder;  //
    Eigen::VectorXd outputdata_nlp;         //
    Eigen::VectorXd outputdata_nlp_encoder; //
    Eigen::VectorXd nlpout;
    Eigen::VectorXd history_pos;     //
    Eigen::VectorXd history_vel;     //
    Eigen::VectorXd history_action;  //
    Eigen::VectorXd history_gravity; //
    Eigen::VectorXd history_angvel;  //

    Eigen::VectorXd history_pos_obs; //
    Eigen::VectorXd history_vel_obs; //

    Eigen::VectorXd avg_x_vel;   //
    Eigen::VectorXd avg_y_vel;   //
    Eigen::VectorXd avg_yaw_vel; //

    // int obs_num = 54;
    // int obs_num = 175;
    // int obs_num = 57;
    int obs_num = 321;
    int encoder_num = 3600;
    // int encoder_num = 4;
    int latent_num = 4;
    int action_num = 23;
    int his_len = 50;
    int his_len_obs = 5;

    int lstm_layers = 0;
    int lstm_hidden_size = 0;
    // nlp output data
    double s = 2;
    double obs_scales_lin_vel = 2.0;
    double obs_scales_ang_vel = 0.25;
    Eigen::Vector3d command_scales;
    double obs_scales_dof_pos = 1.0;
    double obs_scales_dof_vel = 0.05;
    Eigen::VectorXd default_dof_pos;
    Eigen::VectorXd action_last;
    double height_scales = 5.0;
    double action_scales = 0.5;

#ifdef WEBOTS
    std::string model_pb = "./PythonModule/source/student/policy_5700_feet_high.pt";
    std::string encoder_pb = "./PythonModule/source/student/encoder_1100_5700_high.pt";

    std::string model2_pb = "./PythonModule/source/student/policy_1500_stand.pt";
    std::string encoder2_pb = "./PythonModule/source/student/encoder_1100_5700_high.pt";
#else
    std::string model_pb = "../PythonModule/source/student/policy_5700_feet_high.pt";
    std::string encoder_pb = "../PythonModule/source/student/encoder_1100_5700_high.pt";

    std::string model2_pb = "../PythonModule/source/student/policy_1500_stand.pt";
    std::string encoder2_pb = "../PythonModule/source/student/encoder_1100_5700_high.pt";
#endif

    NlpModel *nlpmodel;
    NlpModel *nlpmodel2;
};
#endif
