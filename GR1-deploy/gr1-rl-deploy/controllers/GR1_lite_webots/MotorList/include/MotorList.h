/**
 * @file MotorList.h
 * @author Ren Xiaoyu (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-07-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MOTOR_LIST_H_
#define MOTOR_LIST_H_
// #include "fourier-cpp-sdk/src/groupCommand.hpp"
// #include "fourier-cpp-sdk/src/groupFeedback.hpp"
// #include "fourier-cpp-sdk/src/lookup.hpp"
// #include "fourier-cpp-sdk/fourier/include/aios.h"
// #include "../../S2P/include/functionIKID_S2P.h"
#include "../../S2P/include/functionIKID_S2P.h"
#include "frictionnlpfunction.hpp"
#include "fsa-cpp-sdk/include/FsaMutiMotor.h"
#include <Eigen/Dense>
// // #include "twoOrderFilter.h"
#include "../../RobotController/include/KalmanFilter.h"
#include "../../RobotController/include/LowPassFilter.h"
#include <QCoreApplication>
#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>
#include <QStringList>
#include <fstream>
#include <map>
#define MOTOR_DATA_LOG
typedef float (*frictionNLP)(float, double, double);
class MotorList {
  public:
    //
    MotorList(int allnum) : motor(allnum){};
    // MotorList(int num, std::vector<std::string> ip_list);
    // ~MotorList(int allnum);
    /**
     * @brief Construct a new init object
     * @function: 1 read motor para; 2 construct comunication with motors;
     * @param num
     * @param ip_list
     * @param path
     */
    void init(int num, double dt, Eigen::VectorXd &absolute_pos, std::vector<std::string> ip, QString path);
    // filtertypeforfriciton: 0 for qdotcmd ,1 for qdotakalman, 2 for qdotalowpass
    // frictiontype: 0 for none, 1 for stribeck, 2 for stribeck and nlp
    void setcommand(Eigen::VectorXd pos_cmd, Eigen::VectorXd vel_cmd, Eigen::VectorXd tor_cmd, int frictiontype, int filtertypeforfriciton);
    void setcommand(Eigen::VectorXd pos_cmd, Eigen::VectorXd vel_cmd, Eigen::VectorXd tor_cmd, int frictiontype, int filtertypeforfriciton, Eigen::VectorXd vel_cmd_actual);

    // filtertype: 0 for none ,1 for kalman, 2 for lowpass
    void getstate(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &tor, int filtertype);
    // set tolerate losing pakage number
    void settolerance_count(int _count);
    // disable
    void disable();

  private:
    // group
    // std::shared_ptr<Fourier::Group> group;
    // //
    // Fourier::GroupFeedback *feedback;
    // //
    // Fourier::GroupCommand *group_command;
    // motor num
    int motornum;
    // command frequency
    double dt;
    // std::vector<FSA_CONNECT::FSA> motor;
    FSA_CONNECT::FSAMutiMotor motor;

    // iplist
    std::vector<std::string> ip_list;
    // Eigen::VectorXd motorIp;
    // Eigen::VectorXi sortedInd;
    // Eigen::VectorXd sortedIpVec;
    std::map<std::string, int> motorid;
    //-----------absolute encoder--------------//
    Eigen::VectorXd absolute_pos_;
    Eigen::VectorXd absolute_pos_zero_;
    Eigen::VectorXd absolute_pos_dir_;
    Eigen::VectorXd absolute_pos_gear_ratio_;
    //-----------------------------------------//
    // gear ratio
    std::vector<double> motor_gear_ratio_;
    // motor direction
    std::vector<int> motorDir_;
    // motor current-torque scale
    std::vector<double> c_t_scale_;
    // set linear count, same sort with group
    std::vector<float> linearCount_;
    std::vector<FSA_CONNECT::FSAConfig::FSAPIDParams> controlConfig_;
    // set motor config
    // std::vector<MotionControllerConfig *> controlConfig_;
    // start enabling Devieces
    std::vector<float> enable_status_;
    // control commmand
    // std::vector<PosPtInfo> controlCommand_;
    // kalman filter
    std::vector<KalmanFilter *> kalman_joint_v_;
    Eigen::VectorXd d_sysnoise_;
    Eigen::VectorXd d_meannoise_;
    // lowpass
    std::vector<LowPassFilter *> lowpass_;
    // friction stribeck para: a b S alpha v
    Eigen::MatrixXd stribeck_para_;
    // friction NLP
    // std::vector<frictionNLP> frictionnlp_;
    std::map<std::string, frictionNLP> frictionnlp_;
    // from NLP trans to stribeck: speed
    Eigen::VectorXd v_lb_;
    Eigen::VectorXd v_ub_;
    // data

    Eigen::VectorXd poscmd;
    Eigen::VectorXd velcmd;
    Eigen::VectorXd torcmd;
    Eigen::VectorXd qa;
    Eigen::VectorXd qda;
    Eigen::VectorXd qda_lowpass;
    Eigen::VectorXd qda_kalman;
    Eigen::VectorXd currenta;
    Eigen::VectorXd qtora;
    // sorted vector
    // void sort_vec(const Eigen::VectorXd& vec, Eigen::VectorXd& sorted_vec,  Eigen::VectorXi& ind);
    // stribeck
    double frictioncompensation_stribeck(double a, double b, double s, double alpha, double v, double qdot, double ctscale, double gear_ratio);
    // interp function
    bool quintic(double x1, double x2, double y1, double y2, double dy1, double dy2, double ddy1, double ddy2, double x, double dx, double &y, double &dy, double &ddy);
    double clamp(double num, double lim1, double lim2);
    double myPow(double x, int n);
    // communication error
    std::vector<int> loseerror_count;
    int tolerance_count; // default 1;
    Eigen::VectorXd fricitonnlp_tor;
    Eigen::VectorXd frictionstribeck_tor;
// data log
#ifdef MOTOR_DATA_LOG
    // data log
    std::ofstream foutData;
    Eigen::VectorXd dataL;
    bool dataLog(Eigen::VectorXd &v, std::ofstream &f);
#endif
};
#endif