#ifndef STATE_H
#define STATE_H

#include "../../RobotController/include/LowPassFilter.h"
#include "../../WalkPlan/include/aeroWalkPlan.h"
#include "../../WalkPlan/include/gaitPlan.h"
#include "XFsmState.h"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

class Start : public XFsmState {
  public:
    Start(void *App) : XFsmState(App){};
    void onEnter();
    void run();
    void onExit();
    void init();
};

class Zero : public XFsmState {
  public:
    Zero(void *App) : XFsmState(App){};
    void onEnter();
    void run();
    void onExit();
    void init();
    // clock
    double timer = 0.0;
    // current joint states
    Eigen::MatrixXd qa;
    Eigen::MatrixXd qa_dot;
    // target joint states
    Eigen::MatrixXd qd;
    Eigen::MatrixXd qd_dot;
    // current joint states
    Eigen::MatrixXd qaArm;
    Eigen::MatrixXd qa_dotArm;
    // target joint states
    Eigen::MatrixXd qdArm;
    Eigen::MatrixXd qd_dotArm;
    // current joint states
    Eigen::MatrixXd qaWaist;
    Eigen::MatrixXd qa_dotWaist;
    // target joint states
    Eigen::MatrixXd qdWaist;
    Eigen::MatrixXd qd_dotWaist;
    // total time
    double totaltime;
    // average speed
    double avr_v;
    // data log
    std::ofstream foutData;
    Eigen::VectorXd dataL;

    Eigen::VectorXd xStand, qCmd, qDotCmd;
    bool footflag;
};

class NLP : public XFsmState {
  public:
    NLP(void *App) : XFsmState(App){};
    void onEnter();
    void run();
    void onExit();
    void init();
    // clock
    double timer = 0.0;
    int count = 0;
    //
    Eigen::VectorXd q_init;
    Eigen::VectorXd inputdata_nlp_tmp;
    //
    Eigen::VectorXd q_c;
    Eigen::VectorXd q_dot_c;
    Eigen::VectorXd q_ddot_c;
    Eigen::VectorXd tau_c;
    // data log
    std::ofstream foutData;
    Eigen::VectorXd dataL;
    //
    int mode_a = 0;
    LowPassFilter *omega_filter;
    LowPassFilter *omega_filter_lin_vel;
    LowPassFilter *omega_filter_dof_vel;
    LowPassFilter *base_vel;
};

#endif
