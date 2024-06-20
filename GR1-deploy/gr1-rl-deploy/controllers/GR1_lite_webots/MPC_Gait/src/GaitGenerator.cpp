#include "../include/GaitGenerator.h"
// #include "GaitGenerator.h"
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
#include <vector>

GaitGenerator::GaitGenerator() {}

GaitGenerator::~GaitGenerator() {}

void GaitGenerator::init(QString path, QString path2, double _dt, DataPackage *data) {

    // init robot_control
    robot_controller_.init(path, _dt);
    robot_controller_.init_DataPackage(data);

    // init fsm
    initFsm();
    // init nlp
    command_scales[0] = obs_scales_lin_vel;
    command_scales[1] = obs_scales_lin_vel;
    command_scales[2] = obs_scales_ang_vel;
    default_dof_pos = Eigen::VectorXd::Zero(action_num);
    default_dof_pos << 0.0, 0.0, -0.5236, 1.0472, -0.5236, 0.0,
        0.0, 0.0, -0.5236, 1.0472, -0.5236, 0.0,
        0, 0, 0,
        0.0, 0.3, 0.0, -0.3, 0.0, -0.3, 0.0, -0.3;
    action_last = Eigen::VectorXd::Zero(action_num);

    command.setZero();
    // perform freq Hz

    inputdata_nlp = Eigen::VectorXd::Zero(obs_num);
    inputdata_nlp_encoder = Eigen::VectorXd::Zero(encoder_num);
    outputdata_nlp = Eigen::VectorXd::Zero(action_num);
    nlpout = Eigen::VectorXd::Zero(action_num);
    outputdata_nlp_encoder = Eigen::VectorXd::Zero(latent_num);

    history_pos = Eigen::VectorXd::Zero(action_num * his_len);
    history_vel = Eigen::VectorXd::Zero(action_num * his_len);
    history_action = Eigen::VectorXd::Zero(action_num * his_len);
    history_gravity = Eigen::VectorXd::Zero(3 * his_len);
    history_angvel = Eigen::VectorXd::Zero(3 * his_len);
    history_pos_obs = Eigen::VectorXd::Zero(action_num * his_len_obs);
    history_vel_obs = Eigen::VectorXd::Zero(action_num * his_len_obs);

    avg_x_vel = Eigen::VectorXd::Zero(freqency);
    avg_y_vel = Eigen::VectorXd::Zero(freqency);
    avg_yaw_vel = Eigen::VectorXd::Zero(freqency);
    // std::cout<<"gait->inputdata gait: "<<std::endl<<inputdata_nlp.transpose()<<std::endl;
    std::cout << "nlp model start construct!" << std::endl;
    nlpmodel = new NlpModel();
    nlpmodel2 = new NlpModel();
    nlpmodel->init(model_pb, encoder_pb, obs_num, encoder_num, action_num, latent_num, lstm_layers, lstm_hidden_size);
    nlpmodel2->init(model2_pb, encoder2_pb, obs_num, encoder_num, action_num, latent_num, lstm_layers, lstm_hidden_size);
    std::cout << "nlp model construct complete!" << std::endl;
    W2S = false;
}

int GaitGenerator::initFsm() {
    startstate = new Start(this);

    zerostate = new Zero(this);
    nlpstate = new NLP(this);

    startstate->init();

    zerostate->init();
    nlpstate->init();

    fsm.addState(nlpstate);
    fsm.addState(startstate, 1);

    fsm.addState(zerostate);

    fsm.initialize();

    std::cout << "FSM init " << std::endl;
    return 0;
}

void GaitGenerator::gait_run(DataPackage *data_) {
    // update joint state
    robot_controller_.set_inputdata(data_);
    fsm.runFSM(event);
    //
    fsm.currentstatename(fsmstatename);
    // std::cout<<"gait_run!"<<std::endl;
    robot_controller_.get_outputdata(data_);
    // std::cout<<"get_outputdata!"<<std::endl;
}

void GaitGenerator::setevent(string event_) {
    // if(event_ == "")
    // {

    // }else{
    //     if(fsmstatename == "Walk")
    //     {
    //         if(event_ == "gotoZ2S"){
    //             W2S = true;
    //         }
    //         if((W2S == true)&&(event_ == "gotoStand"))
    //         {
    //             event = event_;
    //         }
    //     }else{
    event = event_;
    //     }
    // }
}

void GaitGenerator::set_current_fsm_command(string current_command) {
    current_fsmstate_command = current_command;
}

void GaitGenerator::setvelocity(double vx, double vy, double vyaw) {
    robot_controller_._robot_data->vCmd_joystick(0) = vx;
    robot_controller_._robot_data->vCmd_joystick(1) = vy;
    robot_controller_._robot_data->vCmd_joystick(2) = vyaw;
}

void GaitGenerator::setvelocity_offset(double vx_offest, double vy_offset) {
    robot_controller_._robot_data->vCmd_offset_joystick(0) = vx_offest;
    robot_controller_._robot_data->vCmd_offset_joystick(1) = vy_offset;
    robot_controller_._robot_data->vCmd_offset_joystick(2) = 0.0;
}

void GaitGenerator::setxyz(double x, double y, double z) {
    robot_controller_._robot_data->pCmd_joystick(0) = x;
    robot_controller_._robot_data->pCmd_joystick(1) = y;
    robot_controller_._robot_data->pCmd_joystick(2) = z;
}

void GaitGenerator::setrollpitch(double roll, double pitch) {
}

// robotarm
double GaitGenerator::getleftamp() {
    return leftarm_amp;
}
double GaitGenerator::getleftphase() {
    return leftarm_phase;
}
double GaitGenerator::getrightamp() {
    return rightarm_amp;
}
double GaitGenerator::getrightphase() {
    return rightarm_phase;
}

void GaitGenerator::FifthPoly(Eigen::VectorXd p0, Eigen::VectorXd p0_dot, Eigen::VectorXd p0_dotdot, // start point states
                              Eigen::VectorXd p1, Eigen::VectorXd p1_dot, Eigen::VectorXd p1_dotdot, // end point states
                              double totalTime,                                                      // total permating time
                              double currenttime,                                                    // current time,from 0 to total time
                              Eigen::VectorXd &pd, Eigen::VectorXd &pd_dot, Eigen::VectorXd &pd_dotdot) {
    double t = currenttime;
    double time = totalTime;
    if (t < totalTime) {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
        A << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1 / 2, 0, 0, 0,
            -10 / pow(time, 3), -6 / pow(time, 2), -3 / (2 * time), 10 / pow(time, 3), -4 / pow(time, 2), 1 / (2 * time),
            15 / pow(time, 4), 8 / pow(time, 3), 3 / (2 * pow(time, 2)), -15 / pow(time, 4), 7 / pow(time, 3), -1 / pow(time, 2),
            -6 / pow(time, 5), -3 / pow(time, 4), -1 / (2 * pow(time, 3)), 6 / pow(time, 5), -3 / pow(time, 4), 1 / (2 * pow(time, 3));
        Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(6, 1);
        Eigen::MatrixXd a = Eigen::MatrixXd::Zero(6, 1);
        for (int i = 0; i < p0.size(); i++) {
            x0 << p0(i), p0_dot(i), p0_dotdot(i), p1(i), p1_dot(i), p1_dotdot(i);
            a = A * x0;
            pd(i) = a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t + a(5) * t * t * t * t * t;
            pd_dot(i) = a(1) + 2 * a(2) * t + 3 * a(3) * t * t + 4 * a(4) * t * t * t + 5 * a(5) * t * t * t * t;
            pd_dotdot(i) = 2 * a(2) + 6 * a(3) * t + 12 * a(4) * t * t + 20 * a(5) * t * t * t;
        }
    } else {
        pd = p1;
        pd_dot = p1_dot;
        pd_dotdot = p1_dotdot;
    }
}

void GaitGenerator::FifthPoly(double p0, double p0_dot, double p0_dotdot,    // start point states
                              double p1, double p1_dot, double p1_dotdot,    // end point states
                              double totalTime,                              // total permating time
                              double currenttime,                            // current time,from 0 to total time
                              double &pd, double &pd_dot, double &pd_dotdot) // output command
{
    double t = currenttime;
    double time = totalTime;
    if (t < totalTime) {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
        A << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1 / 2, 0, 0, 0,
            -10 / pow(time, 3), -6 / pow(time, 2), -3 / (2 * time), 10 / pow(time, 3), -4 / pow(time, 2), 1 / (2 * time),
            15 / pow(time, 4), 8 / pow(time, 3), 3 / (2 * pow(time, 2)), -15 / pow(time, 4), 7 / pow(time, 3), -1 / pow(time, 2),
            -6 / pow(time, 5), -3 / pow(time, 4), -1 / (2 * pow(time, 3)), 6 / pow(time, 5), -3 / pow(time, 4), 1 / (2 * pow(time, 3));
        Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(6, 1);
        Eigen::MatrixXd a = Eigen::MatrixXd::Zero(6, 1);
        x0 << p0, p0_dot, p0_dotdot, p1, p1_dot, p1_dotdot;
        a = A * x0;
        pd = a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t + a(5) * t * t * t * t * t;
        pd_dot = a(1) + 2 * a(2) * t + 3 * a(3) * t * t + 4 * a(4) * t * t * t + 5 * a(5) * t * t * t * t;
        pd_dotdot = 2 * a(2) + 6 * a(3) * t + 12 * a(4) * t * t + 20 * a(5) * t * t * t;
    } else {
        pd = p1;
        pd_dot = p1_dot;
        pd_dotdot = p1_dotdot;
    }
}

void GaitGenerator::Thirdpoly(double p0, double p0_dot, double p1, double p1_dot,
                              double totalTime,   // total permating time
                              double currenttime, // current time,from 0 to total time
                              double &pd, double &pd_dot) {
    if (currenttime < totalTime) {
        double a0 = p0;
        double a1 = p0_dot;
        double m = p1 - p0 - p0_dot * totalTime;
        double n = p1_dot - p0_dot;
        double a2 = 3 * m / (totalTime * totalTime) - n / totalTime;
        double a3 = -2 * m / (totalTime * totalTime * totalTime) + n / (totalTime * totalTime);
        pd = a3 * currenttime * currenttime * currenttime + a2 * currenttime * currenttime + a1 * currenttime + a0;
        pd_dot = 3 * a3 * currenttime * currenttime + 2 * a2 * currenttime + a1;
    } else {
        pd = p1;
        pd_dot = p1_dot;
    }
}

void GaitGenerator::LinePoly(double p0, double p1,
                             double totalTime,   // total permating time
                             double currenttime, // current time,from 0 to total time
                             double &pd, double &pd_dot) {
    if (currenttime < totalTime) {
        pd_dot = (p1 - p0) / totalTime;
        pd = p0 + pd_dot * currenttime;
    } else {
        pd = p1;
        pd_dot = 0;
    }
}
void GaitGenerator::ExcitingTrajectoryFunction(Eigen::VectorXd series_para,
                                               Eigen::VectorXd init_pos,
                                               int dof_num,
                                               int series_num,
                                               double base_freq,
                                               double curtime,
                                               Eigen::VectorXd &pos,
                                               Eigen::VectorXd &vel,
                                               Eigen::VectorXd &acc) {
    double base_ang_freq = 2 * M_PI * base_freq;
    //   std::cout<<"1"<<std::endl;
    // assert series parameters
    assert(series_para.rows() == (2 * series_num * dof_num));

    pos.setZero();
    vel.setZero();
    acc.setZero();
    //   std::cout<<"1"<<std::endl;
    for (int i = 0; i < dof_num; i++) {
        pos(i) = init_pos(i);
        for (int j = 1; j < (series_num + 1); j++) {
            pos(i) += series_para(j + 2 * series_num * i - 1) * sin(base_ang_freq * j * curtime);
            pos(i) += series_para(j + 2 * series_num * i + series_num - 1) * cos(base_ang_freq * j * curtime);

            vel(i) += series_para(j + 2 * series_num * i - 1) * cos(base_ang_freq * j * curtime) * base_ang_freq * j;
            vel(i) -= series_para(j + 2 * series_num * i + series_num - 1) * sin(base_ang_freq * j * curtime) * base_ang_freq * j;

            acc(i) -= series_para(j + 2 * series_num * i - 1) * sin(base_ang_freq * j * curtime) * base_ang_freq * base_ang_freq * j * j;
            acc(i) -= series_para(j + 2 * series_num * i + series_num - 1) * cos(base_ang_freq * j * curtime) * base_ang_freq * base_ang_freq * j * j;
        }
    }
}
// unit Quaternion interpolation
Eigen::Matrix3d GaitGenerator::QuanteiniontoMatrix(RigidBodyDynamics::Math::Quaternion Q) {
    double x = Q[0];
    double y = Q[1];
    double z = Q[2];
    double w = Q[3];
    Eigen::Matrix3d R;
    R << 1 - 2 * y * y - 2 * z * z,
        2 * x * y - 2 * w * z,
        2 * x * z + 2 * w * y,

        2 * x * y + 2 * w * z,
        1 - 2 * x * x - 2 * z * z,
        2 * y * z - 2 * w * x,

        2 * x * z - 2 * w * y,
        2 * y * z + 2 * w * x,
        1 - 2 * x * x - 2 * y * y;
    return R;
}
void GaitGenerator::QuaternionInterp(Eigen::Matrix3d R_start,
                                     Eigen::Matrix3d R_end,
                                     double totaltime, double currenttime,
                                     Eigen::Matrix3d &R_d, Eigen::Vector3d &omiga_d, Eigen::Vector3d &acc_d) {
    RigidBodyDynamics::Math::Quaternion Q_start = Q_start.fromMatrix(R_start).conjugate();
    RigidBodyDynamics::Math::Quaternion Q_end = Q_end.fromMatrix(R_end).conjugate();
    RigidBodyDynamics::Math::Quaternion Q_d;

    RigidBodyDynamics::Math::Quaternion deltQ = Q_start.conjugate() * Q_end;

    Eigen::Vector3d p = (deltQ.block<3, 1>(0, 0));
    if (p.norm() != 0) {
        p = p / p.norm();
    }

    double delttheta = 2 * acos(deltQ(3));

    Eigen::VectorXd p0 = Eigen::VectorXd::Zero(1); // p0 = p0 p0_dot p0_ddot p1_dot p1_ddot

    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(1);
    p1[0] = delttheta;

    Eigen::VectorXd pd = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd pd_dot = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd pd_ddot = Eigen::VectorXd::Zero(1);

    FifthPoly(p0, p0, p0, p1, p0, p0, totaltime, currenttime, pd, pd_dot, pd_ddot);
    deltQ.block<3, 1>(0, 0) = p * sin(pd[0] / 2.0);
    deltQ[3] = cos(pd[0] / 2.0);
    Q_d = Q_start * deltQ;
    // Q_d = Q_d/sqrt(Q_d.dot(Q_d));

    R_d = QuanteiniontoMatrix(Q_d);
    omiga_d = R_start * p * pd_dot[0];
    acc_d = R_start * p * pd_ddot[0];

    // std::cout<<"R_d:"<<std::endl<<R_d<<std::endl;
    // std::cout<<"Q_end:"<<std::endl<<Q_end<<std::endl;
    // std::cout<<"omiga_d:"<<std::endl<<omiga_d.transpose()<<std::endl;
    // std::cout<<"acc_d:"<<std::endl<<acc_d.transpose()<<std::endl;
}
