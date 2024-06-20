#include "../../RobotController/include/KalmanFilter.h"
#include "../../RobotController/include/LowPassFilter.h"
#include "broccoli/core/Time.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/time.h>
#include <thread>
#include <time.h>
#include <torch/script.h>
#include <unistd.h>
#include <vector>

#define JOYSTICK
// #define DATALOG_MAIN
// #define WEBOTS
#define REALROBOTS

// include open source
#include "../../MPC_Gait/include/GaitGenerator.h"
#include "../../WalkPlan/include/aeroWalkPlan.h"
#include "../../joystick/include/joystick.h"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <math.h>
#include <rbdl/rbdl.h>
#include <sys/time.h>
#ifdef WEBOTS
#include "../../WebotsInterface/include/webotsInterface.h"
#endif
#ifdef REALROBOTS
#include "../../MotorList/include/MotorList.h"
// #include "../../vnIMU/include/vnIMU.h"
#include "../../ch108IMU/include/ch108IMU.h"
// #include "fourier-cpp-sdk/fourier/include/aios.h"
// #include "fourier-cpp-sdk/src/groupCommand.hpp"
// #include "fourier-cpp-sdk/src/groupFeedback.hpp"
// #include "fourier-cpp-sdk/src/lookup.hpp"
#endif
//
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
using namespace broccoli::core;
#ifndef PI
#define PI 3.141592654
#endif // PI

// json read
void readabsoluteencoder(QString path, Eigen::VectorXd &initpos);
// imu data
#ifdef REALROBOTS
// Eigen::VectorXd vnIMU::imuData = Eigen::VectorXd::Zero(9);
#endif
int main(int argc, char **argv) {

#ifdef JOYSTICK
    Joystick_robot joy;
    joy.init();
#endif
    std::cout << "start construct webots!" << std::endl;
// Initialize Webots
#ifdef WEBOTS
    WebotsRobot GR1L_Sim;
    GR1L_Sim.initWebots();
    webotState robotStateSim;
    Eigen::VectorXd standPosCmd = Eigen::VectorXd::Zero(23);
    Eigen::VectorXd jointTorCmd = Eigen::VectorXd::Zero(23);
    Eigen::VectorXd jointP = Eigen::VectorXd::Zero(23);
    Eigen::VectorXd jointD = Eigen::VectorXd::Zero(23);
    double simTime = 0;
    double myInf = 100000;
    std::cout << "webots construct complete!" << std::endl;
#endif

#ifdef WEBOTS
    Time start_time;
    Time period(0, 2500000);
    Time sleep2Time;
    Time timer;
    timespec sleep2Time_spec;
    double dt = 0.0025;
#endif

#ifdef REALROBOTS
    Time start_time;
    Time period(0, 2500000);
    Time sleep2Time;
    Time timer;
    timespec sleep2Time_spec;
    double dt = 0.0025;
#endif
// s2p
#ifdef REALROBOTS
    functionIKID_S2P *funS2P = new functionIKID_S2P();
    Eigen::VectorXd parallelQEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd parallelQDotEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd parallelQTorEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd parallelQCmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd parallelQDotCmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd parallelQTorCmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleOrienEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleOmegaEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleTorEst = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleOrienRef = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleOmegaRef = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd ankleTorDes = Eigen::VectorXd::Zero(4);
#endif
    //
    int motorNum = 23;
    Eigen::VectorXd absolute_pos = Eigen::VectorXd::Zero(motorNum);
#ifdef REALROBOTS
    // int motorNumleg = 12;
    ch108::ch108IMU imu;
    imu.initIMU();
    // read absolute encoder
    QString Anglepath = "../absAngle.json";

    Eigen::VectorXd absolute_pos_leg = Eigen::VectorXd::Zero(motorNum);
    readabsoluteencoder(Anglepath, absolute_pos_leg);
    // absolute_pos.head(motorNum) = absolute_pos_leg;
    absolute_pos = absolute_pos_leg;
    std::vector<std::string> ip(motorNum);
    ip[0] = "192.168.137.70";
    ip[1] = "192.168.137.71";
    ip[2] = "192.168.137.72";
    ip[3] = "192.168.137.73";
    ip[4] = "192.168.137.74";
    ip[5] = "192.168.137.75";

    ip[6] = "192.168.137.50";
    ip[7] = "192.168.137.51";
    ip[8] = "192.168.137.52";
    ip[9] = "192.168.137.53";
    ip[10] = "192.168.137.54";
    ip[11] = "192.168.137.55";

    ip[12] = "192.168.137.90";
    ip[13] = "192.168.137.91";
    ip[14] = "192.168.137.92";

    ip[15] = "192.168.137.10";
    ip[16] = "192.168.137.11";
    ip[17] = "192.168.137.12";
    ip[18] = "192.168.137.13";

    ip[19] = "192.168.137.30";
    ip[20] = "192.168.137.31";
    ip[21] = "192.168.137.32";
    ip[22] = "192.168.137.33";

    QString motorlistpath = "../MotorList/sources/motorlist.json";
    MotorList motorlist(motorNum);
    motorlist.init(motorNum, dt, absolute_pos, ip, motorlistpath);
    motorlist.settolerance_count(3);
    std::cout << "absolute_pos: " << absolute_pos.transpose() << std::endl;
//
#endif
    //
    Eigen::VectorXd qEstDeg = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotEstDeg = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qEst = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotEst = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotEst_lowpass = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotEst_kalman = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd currentEst = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qTorEst = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDDotRef = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd currentCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qTorCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd frictioncurrentCmd = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd frictioncurrentCmd2 = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd frictionTor = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qCmd2 = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotCmd2 = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qTorCmd2 = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qEst_p = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd qDotEst_p = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd q_factor = Eigen::VectorXd::Ones(motorNum);
    Eigen::VectorXd qdot_factor = Eigen::VectorXd::Ones(motorNum);
    Eigen::VectorXd qDotfriciton = Eigen::VectorXd::Zero(motorNum);

    Eigen::VectorXd god_vision = Eigen::VectorXd::Zero(6);

    // init pos
    bool flag_start = true;
    Eigen::VectorXd qEstInit = Eigen::VectorXd::Zero(motorNum);

    int simCnt = 0;
    double timeSim = 0.0;
    double timeStep = dt;
    double timeTotal = 500.0;
    int simTotalNum = timeTotal / timeStep + 10;

    std::cout << "simTotalNum: " << simTotalNum << std::endl;
    //
    int motionStartCnt = 75;     // min >= 75*0.004
    int cartMotionCycleCnt = 75; // min >= 75*0.004
//
#ifdef DATALOG_MAIN
    std::ofstream foutData;
    foutData.open("datacollection.txt", std::ios::out);
    // Eigen::VectorXd dataL = Eigen::VectorXd::Zero(3 + 10 * motorNum + 50);
    Eigen::VectorXd dataL = Eigen::VectorXd::Zero(13);
#endif

    DataPackage *data = new DataPackage();
    // construct robot controller
    GaitGenerator gait;
    std::cout << "start json init!" << std::endl;
    // to init the task state
#ifdef WEBOTS
    QString path = "./br/GR1L.json";
#else
    QString path = "../br/GR1L.json";
#endif
    QString path2 = " ";
    gait.init(path, path2, dt, data);
    std::cout << "data init!" << std::endl;

#ifdef WEBOTS
#else
    int aaaa;
    std::cout << "start?" << std::endl;
    std::cin >> aaaa;
    std::cout << "start!" << std::endl;
#endif

#ifdef WEBOTS
    while (GR1L_Sim.robot->step(TIME_STEP) != -1)
#endif
#ifdef REALROBOTS
        while (true)
#endif
        {
            start_time = timer.currentTime();

//--------------------state estimation--------------------
#ifdef WEBOTS
            simTime = GR1L_Sim.robot->getTime();
            // std::cout<<"webots get sim time step!"<<std::endl;

            GR1L_Sim.readData(simTime, robotStateSim, god_vision);

            // std::cout<<"webots read sensor data!"<<std::endl;
            qEst = robotStateSim.jointPosAct.head(motorNum);
            qDotEst = robotStateSim.jointVelAct.head(motorNum);
            qTorEst = jointTorCmd.head(motorNum);

#endif
#ifdef REALROBOTS
            motorlist.getstate(qEst, qDotEst, qTorEst, 0);
            //--------------------Parrel to Serial Transform--------------------
            qEst_p = qEst;
            qDotEst_p = qDotEst;

            // std::cout<<"hehe: 1"<< std::endl;
            // p-joint to s-joint
            parallelQEst << qEst(4), qEst(5), qEst(10), qEst(11);
            parallelQDotEst << qDotEst(4), qDotEst(5), qDotEst(10), qDotEst(11);
            parallelQTorEst << qTorEst(4), qTorEst(5), qTorEst(10), qTorEst(11);
            funS2P->setEst(parallelQEst, parallelQDotEst, parallelQTorEst); // set parallel joint pos and vel
            funS2P->calcFK();                                               // calc serial joint pos
            funS2P->calcIK();                                               // calc jacobian at this pose and serial joint vel
            funS2P->getAnkleState(ankleOrienEst, ankleOmegaEst, ankleTorEst);
            qEst(4) = ankleOrienEst(0);
            qEst(5) = ankleOrienEst(1);
            qEst(10) = ankleOrienEst(2);
            qEst(11) = ankleOrienEst(3);
            qDotEst(4) = ankleOmegaEst(0);
            qDotEst(5) = ankleOmegaEst(1);
            qDotEst(10) = ankleOmegaEst(2);
            qDotEst(11) = ankleOmegaEst(3);
            qTorEst(4) = ankleTorEst(0);
            qTorEst(5) = ankleTorEst(1);
            qTorEst(10) = ankleTorEst(2);
            qTorEst(11) = ankleTorEst(3);
#endif
            //---------------------high-level control-----------------------
            //

            if (timeSim < 1.0) {
                for (int i = 0; i < motorNum; i++) {
                    qCmd[i] = absolute_pos(i);
                }
                qDotCmd.setZero();
                q_factor = 1.0 * Eigen::VectorXd::Ones(motorNum);
                qdot_factor = 1.0 * Eigen::VectorXd::Ones(motorNum);
            } else {
                //         // set actual task variable
                // std::cout<<"gait run!"<<std::endl;
                data->dim = 18;
                data->dt = dt;
                // // right leg
                // std::cout<<"qEst: "<<data->q_a<<std::endl;
                data->q_a.block(6, 0, 12, 1) = qEst.head(12);
                // std::cout<<"qdotEst: "<<qDotEst.transpose()<<std::endl;
                data->q_dot_a.block(6, 0, 12, 1) = qDotEst.head(12);
                // std::cout<<"qTor: "<<qTorEst.transpose()<<std::endl;
                data->tau_a.block(6, 0, 12, 1) = qTorEst.head(12);

                data->q_a_Waist = qEst.block(12, 0, 3, 1);
                data->q_dot_a_Waist = qDotEst.block(12, 0, 3, 1);
                data->tau_a_Waist = qTorEst.block(12, 0, 3, 1);

                data->q_a_Arm = qEst.tail(8);
                data->q_dot_a_Arm = qDotEst.tail(8);
                data->tau_a_Arm = qTorEst.tail(8);
                data->imu_sensor.block(0, 0, 18, 1).setZero();
// std::cout<<"motor data set complete!"<<std::endl;
#ifdef WEBOTS
                data->imu_sensor.block(0, 0, 9, 1) = robotStateSim.imu9DAct;
#endif
#ifdef REALROBOTS
                // data->imu_sensor.block(0, 0, 9, 1) = vnIMU::imuData;
                data->imu_sensor.block(0, 0, 9, 1) = imu.imudata;

#endif
// // //
#ifdef JOYSTICK
                if (gait.fsmstatename == "S2W" || gait.fsmstatename == "Z2S") {
                } else {
                    gait.setevent(joy.get_state_change());
                    gait.set_current_fsm_command(joy.get_current_state_command());
                }

                if (gait.fsmstatename == "NLP") {
                    gait.setvelocity(joy.get_walk_x_direction_speed(),
                                     joy.get_walk_y_direction_speed(),
                                     joy.get_walk_yaw_direction_speed());
                    gait.setvelocity_offset(joy.get_walk_x_direction_speed_offset(),
                                            joy.get_walk_y_direction_speed_offset());
                } else {
                }
#endif
                // std::cout<<"event: "<<gait.event<<std::endl;

                gait.gait_run(data);

                // //
                qCmd.block(0, 0, 12, 1) = data->q_c.block(6, 0, 12, 1);
                qDotCmd.block(0, 0, 12, 1) = data->q_dot_c.block(6, 0, 12, 1);
                qTorCmd.block(0, 0, 12, 1) = data->tau_c.block(6, 0, 12, 1);
                q_factor.block(0, 0, 12, 1) = data->q_factor;
                qdot_factor.block(0, 0, 12, 1) = data->q_dot_factor;

                qCmd.block(12, 0, 3, 1) = data->q_waist_c;
                qDotCmd.block(12, 0, 3, 1) = data->q_ddot_c_Waist;
                qTorCmd.block(12, 0, 3, 1) = data->tau_c_Waist;
                q_factor.block(12, 0, 3, 1) = data->q_factor_Waist;
                qdot_factor.block(12, 0, 3, 1) = data->q_dot_factor_Waist;

                qCmd.block(15, 0, 8, 1) = data->q_c_Arm;
                qDotCmd.block(15, 0, 8, 1) = data->q_ddot_c_Arm;
                qTorCmd.block(15, 0, 8, 1) = data->tau_c_Arm;
                q_factor.block(15, 0, 8, 1) = data->q_factor_Arm;
                qdot_factor.block(15, 0, 8, 1) = data->q_dot_factor_Arm;

                //----------------------------------//
            }
#ifdef REALROBOTS
            //--------------------Serial to Parrel Transform--------------------
            ankleOrienRef(0) = qCmd(4);
            ankleOrienRef(1) = qEst(5);
            ankleOrienRef(2) = qCmd(10);
            ankleOrienRef(3) = qEst(11);

            ankleOmegaRef(0) = qDotCmd(4);
            ankleOmegaRef(1) = 0.0;
            ankleOmegaRef(2) = qDotCmd(10);
            ankleOmegaRef(3) = 0.0;
            // time3 = timer.currentTime() - start_time - time2 - time1;
            // // s2p
            ankleTorDes << qTorCmd(4), qTorCmd(5), qTorCmd(10), qTorCmd(11);
            funS2P->setDes(ankleOrienRef, ankleOmegaRef);
            funS2P->calcJointPosRef();
            funS2P->setDesTorque(ankleTorDes);
            funS2P->calcJointTorDes();
            funS2P->getDes(parallelQCmd, parallelQDotCmd, parallelQTorCmd);
            qCmd(4) = parallelQCmd(0);
            qCmd(5) = parallelQCmd(1);
            qCmd(10) = parallelQCmd(2);
            qCmd(11) = parallelQCmd(3);
            qDotCmd(4) = parallelQDotCmd(0);
            qDotCmd(5) = parallelQDotCmd(1);
            qDotCmd(10) = parallelQDotCmd(2);
            qDotCmd(11) = parallelQDotCmd(3);
            qTorCmd(4) = parallelQTorCmd(0);
            qTorCmd(5) = parallelQTorCmd(1);
            qTorCmd(10) = parallelQTorCmd(2);
            qTorCmd(11) = parallelQTorCmd(3);
#endif

#ifdef WEBOTS
            for (int i = 0; i < motorNum; i++) {
                qCmd2(i) = (1.0 - q_factor(i)) * qEst(i) + q_factor(i) * qCmd(i);
                qDotCmd2(i) = (1.0 - qdot_factor(i)) * qDotEst(i) + qdot_factor(i) * qDotCmd(i);
            }

            qTorCmd2 = qTorCmd;
            // jointP << 251.625, 362.5214, 71, 71, 10.9805, 0.25,
            //     251.625, 362.5214, 71, 71, 10.9805, 0.25,
            //     362.5214, 362.5214, 362.5214,
            //     92.85, 92.85, 112.06, 112.06,
            //     92.85, 92.85, 112.06, 112.06;

            // jointP = 1. * jointP;
            // jointD << 14.72, 10.0833, 4, 4, 0.5991, 0.01,
            //     14.72, 10.0833, 4, 4, 0.5991, 0.01,
            //     10.0833, 10.0833, 10.0833,
            //     2.575, 2.575, 3.1, 3.1,
            //     2.575, 2.575, 3.1, 3.1;

            // jointD = 1 * jointD;
            jointP << 251.625, 362.5214, 200, 200, 10.9805, 0.25,
                251.625, 362.5214, 200, 200, 10.9805, 0.25,
                362.5214, 362.5214, 362.5214,
                92.85, 92.85, 112.06, 112.06,
                92.85, 92.85, 112.06, 112.06;

            jointP = 1. * jointP;
            jointD << 14.72, 10.0833, 11, 11, 0.5991, 0.1,
                14.72, 10.0833, 11, 11, 0.5991, 0.1,
                10.0833, 10.0833, 10.0833,
                2.575, 2.575, 3.1, 3.1,
                2.575, 2.575, 3.1, 3.1;

            // jointD = 1 * jointD;
            // qTorCmd2 = qTorCmd;
            // jointP << 328.5, 200.0, 300.0, 300.0, 10.0, 0.0,
            //     328.5, 200.0, 300.0, 300.0, 10.0, 0.0,
            //     // 200.0,200.0,200.0,
            //     20.0, 10.0, 10.0, 10.0,
            //     20.0, 10.0, 10.0, 10.0;
            // jointP = 1.2 * jointP;
            // jointD << 13.14, 4.0, 6.0, 6.0, 1.25, 0.175,
            //     13.14, 4.0, 6.0, 6.0, 1.25, 0.175,
            //     // 4.0,  4.0,  4.0,
            //     1., 1., 1., 1., 1., 1., 1., 1.;
            for (int i = 0; i < motorNum; i++) {
                jointTorCmd(i) = jointP(i) * (qCmd2(i) - qEst(i)) + jointD(i) * (-qDotEst(i));
                standPosCmd(i) = qCmd2(i); // myInf;
            }

            if (timeSim < 1.0 || gait.fsmstatename == "Start" || gait.fsmstatename == "Zero") {
                GR1L_Sim.setMotorPos(standPosCmd);
            } else {
                // GR1L_Sim.setMotorPos(standPosCmd);

                GR1L_Sim.setMotorTau(jointTorCmd);
            }
#endif

#ifdef REALROBOTS
            for (int i = 0; i < motorNum; i++) {
                qCmd2(i) = (1.0 - q_factor(i)) * qEst_p(i) + q_factor(i) * qCmd(i);
                qDotCmd2(i) = (1.0 - qdot_factor(i)) * qDotEst_p(i) + qdot_factor(i) * qDotCmd(i);
            }
            qTorCmd2 = qTorCmd;
            qTorCmd2.setZero();
            motorlist.setcommand(qCmd2, qDotCmd2, qTorCmd2, 0, 0, qDotfriciton);
#endif

            simCnt += 1;
            timeSim = simCnt * timeStep;
//
#ifdef DATALOG_MAIN
            dataL[0] = timeSim;
            dataL.block(1, 0, (motorNum), 1) = qCmd2;
            dataL.block(1 + (motorNum), 0, (motorNum), 1) = qDotCmd2;
            dataL.block(1 + 2 * (motorNum), 0, (motorNum), 1) = qTorCmd;
            dataL.block(1 + 3 * (motorNum), 0, (motorNum), 1) = qEst;
            dataL.block(1 + 4 * (motorNum), 0, (motorNum), 1) = qDotEst;
            dataL.block(1 + 5 * (motorNum), 0, (motorNum), 1) = qTorEst;
            dataL[1 + 6 * (motorNum) + 9] = start_time.m_seconds;
            dataL[1 + 6 * (motorNum) + 10] = start_time.m_nanoSeconds;
            // dataL.block(1, 0, 3, 1) = gait.robot_controller_._robot_data->q_a.head(3);
            // dataL.block(4, 0, 3, 1) = gait.robot_controller_._robot_data->q_dot_a.head(3);
            // dataL.block(7, 0, 6, 1) = god_vision;
            dataLog(dataL, foutData);
#endif
#ifdef REALROBOTS
            sleep2Time = start_time + period;
            sleep2Time_spec = sleep2Time.toTimeSpec();
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(sleep2Time_spec), NULL);
#endif
        }

#ifdef WEBOTS
    GR1L_Sim.deleteRobot();
#endif
#ifdef DATALOG_MAIN
    foutData.close();
#endif
    return 0;
}

void readabsoluteencoder(QString path, Eigen::VectorXd &initpos) {
    // read the json file
    QFile loadFile(path);
    if (!loadFile.open(QIODevice::ReadOnly)) {
        qDebug() << "could't open projects json";
        return;
    }

    QByteArray allData = loadFile.readAll();
    loadFile.close();

    QJsonParseError jsonerror;
    QJsonDocument doc(QJsonDocument::fromJson(allData, &jsonerror));

    if (jsonerror.error != QJsonParseError::NoError) {
        qDebug() << "json error!" << jsonerror.errorString();
        return;
    }

    if (!doc.isNull() && jsonerror.error == QJsonParseError::NoError) {
        if (doc.isObject()) {
            QJsonObject object = doc.object();
            QJsonObject::iterator it = object.begin();
            // read the dimesion of the variables
            while (it != object.end()) {
                if (it.key() == "192.168.137.170") {
                    QJsonObject it_object = it.value().toObject();
                    QJsonObject::iterator iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(0) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.171") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(1) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.172") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(2) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.173") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(3) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.174") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(4) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.175") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(5) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.150") {
                    QJsonObject it_object = it.value().toObject();
                    QJsonObject::iterator iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(6) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.151") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(7) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.152") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(8) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.153") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(9) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.154") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(10) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.155") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(11) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.190") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(12) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.191") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(13) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.192") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(14) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                it++;
            }
        }
    }
}

void readabsoluteencoder_lite(QString path, Eigen::VectorXd &initpos) {
    // read the json file
    QFile loadFile(path);
    if (!loadFile.open(QIODevice::ReadOnly)) {
        qDebug() << "could't open projects json";
        return;
    }

    QByteArray allData = loadFile.readAll();
    loadFile.close();

    QJsonParseError jsonerror;
    QJsonDocument doc(QJsonDocument::fromJson(allData, &jsonerror));

    if (jsonerror.error != QJsonParseError::NoError) {
        qDebug() << "json error!" << jsonerror.errorString();
        return;
    }

    if (!doc.isNull() && jsonerror.error == QJsonParseError::NoError) {
        if (doc.isObject()) {
            QJsonObject object = doc.object();
            QJsonObject::iterator it = object.begin();
            // read the dimesion of the variables
            while (it != object.end()) {
                if (it.key() == "192.168.137.170") {
                    QJsonObject it_object = it.value().toObject();
                    QJsonObject::iterator iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(0) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.171") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(1) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.172") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(2) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.173") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(3) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.174") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(4) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.175") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(5) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.150") {
                    QJsonObject it_object = it.value().toObject();
                    QJsonObject::iterator iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(6) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.151") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(7) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.152") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(8) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.153") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(9) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.154") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(10) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                if (it.key() == "192.168.137.155") {
                    QJsonObject it_object = it.value().toObject();
                    auto iter = it_object.begin();
                    while (iter != it_object.end()) {
                        if (iter.key() == "radian") {
                            initpos(11) = iter.value().toDouble();
                        }
                        iter++;
                    }
                }
                // if(it.key() == "192.168.137.190"){
                //     QJsonObject it_object = it.value().toObject();
                //     auto iter = it_object.begin();
                //     while(iter!=it_object.end())
                //     {
                //         if(iter.key() == "radian"){
                //             initpos(12) = iter.value().toDouble();
                //         }
                //         iter++;
                //     }

                // }
                // if (it.key() == "192.168.137.191") {
                //     QJsonObject it_object = it.value().toObject();
                //     auto iter = it_object.begin();
                //     while (iter != it_object.end()) {
                //         if (iter.key() == "radian") {
                //             initpos(13) = iter.value().toDouble();
                //         }
                //         iter++;
                //     }
                // }
                // if (it.key() == "192.168.137.192") {
                //     QJsonObject it_object = it.value().toObject();
                //     auto iter = it_object.begin();
                //     while (iter != it_object.end()) {
                //         if (iter.key() == "radian") {
                //             initpos(14) = iter.value().toDouble();
                //         }
                //         iter++;
                //     }
                // }
                it++;
            }
        }
    }
}