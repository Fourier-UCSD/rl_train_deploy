#include "../include/MotorList.h"
#include <chrono>
#include <iostream>
#include <pthread.h>
#include <thread>

// using json = nlohmann::json;
// MotorList::MotorList(int allnum) {}

// MotorList::~MotorList() {}

void MotorList::init(int num, double dt, Eigen::VectorXd &absolute_pos, std::vector<std::string> ip, QString path) {
    motornum = num;
    this->dt = dt;
    // size init
    ip_list.resize(motornum);
    poscmd.resize(motornum);
    velcmd.resize(motornum);
    torcmd.resize(motornum);
    // motorIp.resize(motornum);
    // sortedInd.resize(motornum);
    // sortedIpVec.resize(motornum);
    absolute_pos_.resize(motornum);
    absolute_pos_zero_.resize(motornum);
    absolute_pos_dir_.resize(motornum);
    absolute_pos_gear_ratio_.resize(motornum);
    motor_gear_ratio_.resize(motornum);
    motorDir_.resize(motornum);
    c_t_scale_.resize(motornum);
    linearCount_.resize(motornum);
    controlConfig_.resize(motornum);
    enable_status_.resize(motornum);
    // controlCommand_.resize(motornum);
    kalman_joint_v_.resize(motornum);
    d_sysnoise_.resize(motornum);
    d_meannoise_.resize(motornum);
    lowpass_.resize(motornum);
    stribeck_para_.resize(motornum, 5);
    // frictionnlp_.resize(motornum);
    v_lb_.resize(motornum);
    v_ub_.resize(motornum);
    // data
    qa.resize(motornum);
    qda.resize(motornum);
    qda_lowpass.resize(motornum);
    qda_kalman.resize(motornum);
    currenta.resize(motornum);
    qtora.resize(motornum);
    fricitonnlp_tor = Eigen::VectorXd::Zero(motornum);
    frictionstribeck_tor = Eigen::VectorXd::Zero(motornum);
    // init value
    ip_list = ip;

    // motor.resize(motornum);

    // for (int i = 0; i < motornum; i++) {

    //     motor[i].init(ip_list[i]);
    // }

    motor.init(ip_list);

    // motorIp.setZero();
    // sortedInd.setZero();
    // sortedIpVec.setZero();
    for (int i = 0; i < motornum; i++) {
        motorid.insert(std::pair<std::string, int>(ip_list[i], i));
    }
    //------------- frictionNLP register ------------------------//
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.50", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.51", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.52", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.53", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.54", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.55", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.70", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.71", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.72", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.73", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.74", frictionFF_0));
    frictionnlp_.insert(std::pair<std::string, frictionNLP>("10.10.10.75", frictionFF_0));
    // todo
    //-----------------------------------------------------------//

    absolute_pos_.setZero();
    absolute_pos_zero_.setZero();
    absolute_pos_dir_.setZero();
    absolute_pos_gear_ratio_.setZero();
    for (int i = 0; i < motornum; i++) {
        motor_gear_ratio_[i] = 1.0;
        motorDir_[i] = 1.0;
        c_t_scale_[i] = 0.0;
        linearCount_[i] = 0.0;
        // controlConfig_[i] = new MotionControllerConfig();
        // enable motor
        enable_status_[i] = 1;
        // control copmmand init
        // controlCommand_[i].pos = 0.0;
        // controlCommand_[i].vel = 0.0;
        // controlCommand_[i].torque = 0.0;
        // kalman_joint_v
        // lowpass
        // frictionnlp
    }
    d_sysnoise_.setZero();
    d_meannoise_.setZero();
    stribeck_para_.setZero();
    v_lb_.setZero();
    v_ub_.setZero();

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
            for (int i = 0; i < motornum; i++) {
                bool findflag = false;
                QJsonObject::iterator it = object.begin();
                // read the dimesion of the variables
                while (it != object.end()) {
                    if (it.key() == QString::fromStdString(ip_list[i])) {
                        findflag = true;
                        QJsonObject it_object = it.value().toObject();
                        QJsonObject::iterator iter = it_object.begin();
                        while (iter != it_object.end()) {
                            if (iter.key() == "absolute_pos_zero") {
                                absolute_pos_zero_(i) = iter.value().toDouble();
                            }
                            if (iter.key() == "absolute_pos_dir") {
                                absolute_pos_dir_(i) = iter.value().toDouble();
                            }
                            if (iter.key() == "absolute_pos_gear_ratio") {
                                absolute_pos_gear_ratio_(i) = iter.value().toDouble();
                            }
                            if (iter.key() == "motor_gear_ratio") {
                                motor_gear_ratio_[i] = iter.value().toDouble();
                            }
                            if (iter.key() == "motorDir") {
                                motorDir_[i] = iter.value().toDouble();
                            }
                            if (iter.key() == "c_t_scale") {
                                c_t_scale_[i] = iter.value().toDouble();
                            }
                            if (iter.key() == "controlConfig") {
                                QJsonObject iter_object = iter.value().toObject();
                                QJsonObject::iterator config_it = iter_object.begin();
                                while (config_it != iter_object.end()) {
                                    if (config_it.key() == "pos_gain") {
                                        controlConfig_[i].control_position_kp = config_it.value().toDouble();
                                    }
                                    if (config_it.key() == "vel_gain") {
                                        controlConfig_[i].control_velocity_kp = config_it.value().toDouble();
                                    }
                                    // if (config_it.key() == "vel_integrator_gain") {
                                    //     controlConfig_[i].control_velocity_ki = config_it.value().toDouble();
                                    // }
                                    // if (config_it.key() == "vel_limit") {
                                    //     controlConfig_[i].control_position_output_max = config_it.value().toDouble();
                                    // }
                                    // if (config_it.key() == "vel_limit_tolerance") {
                                    //     controlConfig_[i].control_velocity_output_max = config_it.value().toDouble();
                                    // }
                                    config_it++;
                                }
                            }
                            if (iter.key() == "d_sysnoise") {
                                d_sysnoise_(i) = iter.value().toDouble();
                            }
                            if (iter.key() == "d_meannoise") {
                                d_meannoise_(i) = iter.value().toDouble();
                            }
                            if (iter.key() == "stribeck_para") {
                                stribeck_para_(i, 0) = iter.value().toArray().at(0).toDouble();
                                stribeck_para_(i, 1) = iter.value().toArray().at(1).toDouble();
                                stribeck_para_(i, 2) = iter.value().toArray().at(2).toDouble();
                                stribeck_para_(i, 3) = iter.value().toArray().at(3).toDouble();
                                stribeck_para_(i, 4) = iter.value().toArray().at(4).toDouble();
                            }
                            if (iter.key() == "v_lb") {
                                v_lb_(i) = iter.value().toDouble();
                            }
                            if (iter.key() == "v_ub") {
                                v_ub_(i) = iter.value().toDouble();
                            }
                            if (iter.key() == "frictionnlp") {
                                // todo
                            }
                            iter++;
                        }
                    }
                    it++;
                }
                if (findflag == false) {
                    std::cout << "ip: " << ip_list[i] << " not found!" << std::endl;
                }
            }
        }
    }
    // print read res
    for (int i = 0; i < motornum; i++) {
        std::cout << "ip_list: " << ip_list[i] << std::endl;
        // todo
    }
    for (auto iter = motorid.begin(); iter != motorid.end(); iter++) {
        std::cout << "ip_map: " << iter->first << " value: " << iter->second << std::endl;
    }
    // abs set
    {
        absolute_pos_ = absolute_pos - absolute_pos_zero_;

        for (int i = 0; i < motornum; i++) {
            if (absolute_pos_(i) < -M_PI) {
                absolute_pos_(i) = absolute_pos_(i) + 2.0 * M_PI;
            } else if (absolute_pos_(i) > M_PI) {

                absolute_pos_(i) = absolute_pos_(i) - 2.0 * M_PI;
            }

            absolute_pos_(i) = absolute_pos_(i) / (absolute_pos_dir_(i) * absolute_pos_gear_ratio_(i));
        }
        // std::cout<<"absolute_pos: "<<absolute_pos.transpose()<<std::endl;
        // std::cout<<"absolute_pos_zero_: "<<absolute_pos_zero_.transpose()<<std::endl;
        // std::cout<<"absolute_pos_: "<<absolute_pos_.transpose()<<std::endl;
        // std::cout<<"hehe"<<std::endl;
        // s2p

        // functionIKID_S2P *funS2P = new functionIKID_S2P();
        // // Eigen::VectorXd parallelQEst = Eigen::VectorXd::Zero(4);
        // // Eigen::VectorXd parallelQDotEst = Eigen::VectorXd::Zero(4);
        // // Eigen::VectorXd parallelQTorEst = Eigen::VectorXd::Zero(4);
        // Eigen::VectorXd parallelQCmd = Eigen::VectorXd::Zero(4);
        // Eigen::VectorXd parallelQDotCmd = Eigen::VectorXd::Zero(4);
        // Eigen::VectorXd parallelQTorCmd = Eigen::VectorXd::Zero(4);
        // // Eigen::VectorXd ankleOrienEst = Eigen::VectorXd::Zero(4);
        // // Eigen::VectorXd ankleOmegaEst = Eigen::VectorXd::Zero(4);
        // // Eigen::VectorXd ankleTorEst = Eigen::VectorXd::Zero(4);
        // Eigen::VectorXd ankleOrienRef = Eigen::VectorXd::Zero(4);
        // Eigen::VectorXd ankleOmegaRef = Eigen::VectorXd::Zero(4);
        // Eigen::VectorXd ankleTorDes = Eigen::VectorXd::Zero(4);
        // ankleOrienRef(0) = absolute_pos_(4);
        // ankleOrienRef(1) = absolute_pos_(5);
        // ankleOrienRef(2) = absolute_pos_(10);
        // ankleOrienRef(3) = absolute_pos_(11);
        // funS2P->setDes(ankleOrienRef, ankleOmegaRef);
        // funS2P->calcJointPosRef();
        // funS2P->setDesTorque(ankleTorDes);
        // funS2P->calcJointTorDes();
        // funS2P->getDes(parallelQCmd, parallelQDotCmd, parallelQTorCmd);
        // absolute_pos_(4) = parallelQCmd(0);
        // absolute_pos_(5) = parallelQCmd(1);
        // absolute_pos_(10) = parallelQCmd(2);
        // absolute_pos_(11) = parallelQCmd(3);

        // std::cout<<"hehe"<<std::endl;
    }
    assert(absolute_pos.size() == motornum);
    // output abs
    absolute_pos = absolute_pos_;
    // communication error
    loseerror_count.resize(motornum);
    for (int i = 0; i < motornum; i++) {
        loseerror_count[i] = 0;
    }
    tolerance_count = 1;
    //
    // int a;
    // std::cout<<" confirm the motor para, press 1 and enter to confirm: "<<std::endl;
    // std::cin>>a;
    // std::cout<<"confirm!"<<std::endl;

    // init motor
    // from iplist to motorip
    // todo
    // Eigen::VectorXi sortedInd = Eigen::VectorXi::LinSpaced(motorIp.size(),0,motorIp.size()-1);
    // sort_vec(motorIp,sortedIpVec,sortedInd);
    // Try and get the requested group.
    // {
    //     std::string str("10.10.10.255");
    //     Fourier::Lookup lookup(&str);
    //     std::this_thread::sleep_for(std::chrono::seconds(1));

    //     lookup.setLookupFrequencyHz(0);
    //     group = lookup.getGroupFromFamily("Default");
    //     if (!group) {
    //         std::cout << "No group found!" << std::endl;
    //         return;
    //     }
    // }
    // assert(group->size() == motornum);
    // std::cout << "group size: " << group->size() << std::endl;
    // // set error level
    // fourierSetLogLevel("ERROR");
    //
    // feedback = new Fourier::GroupFeedback(group->size());
    //
    // group_command = new Fourier::GroupCommand(group->size());
    // // reboot
    // group_command->reboot(std::vector<bool>(group->size(),true));
    // group->sendCommand(*group_command);
    // std::this_thread::sleep_for(std::chrono::seconds(10));

    // set linear count
    // std::vector<float> linearCount(motornum);
    // for (int i = 0; i < motornum; i++) {
    //     linearCount_[i] = absolute_pos_(i) * motorDir_[i] * motor_gear_ratio_[i] / (2.0 * M_PI);
    // }

    // std::map<std::string, int>::iterator it;
    // int count = 0;
    // for (it = motorid.begin(); it != motorid.end(); it++) {
    //     linearCount[count] = linearCount_[it->second];
    //     count++;
    // }
    // std::cout<<"linearcount: "<<linearCount[0]<< " "<< linearCount[1]<< " "<<linearCount[2]<< " "<<linearCount[3]<< " "<<linearCount[4]<< " "<<linearCount[5]<<std::endl;
    // group_command->resetLinearCount(linearCount);
    // group->sendCommand(*group_command);

    // set motor config
    // std::vector<FSA_CONNECT::FSAConfig::FSAPIDParams> controlConfig(motornum);

    // count = 0;
    // for (it = motorid.begin(); it != motorid.end(); it++) {
    //     controlConfig[count] = controlConfig_[it->second];
    //     count++;
    // }
    // std::cout<<"linearcount: "<<linearCount[0]<< " "<< linearCount[1]<< " "<<linearCount[2]<< " "<<linearCount[3]<< " "<<linearCount[4]<< " "<<linearCount[5]<<std::endl;
    // group_command->resetLinearCount(linearCount);
    // group->sendCommand(*group_command);

    // set motor config
    // std::vector<MotionControllerConfig *> controlConfig(motornum);

    // count = 0;
    // for (it = motorid.begin(); it != motorid.end(); it++) {
    //     controlConfig[count] = controlConfig_[it->second];
    //     count++;
    // }
    // for(int i = 0;i<motornum;i++)
    // {
    //     controlConfig[i] = controlConfig_[sortedInd(i)];
    // }
    // group_command->setMotionCtrlConfig(controlConfig);
    // group->sendCommand(*group_command);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "log json succed" << std::endl;
    // get motor config
    // group->sendFeedbackRequest();
    // std::this_thread::sleep_for(std::chrono::seconds(3));
    // group->getNextFeedback(*feedback, 20);

    // count = 0;
    // for (it = motorid.begin(); it != motorid.end(); it++) {
    //     std::cout << "Actor [" << it->first << "] pos_gain = " << (*feedback)[count]->motion_ctrl_config.pos_gain << std::endl;
    //     std::cout << "Actor [" << it->first << "] vel_gain = " << (*feedback)[count]->motion_ctrl_config.vel_gain << std::endl;
    //     std::cout << "Actor [" << it->first << "] inter_gain = " << (*feedback)[count]->motion_ctrl_config.vel_integrator_gain << std::endl;
    //     std::cout << "Actor [" << it->first << "] vel_limit = " << (*feedback)[count]->motion_ctrl_config.vel_limit << std::endl;
    //     std::cout << "Actor [" << it->first << "] vel_limit_tolerance = " << (*feedback)[count]->motion_ctrl_config.vel_limit_tolerance << std::endl;
    //     std::cout << "Zeroposition: " << it->first << "---" << (*feedback)[count]->position * motorDir_[it->second] * 2.0 * M_PI / motor_gear_ratio_[it->second] << std::endl;
    //     count++;
    // }

    // for (int i = 0; i < feedback->size(); i++)
    // {
    //     std::cout<<"Actor ["<<sortedInd(i)<<"] pos_gain = "<<(*feedback)[i]->motion_ctrl_config.pos_gain<<std::endl;
    //     std::cout<<"Actor ["<<sortedInd(i)<<"] vel_gain = "<<(*feedback)[i]->motion_ctrl_config.vel_gain<<std::endl;
    //     std::cout<<"Actor ["<<sortedInd(i)<<"] inter_gain = "<<(*feedback)[i]->motion_ctrl_config.vel_integrator_gain<<std::endl;
    //     std::cout<<"Actor ["<<sortedInd(i)<<"] vel_limit = "<<(*feedback)[i]->motion_ctrl_config.vel_limit<<std::endl;
    //     std::cout<<"Actor ["<<sortedInd(i)<<"] vel_limit_tolerance = "<<(*feedback)[i]->motion_ctrl_config.vel_limit_tolerance<<std::endl;
    // }
    // start enabling Devieces
    // for (int i = 0; i < group->size(); ++i) {
    //     enable_status_[i] = 1;
    // }
    // group_command->enable(enable_status_);
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // group->sendCommand(*group_command);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // for (int i = 0; i < motornum; ++i) {
    //     motor[i].SetPIDParams(controlConfig_[i]);
    // }

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // std::cout << "set pid succeed" << std::endl;
    // for (int i = 0; i < motornum; ++i) {

    //     motor[i].Enable();
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // std::cout << "Enable actor succeed" << std::endl;

    // for (int i = 0; i < motornum; ++i) {
    //     motor[i].GetPIDParams(controlConfig_[i]);
    //     std::cout << "motor: " << i << "poskp: " << controlConfig_[i].control_position_kp << std::endl;
    //     std::cout << "motor: " << i << "velkp: " << controlConfig_[i].control_velocity_kp << std::endl;
    // }

    // for (int i = 0; i < motornum; ++i) {
    //     motor[i].EnablePosControl();
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // std::cout << "Enable actor pos control succeed" << std::endl;

    // for (int i = 0; i < motornum; ++i) {
    //     motor[i].SetPosition(0, 0, 0);
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    motor.SetPIDParams(controlConfig_);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "set pid succeed" << std::endl;

    motor.Enable();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "Enable actor succeed" << std::endl;

    motor.GetPIDParams(controlConfig_);

    for (int i = 0; i < motornum; ++i) {

        std::cout << "motor: " << i << "poskp: " << controlConfig_[i].control_position_kp << std::endl;
        std::cout << "motor: " << i << "velkp: " << controlConfig_[i].control_velocity_kp << std::endl;
    }

    motor.EnablePosControl();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "Enable actor pos control succeed" << std::endl;

    // init kalman filter
    for (int i = 0; i < motornum; i++) {
        kalman_joint_v_[i] = new KalmanFilter(d_sysnoise_(i), d_meannoise_(i), 1);
        lowpass_[i] = new LowPassFilter(10.0, 0.707, dt, 1);
    }
#ifdef MOTOR_DATA_LOG
    foutData.open("MotorList.txt", std::ios::out);
    dataL = Eigen::VectorXd::Zero(100);
#endif
}

void MotorList::setcommand(Eigen::VectorXd pos_cmd, Eigen::VectorXd vel_cmd, Eigen::VectorXd tor_cmd, int frictiontype, int filtertypeforfriciton) {
    //
    pos_cmd = pos_cmd - absolute_pos_;
    // friction compensation
    Eigen::VectorXd qd_;
    switch (filtertypeforfriciton) {
    case (0): {
        qd_ = vel_cmd;
        break;
    }
    case (1): {
        qd_ = qda_kalman;
        break;
    }
    case (2): {
        qd_ = qda_lowpass;
        break;
    }
    default: {
        qd_ = qda;
        std::cout << "no such filter!" << std::endl;
        break;
    }
    }
    // tor_cmd.setZero();
    switch (frictiontype) {
    case (0): {
        break;
    }
    case (1): {
        for (int i = 0; i < motornum; i++) {
            tor_cmd[i] += frictioncompensation_stribeck(stribeck_para_(i, 0), stribeck_para_(i, 1), stribeck_para_(i, 2), stribeck_para_(i, 3), stribeck_para_(i, 4), qd_(i), c_t_scale_[i], motor_gear_ratio_[i]) * c_t_scale_[i] * motor_gear_ratio_[i];
        }
        break;
    }
    case (2): {
        Eigen::VectorXd c = Eigen::VectorXd::Zero(motornum);
        double c_ = 0.0;
        double cdot = 0.0;
        double cddot = 0.0;
        for (int i = 0; i < motornum; i++) {
            quintic(v_lb_(i), v_ub_(i), 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, abs(qd_(i)), 1.0, c_, cdot, cddot);
            c(i) = c_;
        }
        // nlp friction tor
        Eigen::VectorXd co = Eigen::VectorXd::Zero(12);
        co << 0.4, 0.5, 1.0, 1.25, 0.8, 0.8, 0.4, 0.5, 0.9, 0.9, 0.8, 0.8;
        fricitonnlp_tor.setZero();
        for (auto it = motorid.begin(); it != motorid.end(); it++) {
            fricitonnlp_tor(it->second) = co(it->second) * motorDir_[it->second] * frictionnlp_[it->first](motorDir_[it->second] * qd_(it->second), c_t_scale_[it->second], motor_gear_ratio_[it->second]) * c_t_scale_[it->second] * motor_gear_ratio_[it->second];
        }
        // stribeck friction
        frictionstribeck_tor.setZero();
        for (auto it = motorid.begin(); it != motorid.end(); it++) {
            frictionstribeck_tor(it->second) = motorDir_[it->second] * frictioncompensation_stribeck(stribeck_para_(it->second, 0), stribeck_para_(it->second, 1), stribeck_para_(it->second, 2), stribeck_para_(it->second, 3), stribeck_para_(it->second, 4), motorDir_[it->second] * qd_(it->second), c_t_scale_[it->second], motor_gear_ratio_[it->second]) * c_t_scale_[it->second] * motor_gear_ratio_[it->second];
        }
        // add friction compensation
        for (int i = 0; i < motornum; i++) {
            c(i) = 1.0;
            tor_cmd[i] += c(i) * 1.2 * frictionstribeck_tor(i) + (1.0 - c(i)) * fricitonnlp_tor(i);
        }
        break;
    }
    default: {
        break;
    }
    }
    // end
    // std::map<std::string, int>::iterator it;
    // int count = 0;
    for (int it = 0; it < motornum; it++) {
        poscmd[it] = motorDir_[it] * pos_cmd[it] * 180 / M_PI;
        velcmd[it] = motorDir_[it] * vel_cmd[it] * 180 / M_PI;
        torcmd[it] = motorDir_[it] * tor_cmd[it] / (motor_gear_ratio_[it] * c_t_scale_[it]);
    }

    // for (int i = 0; i < motornum; i++) {
    //     motor[i].SetPosition(poscmd[i], velcmd[i], torcmd[i]);
    // }

    motor.SetPosition(poscmd, velcmd, torcmd);

    // for (it = motorid.begin(); it != motorid.end(); it++) {
    //     controlCommand_[count].pos = pos_cmd[it->second] * motorDir_[it->second] * motor_gear_ratio_[it->second] / (2.0 * M_PI);
    //     controlCommand_[count].vel = vel_cmd[it->second] * motorDir_[it->second] * motor_gear_ratio_[it->second] / (2.0 * M_PI);
    //     controlCommand_[count].torque = tor_cmd[it->second] * motorDir_[it->second] / (motor_gear_ratio_[it->second] * c_t_scale_[it->second]);
    //     count++;
    // }
    // group_command->setInputPositionPt(controlCommand_);
    // group->sendCommand(*group_command);
#ifdef MOTOR_DATA_LOG
    dataL.block(0, 0, motornum, 1) = pos_cmd;
    dataL.block(motornum, 0, motornum, 1) = vel_cmd;
    dataL.block(2.0 * motornum, 0, motornum, 1) = tor_cmd;
    dataL.block(3.0 * motornum, 0, motornum, 1) = qda_kalman;
    dataL.block(4.0 * motornum, 0, motornum, 1) = qda_lowpass;
    dataL.block(5.0 * motornum, 0, motornum, 1) = fricitonnlp_tor;
    dataL.block(6.0 * motornum, 0, motornum, 1) = frictionstribeck_tor;
    this->dataLog(dataL, foutData);
#endif
}
// vel_cmd_actual is friction compensation ref vel
void MotorList::setcommand(Eigen::VectorXd pos_cmd, Eigen::VectorXd vel_cmd, Eigen::VectorXd tor_cmd, int frictiontype, int filtertypeforfriciton, Eigen::VectorXd vel_cmd_actual) {
    //
    pos_cmd = pos_cmd - absolute_pos_;
    // friction compensation
    Eigen::VectorXd qd_;
    switch (filtertypeforfriciton) {
    case (0): {
        qd_ = vel_cmd_actual;
        break;
    }
    case (1): {
        qd_ = qda_kalman;
        break;
    }
    case (2): {
        qd_ = qda_lowpass;
        break;
    }
    default: {
        qd_ = qda;
        std::cout << "no such filter!" << std::endl;
        break;
    }
    }
    // tor_cmd.setZero();
    switch (frictiontype) {
    case (0): {
        break;
    }
    case (1): {
        for (int i = 0; i < motornum; i++) {
            tor_cmd[i] += frictioncompensation_stribeck(stribeck_para_(i, 0), stribeck_para_(i, 1), stribeck_para_(i, 2), stribeck_para_(i, 3), stribeck_para_(i, 4), qd_(i), c_t_scale_[i], motor_gear_ratio_[i]) * c_t_scale_[i] * motor_gear_ratio_[i];
        }
        break;
    }
    case (2): {
        Eigen::VectorXd c = Eigen::VectorXd::Zero(motornum);
        double c_ = 0.0;
        double cdot = 0.0;
        double cddot = 0.0;
        for (int i = 0; i < motornum; i++) {
            quintic(v_lb_(i), v_ub_(i), 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, abs(qd_(i)), 1.0, c_, cdot, cddot);
            c(i) = c_;
        }
        // nlp friction tor
        // Eigen::VectorXd co = Eigen::VectorXd::Zero(12);
        // co << 0.4, 0.5, 1.0, 1.25, 0.8,0.8,0.4, 0.5, 0.9, 0.9, 0.8,0.8;
        fricitonnlp_tor.setZero();
        // for(auto it = motorid.begin();it!=motorid.end();it++)
        // {
        //     fricitonnlp_tor(it->second) = co(it->second)*motorDir_[it->second]*frictionnlp_[it->first](motorDir_[it->second]*qd_(it->second),c_t_scale_[it->second],motor_gear_ratio_[it->second])*c_t_scale_[it->second]*motor_gear_ratio_[it->second];
        // }
        // stribeck friction
        frictionstribeck_tor.setZero();
        for (auto it = motorid.begin(); it != motorid.end(); it++) {
            frictionstribeck_tor(it->second) = motorDir_[it->second] * frictioncompensation_stribeck(stribeck_para_(it->second, 0), stribeck_para_(it->second, 1), stribeck_para_(it->second, 2), stribeck_para_(it->second, 3), stribeck_para_(it->second, 4), motorDir_[it->second] * qd_(it->second), c_t_scale_[it->second], motor_gear_ratio_[it->second]) * c_t_scale_[it->second] * motor_gear_ratio_[it->second];
        }
        Eigen::VectorXd c_ankle = Eigen::VectorXd::Zero(motornum);
        c_ankle << 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0;
        // add friction compensation
        for (int i = 0; i < motornum; i++) {
            // c(i) = 1.0;
            tor_cmd[i] += c_ankle(i) * (c(i) * 1.2 * frictionstribeck_tor(i) + (1.0 - c(i)) * fricitonnlp_tor(i));
        }
        break;
    }
    default: {
        break;
    }
    }
    // end

    for (int it = 0; it < motornum; it++) {
        poscmd[it] = motorDir_[it] * pos_cmd[it] * 180 / M_PI;
        velcmd[it] = motorDir_[it] * vel_cmd[it] * 180 / M_PI;
        torcmd[it] = motorDir_[it] * tor_cmd[it] / (motor_gear_ratio_[it] * c_t_scale_[it]);
    }
    // int count = 0;
    // for (int i = 0; i < motornum; i++) {
    //     motor[i].SetPosition(poscmd[i], velcmd[i], torcmd[i]);
    // }

    motor.SetPosition(poscmd, velcmd, torcmd);

    // for (it = motorid.begin(); it != motorid.end(); it++) {
    //     controlCommand_[count].pos = pos_cmd[it->second] * motorDir_[it->second] * motor_gear_ratio_[it->second] / (2.0 * M_PI);
    //     controlCommand_[count].vel = vel_cmd[it->second] * motorDir_[it->second] * motor_gear_ratio_[it->second] / (2.0 * M_PI);
    //     controlCommand_[count].torque = tor_cmd[it->second] * motorDir_[it->second] / (motor_gear_ratio_[it->second] * c_t_scale_[it->second]);
    //     count++;
    // }
    // group_command->setInputPositionPt(controlCommand_);
    // group->sendCommand(*group_command);
#ifdef MOTOR_DATA_LOG
    dataL.block(0, 0, motornum, 1) = pos_cmd;
    dataL.block(motornum, 0, motornum, 1) = vel_cmd;
    dataL.block(2.0 * motornum, 0, motornum, 1) = tor_cmd;
    // dataL.block(3.0*motornum,0,motornum,1) = qda_kalman;
    // dataL.block(4.0*motornum,0,motornum,1) = qda_lowpass;
    // dataL.block(5.0*motornum,0,motornum,1) = fricitonnlp_tor;
    // dataL.block(6.0*motornum,0,motornum,1) = frictionstribeck_tor;
    this->dataLog(dataL, foutData);
#endif
}

void MotorList::getstate(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &tor, int filtertype) {
    motor.GetPVC(qa, qda, qtora);

    for (int i = 0; i < motornum; i++) {
        qa[i] = motorDir_[i] * qa[i] * M_PI / 180;
        qda[i] = motorDir_[i] * qda[i] * M_PI / 180;
        qtora[i] = motorDir_[i] * qtora[i] * c_t_scale_[i] * motor_gear_ratio_[i];
    }
    // group->getNextFeedback(*feedback, 3);

    // std::map<std::string, int>::iterator it;
    // int count = 0;
    // for (it = motorid.begin(); it != motorid.end(); it++) {
    //     // lose comunication
    //     if ((*feedback)[count]->position == std::numeric_limits<float>::quiet_NaN()) {
    //         loseerror_count[it->second]++;
    //     } else {
    //         loseerror_count[it->second] = 0;
    //     }
    //     if (loseerror_count[it->second] == tolerance_count) {
    //         std::vector<float> status(motornum, 0);
    //         group_command->ctrlBoxEnable(status);
    //         group->sendCommand(*group_command);
    //         std::cout << "Joint [" << it->second << "] lose communication, ctrlBox is disabled !" << std::endl;
    //     }
    //     qa(it->second) = (*feedback)[count]->position * motorDir_[it->second] * 2.0 * M_PI / motor_gear_ratio_[it->second];
    //     // std::cout<<"qest jointid: "<< it->second << " datanumber: "<< count << " value:" << qa(it->second) << std::endl;
    //     qda(it->second) = (*feedback)[count]->velocity * motorDir_[it->second] * 2.0 * M_PI / motor_gear_ratio_[it->second];
    //     currenta(it->second) = (*feedback)[count]->current * motorDir_[it->second];
    //     qtora(it->second) = currenta(it->second) * c_t_scale_[it->second] * motor_gear_ratio_[it->second];
    //     count++;
    // }
    qa = qa + absolute_pos_;
    pos = qa;
    vel = qda;
    tor = qtora;
    // for(int i = 0; i<motornum;i++)
    // {
    //     pos(sortedInd(i)) = (*feedback)[i]->position*motorDir_[sortedInd(i)]*2*M_PI/motor_gear_ratio_[sortedInd(i)];
    //     vel(sortedInd(i)) = (*feedback)[i]->velocity*motorDir_[sortedInd(i)]*2*M_PI/motor_gear_ratio_[sortedInd(i)];
    //     tor(sortedInd(i)) = (*feedback)[i]->current*motorDir_[sortedInd(i)]*c_t_scale_[sortedInd(i)]*motor_gear_ratio_[sortedInd(i)];
    // }
    switch (filtertype) {
    case (0): {
        break;
    }
    case (1): {
        Eigen::VectorXd q(1);
        for (int i = 0; i < motornum; i++) {
            q(0) = qda(i);
            qda_kalman(i) = kalman_joint_v_[i]->mFilter(q)(0);
        }
        vel = qda_kalman;
        break;
    }
    case (2): {
        Eigen::VectorXd q(1);
        for (int i = 0; i < motornum; i++) {
            q(0) = qda(i);
            qda_lowpass(i) = lowpass_[i]->mFilter(q)(0);
        }
        vel = qda_lowpass;
        break;
    }
    default: {
        std::cout << "no such filter!" << std::endl;
        break;
    }
    }
}

void MotorList::settolerance_count(int _count) {
    tolerance_count = _count;
}

void MotorList::disable() {
    motor.Disable();
    // for (int i = 0; i < motornum; ++i) {
    //     motor[i].Disable();
    // }
    // for (int i = 0; i < group->size(); ++i) {
    //     enable_status_[i] = 0;
    // }
    // group_command->enable(enable_status_);
    // group->sendCommand(*group_command);
    // for (int i = 0; i < motornum; i++) {
    //     std::cout << "Actor [" << i << "] disabled!" << std::endl;
    // }
#ifdef MOTOR_DATA_LOG
    foutData.close();
#endif
}

double MotorList::frictioncompensation_stribeck(double a, double b, double s, double alpha, double v, double qdot, double ctscale, double gear_ratio) {
    return (a * qdot + b + s / (1 + exp(-alpha * (qdot + v)))) / (ctscale * gear_ratio);
}

bool MotorList::quintic(double x1, double x2, double y1, double y2, double dy1, double dy2, double ddy1, double ddy2, double x, double dx, double &y, double &dy, double &ddy) {
    // Limit range since curve fit is only valid within range
    x = clamp(x, x1, x2);

    // Declare variables
    double t, t1, t2, deltaY, deltaT, a0, a1, a2, a3, a4, a5;

    // variable substitution
    t = x / dx;
    t1 = x1 / dx;
    t2 = x2 / dx;
    // interpolate
    deltaY = y2 - y1;
    deltaT = t2 - t1;
    a0 = y1;
    a1 = dy1;
    a2 = 1.0 / 2. * ddy1;
    a3 = 1.0 / (2. * deltaT * deltaT * deltaT) * (20. * deltaY - (8. * dy2 + 12. * dy1) * deltaT + (ddy2 - 3. * ddy1) * (deltaT * deltaT));
    a4 = 1.0 / (2. * deltaT * deltaT * deltaT * deltaT) * (-30. * deltaY + (14. * dy2 + 16. * dy1) * deltaT + (3. * ddy1 - 2. * ddy2) * (deltaT * deltaT));
    a5 = 1.0 / (2. * deltaT * deltaT * deltaT * deltaT * deltaT) * (12. * deltaY - 6. * (dy2 + dy1) * deltaT + (ddy2 - ddy1) * (deltaT * deltaT));

    // position
    y = a0 + a1 * myPow((t - t1), 1) + a2 * myPow((t - t1), 2) + a3 * myPow((t - t1), 3) + a4 * myPow(t - t1, 4) + a5 * myPow(t - t1, 5);
    // velocity
    dy = a1 + 2. * a2 * myPow((t - t1), 1) + 3. * a3 * myPow((t - t1), 2) + 4. * a4 * myPow(t - t1, 3) + 5. * a5 * myPow(t - t1, 4);
    // acceleration
    ddy = 2. * a2 + 6. * a3 * myPow((t - t1), 1) + 12. * a4 * myPow(t - t1, 2) + 20. * a5 * myPow(t - t1, 3);

    return true;
}
double MotorList::clamp(double num, double lim1, double lim2) {
    auto min = std::min(lim1, lim2);
    auto max = std::max(lim1, lim2);

    if (num < min)
        return min;

    if (max < num)
        return max;

    return num;
}
double MotorList::myPow(double x, int n) {
    if (n == 0)
        return 1.0;
    if (n < 0)
        return 1.0 / myPow(x, -n);
    double half = myPow(x, n >> 1);

    if (n % 2 == 0)
        return half * half;
    else {
        return half * half * x;
    }
}
#ifdef MOTOR_DATA_LOG
// logData
bool MotorList::dataLog(Eigen::VectorXd &v, std::ofstream &f) {
    for (int i = 0; i < v.size(); i++) {
        f << v[i] << " ";
    }
    f << std::endl;
    return true;
}
#endif