#include "FsaMutiMotor.h"

namespace FSA_CONNECT {

void FSAMutiMotor::init(std::vector<string> &ip) {

    motornum = ip.size();
    muti_motor.resize(motornum);
    pos_set.resize(motornum);
    pos_set.setZero();
    vel_set.resize(motornum);
    vel_set.setZero();
    cur_set.resize(motornum);
    cur_set.setZero();

    pos_get.resize(motornum);
    pos_get.setZero();
    vel_get.resize(motornum);
    vel_get.setZero();
    cur_get.resize(motornum);
    cur_get.setZero();

    for (int i = 0; i < ip.size(); i++) {

        muti_motor[i] = std::make_shared<FSA_CONNECT::FSA>();
        muti_motor[i]->init(ip[i]);
    }
}

void FSAMutiMotor::Enable() {

    for (int i = 0; i < motornum; i++) {
        muti_motor[i]->Enable();
    }
}

void FSAMutiMotor::Disable() {

    for (int i = 0; i < motornum; i++) {
        muti_motor[i]->Disable();
    }
}

void FSAMutiMotor::SetPIDParams(std::vector<FSAConfig::FSAPIDParams> &pidparams) {
    for (int i = 0; i < motornum; i++) {
        muti_motor[i]->SetPIDParams(pidparams[i]);
    }
}

void FSAMutiMotor::GetPIDParams(std::vector<FSAConfig::FSAPIDParams> &pidparams) {
    for (int i = 0; i < motornum; i++) {
        muti_motor[i]->GetPIDParams(pidparams[i]);
    }
}

void FSAMutiMotor::EnablePosControl() {
    enable_pos_control = true;
    for (int i = 0; i < motornum; i++) {
        muti_motor[i]->EnablePosControl();
    }
    for (int i = 0; i < motornum; i++) {
        muti_motor[i]->GetPVC(pos_get[i], vel_get[i], cur_get[i]);
    }
}

void FSAMutiMotor::EnableVelControl() {
    enable_vel_control = true;
    for (int i = 0; i < motornum; i++) {
        muti_motor[i]->EnableVelControl();
    }
}

void FSAMutiMotor::EnableCurControl() {
    enable_vel_control = true;
    for (int i = 0; i < motornum; i++) {
        muti_motor[i]->EnableCurControl();
    }
}

void FSAMutiMotor::GetPVC(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &cur) {

    pos = pos_get;
    vel = vel_get;
    cur = cur_get;
}

void FSAMutiMotor::SetPosition(Eigen::VectorXd &pos, Eigen::VectorXd &vel_ff, Eigen::VectorXd &cur_ff) {
    using namespace FSA_CONNECT::JsonData;
    int ret = 0;
    for (int i = 0; i < motornum; i++) {
        i_now = i;
        sfd_now = muti_motor[i]->ctrl_udp_socket->sfd;
        JsonData::set_pos_json["position"] = pos[i];
        JsonData::set_pos_json["velocity_ff"] = vel_ff[i];
        JsonData::set_pos_json["current_ff"] = cur_ff[i];
        // if (err_count < 5) {
        ret = muti_motor[i]->ctrl_udp_socket->SendData(JsonData::set_pos_json.dump());

        if (ret < 0) {
            std::cout << "motor ip: " << muti_motor[i]->ip_ << ", udp send failed" << ret << std::endl;
            return;
        }
        pool.enqueue(&FSAMutiMotor::UpdateReceiveData, this, muti_motor[i]->ctrl_udp_socket->sfd, i);
        // } else {
        //     std::cout << "motor delay 1ms times over 5 times!" << std::endl;
        //     exit(0);
        //     // std::cout << " error "
        // }
        // socketFd_now = muti_motor[i]->ctrl_udp_socket->sfd;
        // ip_now = muti_motor[i]->ip_;

        // pool.enqueue([this] {
        //     this->UpdateReceiveData(sfd_now, i_now); // Example count value
        // });
    }
}

void FSAMutiMotor::SetVelocity(Eigen::VectorXd &vel, Eigen::VectorXd &cur_ff) {

    vel_set = vel;
    cur_set = cur_ff;
}

void FSAMutiMotor::SetCurrent(Eigen::VectorXd &cur) {

    cur_set = cur;
}

void FSAMutiMotor::UpdateReceiveData(int sfd_now, int i) {

    int MAX_RECV_SIZE = 1024;

    std::lock_guard<std::mutex> lock(dataMutex);
    struct sockaddr_in ser_addr_;
    std::string receive;
    char recv_data_[MAX_RECV_SIZE] = {'\0'};
    int ret;

    socklen_t addrlen = sizeof(ser_addr_);
    // res1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    ret = recvfrom(sfd_now, recv_data_, sizeof(recv_data_), 0, (struct sockaddr *)&ser_addr_, &addrlen);
    if (ret < 0) {
        err_count++;
        std::cout << "motor: " << i << "udp receive failed" << std::endl;

    } else {

        receive = std::string(recv_data_);
        // res = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // std::cout << "time: " << ret << " : " << std::this_thread::get_id() << ": " << i << std::endl;

        // receive nothing
        if (!receive.empty()) {
            recv_data_json = json::parse(receive);
            // a.clear();
            pos_get[i] = recv_data_json.at("position");
            // std::cout << "position: " << i << ": " << pos_get[i] << std::endl;

            vel_get[i] = recv_data_json.at("velocity");
            cur_get[i] = recv_data_json.at("current");
        }
    }

    // std::thread::id threadId = std::this_thread::get_id();
    // long long threadEndTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // std::cout << "time: " << threadEndTime << std::endl;

    // std::cout << "Thread ID11: " << threadId << std::endl;
    // return;
}

} // namespace FSA_CONNECT