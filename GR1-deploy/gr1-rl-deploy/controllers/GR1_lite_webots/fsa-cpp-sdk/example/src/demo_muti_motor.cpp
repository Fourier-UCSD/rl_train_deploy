#include "FsaMutiMotor.h"
#include <fstream>
int main() {
    string ip1 = "192.168.137.181";
    string ip2 = "192.168.137.163";
    std::vector<std::string> ip_list;
    ip_list.push_back(ip1);
    ip_list.push_back(ip2);
    // string ip2 = "10.10.10.13";
    // string ip3 = "10.10.10.11";
    // string ip4 = "10.10.10.12";
    // string ip5 = "10.10.10.14";
    // string ip6 = "10.10.10.9";
    // FSA_CONNECT::LOG::LOG mylog;
    // std::ofstream setposition_txt;
    // setposition_txt.open("setposition_list.txt", std::ios::out);
    // std::ofstream getposition_txt;
    // getposition_txt.open("getposition_list_vel_ff.txt", std::ios::out);
    // std::ofstream velocity_ff_txt;
    // velocity_ff_txt.open("velocity_ff_txt.txt", std::ios::out);
    // mylog.setLevel(spdlog::level::level_enum::info);
    // FSA_CONNECT::FSAConfig::FSAControlConfig motor1;
    // FSA_CONNECT::FSAConfig::FSAPIDParams pid1;
    // std::vector<FSA_CONNECT::FSAConfig::FSAPIDParams> pid1_list;
    // pid1_list.push_back(pid1);
    // motor1.actuator_type = FSA_CONNECT::Status::TYPE_60_ACB_120;
    // motor1.actuator_reduction_ratio = 7;
    // motor1.motor_index = 1;
    // motor1.motor_vbus = 36;
    // motor1.motor_direction = FSA_CONNECT::Status::ACB;
    // motor1.motor_pole_pairs = 21;
    // motor1.motor_max_speed = 3000;

    // motor1.encoder_direction = 1;
    // motor1.encoder_resolution = 4000;
    // motor1.encoder_phase_offset = 0;
    // std::vector<double> setposition_list;
    // std::vector<double> getposition_list;
    FSA_CONNECT::FSAMutiMotor fsa1(2);

    fsa1.init(ip_list);
    fsa1.Enable();
    // fsa1.GetPIDParams(pid1_list);
    // pid1_list[0].control_position_kp = 0.5;
    // fsa1.SetPIDParams(pid1_list);
    // fsa1.GetPIDParams(pid1_list);
    // FSA_CONNECT::FSA fsa2;
    // fsa2.init(ip2);
    // FSA_CONNECT::FSA fsa3;
    // fsa3.init(ip3);
    // FSA_CONNECT::FSA fsa4;
    // fsa4.init(ip4);
    // FSA_CONNECT::FSA fsa5;
    // fsa5.init(ip5);
    // FSA_CONNECT::FSA fsa6;
    // fsa6.init(ip6);
    // fsa1.Enable();
    sleep(2);
    fsa1.Enable();
    sleep(2);
    // fsa2.Enable();
    // fsa3.Enable();
    // fsa4.Enable();
    // fsa5.Enable();
    // fsa6.Enable();
    // fi_fsa.Disable();
    // fi_fsa.GetControlConfig();
    // fi_fsa.GetRootConfig();
    // fsa1.EnableVelControl();
    fsa1.EnablePosControl();
    // sleep(2);
    // fsa1.EnableVelControl();
    // fsa3.EnableVelControl();
    // fsa4.EnableVelControl();
    // fsa5.EnableVelControl();
    // fsa6.EnableVelControl();
    double pos1;
    double vel1;
    double cur1;

    long long res2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    double max_t = 1;
    double dt = 0.002;
    double dt_count = round(10 / dt);
    Eigen::VectorXd pos_list(2);
    pos_list.setZero();
    Eigen::VectorXd vel_list(2);
    vel_list.setZero();
    Eigen::VectorXd cur_list(2);
    cur_list.setZero();
    fsa1.SetPosition(pos_list, vel_list, cur_list);
    sleep(1);
    fsa1.SetPosition(pos_list, vel_list, cur_list);
    // for (int i = 0; i < 500; i++) {
    //     long long res = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    //     std::cout << "start: " << res << std::endl;
    //     // std::cout << "duration: " << res - res2 << std::endl;
    //     // fsa1.GetPVC(pos1, vel1, cur1);

    //     // fsa2.GetPVC(pos2, vel2, cur2);
    //     // fsa3.GetPVC(pos3, vel3, cur3);
    //     // fsa4.GetPVC(pos1, vel1, cur1);
    //     // fsa5.GetPVC(pos2, vel2, cur2);
    //     // fsa6.GetPVC(pos3, vel3, cur3);
    //     double set_position = 90 * cos(2 * M_PI / max_t * double(i) * dt) - 90;

    //     pos_list(0) = set_position;
    //     pos_list(1) = set_position;
    //     // double set_velocity = -2 * M_PI / max_t * 90 * sin(2 * M_PI / max_t * double(i) * dt);
    //     fsa1.SetPosition(pos_list, vel_list, cur_list);
    //     std::cout << "aaaaaaaa: " << i << std::endl;
    //     // setposition_txt << set_position << std::endl;
    //     // getposition_txt << pos1 << std::endl;
    //     // velocity_ff_txt << set_velocity << std::endl;
    //     // fsa1.SetVelocity(set_velocity, 0);
    //     // fsa3.SetVelocity(2, 0);
    //     // fsa4.SetVelocity(2, 0);

    //     // fsa5.SetVelocity(2, 0);
    //     // fsa6.SetVelocity(2, 0);
    //     // long long res1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    //     // std::cout << "end: " << res1 - res << std::endl;
    //     // std::cout << "pos: " << pos1 << "vel: " << vel1 << "cur: " << cur1 << std::endl;

    //     std::this_thread::sleep_for(std::chrono::milliseconds(2));

    //     // res2 = res;
    // }

    return 0;
}