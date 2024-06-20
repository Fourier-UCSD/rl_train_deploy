#include "FsaMutiMotor.h"
#include <fstream>
int main() {
    string ip1 = "192.168.137.181";
    string ip2 = "192.168.137.163";
    std::vector<std::string> ip_list;
    ip_list.push_back(ip1);
    ip_list.push_back(ip2);

    FSA_CONNECT::FSAMutiMotor fsa1(3);

    fsa1.init(ip_list);
    fsa1.Enable();

    sleep(2);
    // fsa2.Enable();

    fsa1.EnablePosControl();
    sleep(2);

    // while (1) {
    Eigen::VectorXd pos(2);
    Eigen::VectorXd vel(2);
    Eigen::VectorXd cur(2);

    Eigen::VectorXd pos_get(2);
    Eigen::VectorXd vel_get(2);
    Eigen::VectorXd cur_get(2);

    //     fsa1.SetPosition(pos, vel, cur);
    //     usleep(1000);
    // }

    double max_t = 3;
    double dt = 0.0025;
    double dt_count = round(10 / dt);
    while (1) {
        for (int i = 0; i < 1200; i++) {
            auto threadStartTime = std::chrono::high_resolution_clock::now();

            // std::cout << "duration: " << res - res2 << std::endl;
            // fsa1.GetPVC(pos1, vel1, cur1);

            // fsa2.GetPVC(pos2, vel2, cur2);
            // fsa3.GetPVC(pos3, vel3, cur3);
            // fsa4.GetPVC(pos1, vel1, cur1);
            // fsa5.GetPVC(pos2, vel2, cur2);
            // fsa6.GetPVC(pos3, vel3, cur3);
            double set_position = 90 * cos(2 * M_PI / max_t * double(i) * dt) - 90;

            pos(0) = set_position;
            pos(1) = set_position;
            // pos(1) = 0;
            // double set_velocity = -2 * M_PI / max_t * 90 * sin(2 * M_PI / max_t * double(i) * dt);
            // vel(1) = 50;
            fsa1.SetPosition(pos, vel, cur);

            fsa1.GetPVC(pos_get, vel_get, cur_get);

            // setposition_txt << set_position << std::endl;
            // getposition_txt << pos1 << std::endl;
            // velocity_ff_txt << set_velocity << std::endl;
            // fsa1.SetVelocity(set_velocity, 0);
            // fsa3.SetVelocity(2, 0);
            // fsa4.SetVelocity(2, 0);

            // fsa5.SetVelocity(2, 0);
            // fsa6.SetVelocity(2, 0);
            long long res1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            // std::cout << "end: " << res1 << std::endl;
            // std::cout << "pos: " << pos1 << "vel: " << vel1 << "cur: " << cur1 << std::endl;

            std::this_thread::sleep_for(std::chrono::microseconds(2500));
            std::thread::id threadId = std::this_thread::get_id();
            long long threadEndTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

            std::cout << "aa: " << pos_get << std::endl;
            // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(threadEndTime - threadStartTime);
            // std::cout << "main: " << pos_get << std::endl;
            // 将线程 ID 打印出来
            // std::cout << "Thread ID11: " << threadId << std::endl;

            // res2 = res;
        }
    }

    return 0;
}