#include "Fsa.h"
#include "realtime_loop.h"
#include <fstream>
int main() {
    string ip1 = "192.168.137.163";

    // string ip2 = "10.10.10.13";
    // string ip3 = "10.10.10.11";
    // string ip4 = "10.10.10.12";
    // string ip5 = "10.10.10.14";
    // string ip6 = "10.10.10.9";
    // FSA_CONNECT::LOG::LOG mylog;
    std::ofstream getvelocity_txt;
    std::vector<double> getvelocity;
    getvelocity_txt.open("getvelocity_list.txt", std::ios::out);
    std::ofstream getposition_txt;
    std::vector<double> getposition;
    getposition_txt.open("getposition_list.txt", std::ios::out);
    std::ofstream getcur_txt;
    std::vector<double> getcurrent;
    getcur_txt.open("getcurrent_list.txt", std::ios::out);

    FSA_CONNECT::FSA fsa1;
    fsa1.init(ip1);
    fsa1.Enable();
    sleep(1);
    fsa1.EnablePosControl();
    sleep(1);
    // fsa1.EnableVelControl();
    // fsa3.EnableVelControl();
    // fsa4.EnableVelControl();
    // fsa5.EnableVelControl();
    // fsa6.EnableVelControl();
    double pos1 = 0;
    double vel1;
    double cur1;

    long long res2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    double max_t = 1;
    double dt = 0.0025;
    double dt_count = round(10 / dt);
    for (int t = 0; t < 5; t++) {
        for (int i = 0; i < 400; i++) {
            long long res = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::cout << "start: " << res << std::endl;
            rt_info.period_ns = cycle_time_s * 1e9;

            clock_gettime(CLOCK_MONOTONIC, &rt_info.next_period);
            // std::cout << "duration: " << res - res2 << std::endl;
            fsa1.GetPVC(pos1, vel1, cur1);

            // fsa2.GetPVC(pos2, vel2, cur2);
            // fsa3.GetPVC(pos3, vel3, cur3);
            // fsa4.GetPVC(pos1, vel1, cur1);
            // fsa5.GetPVC(pos2, vel2, cur2);
            // fsa6.GetPVC(pos3, vel3, cur3);
            double set_position = 90 * cos(2 * M_PI / max_t * double(i) * dt) - 90;
            double set_velocity = -2 * M_PI / max_t * 90 * sin(2 * M_PI / max_t * double(i) * dt);
            fsa1.SetPosition(set_position, set_velocity, 0);

            getposition_txt << pos1 << std::endl;
            getvelocity_txt << vel1 << std::endl;
            getcur_txt << cur1 << std::endl;

            // getposition_vel_ff.push_back(abs(pos1));
            // fsa1.SetVelocity(set_velocity, 0);
            // fsa3.SetVelocity(2, 0);
            // fsa4.SetVelocity(2, 0);

            // fsa5.SetVelocity(2, 0);
            // fsa6.SetVelocity(2, 0);
            // long long res1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            // std::cout << "end: " << res1 - res << std::endl;
            // std::cout << "pos: " << pos1 << "vel: " << vel1 << "cur: " << cur1 << std::endl;

            // usleep(2000);

            rt_info.next_period.tv_nsec += rt_info.period_ns;

            while (rt_info.next_period.tv_nsec >= 1000000000) {
                /* timespec nsec overflow */
                rt_info.next_period.tv_sec++;
                rt_info.next_period.tv_nsec -= 1000000000;
            }
            // last_time = start_time;

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &rt_info.next_period, NULL);

            // res2 = res;
        }
    }

    fsa1.Disable();
    return 0;
}