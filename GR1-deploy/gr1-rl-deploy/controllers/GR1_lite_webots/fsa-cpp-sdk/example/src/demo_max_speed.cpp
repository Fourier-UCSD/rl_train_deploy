#include "Fsa.h"
#include "realtime_loop.h"
#include <fstream>
int main() {
    string ip1 = "192.168.137.181";

    // string ip2 = "10.10.10.13";
    // string ip3 = "10.10.10.11";
    // string ip4 = "10.10.10.12";
    // string ip5 = "10.10.10.14";
    // string ip6 = "10.10.10.9";
    // FSA_CONNECT::LOG::LOG mylog;
    std::ofstream setvelocity_txt;
    std::vector<double> setvelocity;
    setvelocity_txt.open("setvelocity_list.txt", std::ios::out);
    std::ofstream getcurrent_txt;
    std::vector<double> getcurrent;
    getcurrent_txt.open("getcurrent_list.txt", std::ios::out);

    FSA_CONNECT::FSA fsa1;
    fsa1.init(ip1);
    fsa1.Enable();
    sleep(1);
    fsa1.EnableVelControl();
    sleep(1);
    // fsa1.EnableVelControl();
    // fsa3.EnableVelControl();
    // fsa4.EnableVelControl();
    // fsa5.EnableVelControl();
    // fsa6.EnableVelControl();
    double pos1 = 0;
    double vel1;
    double cur1;

    double max_t = 1;
    double dt = 0.0025;
    double dt_count = round(10 / dt);
    for (int i = 0; i < 1000; i++) {

        rt_info.period_ns = cycle_time_s * 1e9;

        clock_gettime(CLOCK_MONOTONIC, &rt_info.next_period);

        fsa1.GetPVC(pos1, vel1, cur1);
        fsa1.SetVelocity(0, 0);
        setvelocity_txt << vel1 << std::endl;
        getcurrent_txt << cur1 << std::endl;
        setvelocity.push_back(vel1);
        setvelocity.push_back(cur1);
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
    }

    for (int i = 0; i < 4000; i++) {

        long long res = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        rt_info.period_ns = cycle_time_s * 1e9;

        clock_gettime(CLOCK_MONOTONIC, &rt_info.next_period);

        fsa1.GetPVC(pos1, vel1, cur1);
        fsa1.SetVelocity(100, 0);
        setvelocity_txt << vel1 << std::endl;
        getcurrent_txt << cur1 << std::endl;
        setvelocity.push_back(vel1);
        setvelocity.push_back(cur1);
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
    }

    for (int i = 0; i < 1000; i++) {

        long long res = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::cout << "start: " << res << std::endl;
        rt_info.period_ns = cycle_time_s * 1e9;

        clock_gettime(CLOCK_MONOTONIC, &rt_info.next_period);

        fsa1.GetPVC(pos1, vel1, cur1);
        fsa1.SetVelocity(0, 0);
        setvelocity_txt << vel1 << std::endl;
        getcurrent_txt << cur1 << std::endl;
        setvelocity.push_back(vel1);
        setvelocity.push_back(cur1);
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
    }

    return 0;
}