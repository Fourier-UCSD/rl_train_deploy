#include "Fsa.h"
#include "realtime_loop.h"
#include <fstream>
int main() {
    int num = 15;
    std::vector<std::string> ip_list(num);
    // ip_list[0] = "192.168.137.181";
    ip_list[0] = "10.10.10.90";
    ip_list[1] = "10.10.10.91";
    ip_list[2] = "10.10.10.92";
    ip_list[3] = "10.10.10.50";
    ip_list[4] = "10.10.10.51";
    ip_list[5] = "10.10.10.52";
    ip_list[6] = "10.10.10.53";
    ip_list[7] = "10.10.10.54";
    ip_list[8] = "10.10.10.55";
    ip_list[9] = "10.10.10.70";
    ip_list[10] = "10.10.10.71";
    ip_list[11] = "10.10.10.72";
    ip_list[12] = "10.10.10.73";
    ip_list[13] = "10.10.10.74";
    ip_list[14] = "10.10.10.75";

    // FSA_CONNECT::LOG::LOG mylog;

    std::vector<FSA_CONNECT::FSA> fsa1(num);

    int ret = 0;
    for (int i = 0; i < num; i++) {
        fsa1[i].init(ip_list[i]);
        ret = fsa1[i].Enable();
        if (ret < 0) {
            std::cout << "wrong" << std::endl;
            exit(0);
        }
    }

    sleep(1);

    for (int i = 0; i < num; i++) {
        fsa1[i].EnableVelControl();
    }

    sleep(1);
    // fsa1[0].EnableVelControl();
    // fsa3.EnableVelControl();
    // fsa4.EnableVelControl();
    // fsa5.EnableVelControl();
    // fsa6.EnableVelControl();

    for (int k = 0; k < 2000; k++) {

        rt_info.period_ns = cycle_time_s * 1e9;

        clock_gettime(CLOCK_MONOTONIC, &rt_info.next_period);

        for (int i = 0; i < num; i++) {
            fsa1[i].SetVelocity(100, 0);
        }

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

    for (int i = 0; i < num; i++) {
        fsa1[i].Disable();
    }
    return 0;
}