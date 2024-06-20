#include <thread>

struct period_info {
    struct timespec next_period;
    long period_ns;
};
static period_info rt_info;
double cycle_time_s = 0.0025;