
#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h> 
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <chrono>
extern "C"
{
	#include "ch_serial.h"
}

namespace ch108
{


class ch108IMU
{
    public:

        void initIMU();
        void dump_data_packet(raw_t *raw);
        int open_port(char *port_device);
        Eigen::VectorXd imudata {9};
        inline Eigen::Vector3d rotm2zyx(Eigen::Matrix3d zyx)
        {
            Eigen::Vector3d rpy;
            rpy(1) = asin(zyx(0,2));
            if (abs(rpy(1) - M_PI/2) < 1.0e-3) 
            {
                rpy(0) = 0.0;
                rpy(2) = atan2(zyx(1,2) - zyx(0,1), zyx(0,2) + zyx(1,1)) + rpy(0);
            }

            else if (abs(rpy(1) + M_PI/2) < 1.0e-3) 
            {
                rpy(0) = 0.0;
                rpy(2)= atan2(zyx(1,2) - zyx(0,1), zyx(0,2) + zyx(1,1)) - rpy(0);
            }

            else 
            {
                rpy(2) = atan2(-zyx(1,2), zyx(2,2));
                rpy(0)= atan2(-zyx(0,1), zyx(0,0));
            }
            return rpy;
        };



        void thread_imudata( int arg);
        std::thread readdata;

        
};

}
