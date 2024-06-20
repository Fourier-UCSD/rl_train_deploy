#ifndef VNIMU_h
#define VNIMU_h

#include <iostream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

class vnIMU
{
    private:
        const std::string SensorPort = "/dev/ttyUSB0";
        const uint32_t SensorBaudrate = 115200;
        const uint32_t SensorBaudrate2 = 921600;
        VnSensor vs;        
    public:
        bool initIMU();
        bool closeIMU();
        ~vnIMU();
        static Eigen::VectorXd imuData;
        static void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
        {
            if (p.type() == Packet::TYPE_ASCII && p.determineAsciiAsyncType() == VNYPR)
            {
                vec3f ypr;
                p.parseVNYPR(&ypr);
                cout << "ASCII Async YPR: " << ypr << endl;
                return;
            }

            if (p.type() == Packet::TYPE_BINARY)
            {
                // First make sure we have a binary packet type we expect since there
                // are many types of binary output types that can be configured.
                if (!p.isCompatible(
                    COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
                    TIMEGROUP_NONE,
                    IMUGROUP_NONE,
            GPSGROUP_NONE,
                    ATTITUDEGROUP_NONE,
                    INSGROUP_NONE,
            GPSGROUP_NONE))
            // Not the type of binary packet we are expecting.
                    return;

                // Ok, we have our expected binary output packet. Since there are many
                // ways to configure the binary data output, the burden is on the user
                // to correctly parse the binary packet. However, we can make use of
                // the parsing convenience methods provided by the Packet structure.
                // When using these convenience methods, you have to extract them in
                // the order they are organized in the binary packet per the User Manual.
                
                vec3f ypr = p.extractVec3f();
                vec3f dypr = p.extractVec3f();
                vec3f acc = p.extractVec3f();
                for (int i=0; i<3; ++i){
                    vnIMU::imuData(i) = (ypr[i]/180.0)*M_PI;
                    vnIMU::imuData(i+3) = dypr[i];
                    vnIMU::imuData(i+6) = acc[i];
                }

            }
        }; 
};

#endif
#pragma once