#ifndef STATEESTIMATE
#define STATEESTIMATE

#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include "AccKalmanFilter.h"
#include "LeggedKalmanFilter.h"
#include "aeroWalkPlan.h"
#include "../../RobotController/include/LowPassFilter.h"
#include "../../RobotController/include/KalmanFilter.h"
#include "../../RobotController/include/Robot_Data.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class StateEstimate
{
public:
    StateEstimate();
    bool estWaistPosVelInWorld(Robot_Data *robotdata, int FootType);
    bool estWaistPosVelInWorld(Robot_Data *robotdata);
    bool grfEstimating(Robot_Data *robotdata);
    bool stateMachine(Robot_Data *robotdata);
    bool rlStateMachine(Robot_Data *robotdata);

private:
    LowPassFilter *lowpass;
    AccKalmanFilter *accKalman;
    LeggedKalmanFilter *leggedKalman;

    double left_foot_velocity_last;
    double right_foot_velocity_last;
};

#endif // STATEESTIMATE