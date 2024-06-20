#include "webotsInterface.h"

using namespace webots;

void WebotsRobot::initWebots()
{
    // // motors
    // legMotor.resize(nJoint);
    // legMotor[0] = robot->getMotor("hipRoll_Left");
    // legMotor[1] = robot->getMotor("hipYaw_Left");
    // legMotor[2] = robot->getMotor("hipPitch_Left");
    // legMotor[3] = robot->getMotor("kneePitch_Left");
    // legMotor[4] = robot->getMotor("anklePitch_Left");
    // legMotor[5] = robot->getMotor("ankleRoll_Left");
    // legMotor[6] = robot->getMotor("hipRoll_Right");
    // legMotor[7] = robot->getMotor("hipYaw_Right");
    // legMotor[8] = robot->getMotor("hipPitch_Right");
    // legMotor[9] = robot->getMotor("kneePitch_Right");
    // legMotor[10] = robot->getMotor("anklePitch_Right");
    // legMotor[11] = robot->getMotor("ankleRoll_Right");

    // legMotor[12] = robot->getMotor("waistYaw");
    // legMotor[13] = robot->getMotor("waistPitch");
    // legMotor[14] = robot->getMotor("waistRoll");

    // legMotor[15] = robot->getMotor("shoulderPitch_Left");
    // legMotor[16] = robot->getMotor("shoulderRoll_Left");
    // legMotor[17] = robot->getMotor("shoulderYaw_Left");
    // legMotor[18] = robot->getMotor("elbow_Left");
    // // legMotor[19] = robot->getMotor("wristYaw_Left");
    // // legMotor[20] = robot->getMotor("wristPitch_Left");
    // // legMotor[21] = robot->getMotor("wristRoll_Left");
    // legMotor[19] = robot->getMotor("shoulderPitch_Right");
    // legMotor[20] = robot->getMotor("shoulderRoll_Right");
    // legMotor[21] = robot->getMotor("shoulderYaw_Right");
    // legMotor[22] = robot->getMotor("elbow_Right");
    // // legMotor[26] = robot->getMotor("wristYaw_Right");
    // // legMotor[27] = robot->getMotor("wristPitch_Right");
    // // legMotor[28] = robot->getMotor("wristRoll_Right");

    // // legMotor[29] = robot->getMotor("headYaw");
    // // legMotor[30] = robot->getMotor("headRoll");
    // // legMotor[31] = robot->getMotor("headPitch");

    // // motor sensors
    // legSensor.resize(nJoint);
    // legSensor[0] = robot->getPositionSensor("hipRoll_Left_sensor");
    // legSensor[1] = robot->getPositionSensor("hipYaw_Left_sensor");
    // legSensor[2] = robot->getPositionSensor("hipPitch_Left_sensor");
    // legSensor[3] = robot->getPositionSensor("kneePitch_Left_sensor");
    // legSensor[4] = robot->getPositionSensor("anklePitch_Left_sensor");
    // legSensor[5] = robot->getPositionSensor("ankleRoll_Left_sensor");
    // legSensor[6] = robot->getPositionSensor("hipRoll_Right_sensor");
    // legSensor[7] = robot->getPositionSensor("hipYaw_Right_sensor");
    // legSensor[8] = robot->getPositionSensor("hipPitch_Right_sensor");
    // legSensor[9] = robot->getPositionSensor("kneePitch_Right_sensor");
    // legSensor[10] = robot->getPositionSensor("anklePitch_Right_sensor");
    // legSensor[11] = robot->getPositionSensor("ankleRoll_Right_sensor");

    // legSensor[12] = robot->getPositionSensor("waistYaw_sensor");
    // legSensor[13] = robot->getPositionSensor("waistPitch_sensor");
    // legSensor[14] = robot->getPositionSensor("waistRoll_sensor");

    // legSensor[15] = robot->getPositionSensor("shoulderPitch_Left_sensor");
    // legSensor[16] = robot->getPositionSensor("shoulderRoll_Left_sensor");
    // legSensor[17] = robot->getPositionSensor("shoulderYaw_Left_sensor");
    // legSensor[18] = robot->getPositionSensor("elbow_Left_sensor");
    // legSensor[19] = robot->getPositionSensor("shoulderPitch_Right_sensor");
    // legSensor[20] = robot->getPositionSensor("shoulderRoll_Right_sensor");
    // legSensor[21] = robot->getPositionSensor("shoulderYaw_Right_sensor");
    // legSensor[22] = robot->getPositionSensor("elbow_Right_sensor");
    // motors
    legMotor.resize(nJoint);
    legMotor[0] = robot->getMotor("l_hip_roll");
    legMotor[1] = robot->getMotor("l_hip_yaw");
    legMotor[2] = robot->getMotor("l_hip_pitch");
    legMotor[3] = robot->getMotor("l_knee_pitch");
    legMotor[4] = robot->getMotor("l_ankle_pitch");
    legMotor[5] = robot->getMotor("l_ankle_roll");
    legMotor[6] = robot->getMotor("r_hip_roll");
    legMotor[7] = robot->getMotor("r_hip_yaw");
    legMotor[8] = robot->getMotor("r_hip_pitch");
    legMotor[9] = robot->getMotor("r_knee_pitch");
    legMotor[10] = robot->getMotor("r_ankle_pitch");
    legMotor[11] = robot->getMotor("r_ankle_roll");

    legMotor[12] = robot->getMotor("waist_yaw");
    legMotor[13] = robot->getMotor("waist_pitch");
    legMotor[14] = robot->getMotor("waist_roll");

    legMotor[15] = robot->getMotor("l_shoulder_pitch");
    legMotor[16] = robot->getMotor("l_shoulder_roll");
    legMotor[17] = robot->getMotor("l_shoulder_yaw");
    legMotor[18] = robot->getMotor("l_elbow_pitch");

    legMotor[19] = robot->getMotor("r_shoulder_pitch");
    legMotor[20] = robot->getMotor("r_shoulder_roll");
    legMotor[21] = robot->getMotor("r_shoulder_yaw");
    legMotor[22] = robot->getMotor("r_elbow_pitch");

    legSensor.resize(nJoint);
    legSensor[0] = robot->getPositionSensor("l_hip_roll_sensor");
    legSensor[1] = robot->getPositionSensor("l_hip_yaw_sensor");
    legSensor[2] = robot->getPositionSensor("l_hip_pitch_sensor");
    legSensor[3] = robot->getPositionSensor("l_knee_pitch_sensor");
    legSensor[4] = robot->getPositionSensor("l_ankle_pitch_sensor");
    legSensor[5] = robot->getPositionSensor("l_ankle_roll_sensor");
    legSensor[6] = robot->getPositionSensor("r_hip_roll_sensor");
    legSensor[7] = robot->getPositionSensor("r_hip_yaw_sensor");
    legSensor[8] = robot->getPositionSensor("r_hip_pitch_sensor");
    legSensor[9] = robot->getPositionSensor("r_knee_pitch_sensor");
    legSensor[10] = robot->getPositionSensor("r_ankle_pitch_sensor");
    legSensor[11] = robot->getPositionSensor("r_ankle_roll_sensor");

    legSensor[12] = robot->getPositionSensor("waist_yaw_sensor");
    legSensor[13] = robot->getPositionSensor("waist_pitch_sensor");
    legSensor[14] = robot->getPositionSensor("waist_roll_sensor");

    legSensor[15] = robot->getPositionSensor("l_shoulder_pitch_sensor");
    legSensor[16] = robot->getPositionSensor("l_shoulder_roll_sensor");
    legSensor[17] = robot->getPositionSensor("l_shoulder_yaw_sensor");
    legSensor[18] = robot->getPositionSensor("l_elbow_pitch_sensor");

    legSensor[19] = robot->getPositionSensor("r_shoulder_pitch_sensor");
    legSensor[20] = robot->getPositionSensor("r_shoulder_roll_sensor");
    legSensor[21] = robot->getPositionSensor("r_shoulder_yaw_sensor");
    legSensor[22] = robot->getPositionSensor("r_elbow_pitch_sensor");

    // motor sensors
    // legSensor.resize(nJoint);
    // legSensor[0] = robot->getPositionSensor("LLeg1_sensor");
    // legSensor[1] = robot->getPositionSensor("LLeg2_sensor");
    // legSensor[2] = robot->getPositionSensor("LLeg3_sensor");
    // legSensor[3] = robot->getPositionSensor("LLeg4_sensor");
    // legSensor[4] = robot->getPositionSensor("LLeg5_sensor");
    // legSensor[5] = robot->getPositionSensor("LLeg6_sensor");
    // legSensor[6] = robot->getPositionSensor("RLeg1_sensor");
    // legSensor[7] = robot->getPositionSensor("RLeg2_sensor");
    // legSensor[8] = robot->getPositionSensor("RLeg3_sensor");
    // legSensor[9] = robot->getPositionSensor("RLeg4_sensor");
    // legSensor[10] = robot->getPositionSensor("RLeg5_sensor");
    // legSensor[11] = robot->getPositionSensor("RLeg6_sensor");

    // legSensor[12] = robot->getPositionSensor("waist3_sensor");
    // legSensor[13] = robot->getPositionSensor("waist2_sensor");
    // legSensor[14] = robot->getPositionSensor("waist1_sensor");

    // legSensor[12] = robot->getPositionSensor("LArm1_sensor");
    // legSensor[13] = robot->getPositionSensor("LArm2_sensor");
    // legSensor[14] = robot->getPositionSensor("LArm3_sensor");
    // legSensor[15] = robot->getPositionSensor("LArm4_sensor");
    // legSensor[16] = robot->getPositionSensor("RArm1_sensor");
    // legSensor[17] = robot->getPositionSensor("RArm2_sensor");
    // legSensor[18] = robot->getPositionSensor("RArm3_sensor");
    // legSensor[19] = robot->getPositionSensor("RArm4_sensor");

    // other sensors
    imu = robot->getInertialUnit("inertial_unit");
    gyro = robot->getGyro("gyro");
    accelerometer = robot->getAccelerometer("accelerometer");
    // Waistgps = robot->getGPS("gps_upperBody");
    // LFootGps = robot->getGPS("gps_LeftFoot");
    // RFootGps = robot->getGPS("gps_RightFoot");
    Waist = robot->getFromDef("GR1T1L");
    // SoleLeft = robot->getFromDef("Sole_Left");
    // SoleRight = robot->getFromDef("Sole_Right");
    // enable
    for (int i = 0; i < nJoint; i++)
    {
        legMotor[i]->enableTorqueFeedback(TIME_STEP);
    }
    for (int i = 0; i < nJoint; i++)
    {
        legSensor[i]->enable(TIME_STEP);
    }
    // for (int i = 0; i < 6; i++) {
    //     torqueSensor[i]->enableTorqueFeedback(TIME_STEP);
    // }
    // for (int i = 0; i < 2; i++) {
    //     forceSensor[i]->enable(TIME_STEP);
    // }
    imu->enable(TIME_STEP);
    gyro->enable(TIME_STEP);
    accelerometer->enable(TIME_STEP);
    // Waistgps->enable(TIME_STEP);
    // LFootGps->enable(TIME_STEP);
    // RFootGps->enable(TIME_STEP);
    // Derivative
    dRpy.resize(3);
    for (int i = 0; i < 3; i++)
    {
        dRpy.at(i).init(SAMPLE_TIME, 1e-3, 0.);
    }
    dJnt.resize(nJoint);
    for (int i = 0; i < nJoint; i++)
    {
        dJnt.at(i).init(SAMPLE_TIME, 1e-3, 0.);
    }
}

void WebotsRobot::deleteRobot()
{
    delete robot;
}

bool WebotsRobot::readData(double simTime, webotState &robotStateSim, Eigen::VectorXd &GodVision)
{
    // Motor pos
    robotStateSim.jointPosAct = getMotorPos();

    // Motor vel
    if (simTime > SAMPLE_TIME - 1e-6 && simTime < SAMPLE_TIME + 1e-6)
    {
        for (int i = 0; i < nJoint; i++)
        {
            dJnt.at(i).init(SAMPLE_TIME, 1e-3, robotStateSim.jointPosAct(i));
        }
    }
    for (int i = 0; i < nJoint; i++)
    {
        robotStateSim.jointVelAct(i) = dJnt.at(i).mSig(robotStateSim.jointPosAct(i), SAMPLE_TIME);
    }

    // Motor torque
    robotStateSim.jointTorAct = getMotorTau();

    // IMU Data 9-dof
    const double *rotmArray = Waist->getOrientation();
    Eigen::Matrix3d rotm;
    rotm << rotmArray[0], rotmArray[1], rotmArray[2],
        rotmArray[3], rotmArray[4], rotmArray[5],
        rotmArray[6], rotmArray[7], rotmArray[8];
    // robotStateSim.waistRpyAct = rotm2Rpy(rotm);
    // if (simTime > SAMPLE_TIME - 1e-6 && simTime < SAMPLE_TIME + 1e-6){
    //     for (int i = 0; i < 3; i++) {
    //         dRpy.at(i).init(SAMPLE_TIME, 1e-3, robotStateSim.waistRpyAct(i));
    //     }
    // }
    // for (int i = 0; i < 3; i++) {
    //     robotStateSim.waistRpyVelAct(i) = dRpy.at(i).mSig(robotStateSim.waistRpyAct(i));
    // }
    Eigen::Vector3d rpy = rotm2Rpy(rotx(PI) * rotm);
    robotStateSim.waistRpyAct << rpy(2), rpy(1), rpy(0);

    const double *waistVel = Waist->getVelocity(); // the first three is linear velocity the second three is angular velocity
    Eigen::Vector3d angularRate;
    angularRate << waistVel[3], waistVel[4], waistVel[5];
    robotStateSim.waistRpyVelAct = rotm.transpose() * angularRate;

    robotStateSim.waistXyzAccAct = getWaistAcc();
    robotStateSim.imu9DAct << robotStateSim.waistRpyAct, robotStateSim.waistRpyVelAct, robotStateSim.waistXyzAccAct;

    // const double &waistVel = Waist->getVelocity();
    const double *waistPos = Waist->getPosition();
    GodVision << waistVel[0], waistVel[1], waistVel[2], waistPos[0], waistPos[1], waistPos[2];

    // External Force
    //  robotStateSim.footGrfAct = getFootForce12D();

    return true;
}

bool WebotsRobot::setMotorPos(const Eigen::VectorXd &jointPosTar)
{
    for (int i = 0; i < nJoint; i++)
    {
        if (jointPosTar(i, 0) < 50000)
            legMotor[i]->setPosition(jointPosTar(i, 0));
    }
    return true;
}

bool WebotsRobot::setMotorTau(const Eigen::VectorXd &jointTauTar)
{
    for (int i = 0; i < nJoint; i++)
    {
        // if (i == 15 || i == 16 || i == 17 || i == 22 || i == 23 || i == 24 || i == 31 || i == 30 || i == 29) {
        //     legMotor[i]->setPosition(0.0);
        // }
        // if (i >= 15 && i <= 31) {
        //     // std::cout << i << std::endl;
        //     legMotor[i]->setPosition(0.0);
        // }
        // else if (i == 5 || i == 11) //|| i == 14 || i == 15 || i == 18 || i == 119)
        // {
        //     legMotor[i]->setPosition(0.0);
        // }

        if (jointTauTar(i, 0) < 50000)
            legMotor[i]->setTorque(jointTauTar(i));
    }
    return true;
}

Eigen::VectorXd WebotsRobot::getMotorPos()
{
    Eigen::VectorXd Q = Eigen::VectorXd::Zero(nJoint);
    for (int i = 0; i < nJoint; i++)
    {
        Q(i, 0) = legSensor[i]->getValue();
    }
    return Q;
}

Eigen::VectorXd WebotsRobot::getMotorTau()
{
    Eigen::VectorXd Tau = Eigen::VectorXd::Zero(nJoint);
    for (int i = 0; i < nJoint; i++)
    {
        Tau(i, 0) = legMotor[i]->getTorqueFeedback();
    }
    return Tau;
}

Eigen::Vector3d WebotsRobot::getWaistAcc()
{
    const double *data = accelerometer->getValues();
    Eigen::Vector3d acceleration(data[0], data[1], data[2]);
    return acceleration;
}

// Eigen::VectorXd WebotsRobot::getFootForce6D(const int& footFlag) {
//     const double* force;
//     double torqueX;
//     double torqueY;
//     double torqueZ;
//     switch (footFlag) {
//         case LEFTFOOT: {
//             force = forceSensor[0]->getValues();
//             torqueX = torqueSensor[0]->getTorqueFeedback();
//             torqueY = torqueSensor[1]->getTorqueFeedback();
//             torqueZ = torqueSensor[2]->getTorqueFeedback();
//             break;
//         }
//         case RIGHTFOOT: {
//             force = forceSensor[1]->getValues();
//             torqueX = torqueSensor[3]->getTorqueFeedback();
//             torqueY = torqueSensor[4]->getTorqueFeedback();
//             torqueZ = torqueSensor[5]->getTorqueFeedback();
//             break;
//         }
//         default: {
//             std::cout << " footFlag is wrong, return the values of LEFTFOOT by default ! " << std::endl;
//             force = forceSensor[0]->getValues();
//             torqueX = torqueSensor[0]->getTorqueFeedback();
//             torqueY = torqueSensor[1]->getTorqueFeedback();
//             torqueZ = torqueSensor[2]->getTorqueFeedback();
//             break;
//         }
//     }
//     Eigen::Vector3d forceVec = Eigen::Vector3d::Zero();
//     forceVec << force[0], force[1], force[2];
//     Eigen::Vector3d torqueVec = Eigen::Vector3d::Zero();
//     torqueVec << torqueX, torqueY, torqueZ;
//     Eigen::VectorXd footForce = Eigen::VectorXd::Zero(6);
//     footForce << forceVec, torqueVec;
//     return footForce;
// }

// Eigen::VectorXd WebotsRobot::getFootForce12D() {
//     Eigen::VectorXd LFootForce = getFootForce6D(LEFTFOOT);
//     Eigen::VectorXd RFootForce = getFootForce6D(RIGHTFOOT);
//     Eigen::VectorXd FootForce = Eigen::VectorXd::Zero(12);
//     FootForce << LFootForce,  RFootForce;
//     return FootForce;
// }

Eigen::Vector3d WebotsRobot::rotm2Rpy(const Eigen::Matrix3d &rotm)
{
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    rpy(0) = atan2(rotm(2, 1), rotm(2, 2));
    rpy(1) = atan2(-rotm(2, 0), sqrt(rotm(2, 1) * rotm(2, 1) + rotm(2, 2) * rotm(2, 2)));
    rpy(2) = atan2(rotm(1, 0), rotm(0, 0));
    return rpy;
}

Eigen::Vector3d WebotsRobot::rotm2xyz(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();

    euler.setZero();
    // y (-pi/2 pi/2)
    euler(1) = asin(R(0, 2));
    // z [-pi pi]
    double sinz = -R(0, 1) / cos(euler(1));
    double cosz = R(0, 0) / cos(euler(1));
    euler(2) = atan2(sinz, cosz);
    // x [-pi pi]
    double sinx = -R(1, 2) / cos(euler(1));
    double cosx = R(2, 2) / cos(euler(1));
    euler(0) = atan2(sinx, cosx);

    return euler;
}

Eigen::Matrix3d WebotsRobot::rotx(const double theta)
{
    Eigen::Matrix3d matRes;
    matRes << 1., 0., 0.,
        0., std::cos(theta), -std::sin(theta),
        0., std::sin(theta), std::cos(theta);
    return matRes;
}

// **************************************************************************************************************//

Derivative ::Derivative() {}

Derivative ::Derivative(double dT, double c)
{
    double alpha(2. / dT);
    this->a0 = c * alpha + 1.;
    this->a1 = (1. - c * alpha) / this->a0;
    this->b0 = alpha / this->a0;
    this->b1 = -alpha / this->a0;
    this->a0 = 1.;
    this->sigInPrev = 0.;
    this->sigOutPrev = 0.;
}

void Derivative ::init(double dT, double c, double initValue)
{
    double alpha(2. / dT);
    this->a0 = c * alpha + 1.;
    this->a1 = (1. - c * alpha) / this->a0;
    this->b0 = alpha / this->a0;
    this->b1 = -alpha / this->a0;
    this->a0 = 1.;
    this->sigInPrev = initValue;
    this->sigOutPrev = initValue;
}

double Derivative ::mSig(double sigIn, double dT)
{
    double sigOut(0.);
    if (sigIn - sigInPrev < -PI)
    {
        sigOut = (2. * PI + sigIn - sigInPrev) / dT;
    }
    else if (sigIn - sigInPrev > PI)
    {
        sigOut = (sigIn - 2. * PI - sigInPrev) / dT;
    }
    else
    {
        sigOut = (sigIn - sigInPrev) / dT;
    }
    // sigOut = b0 * sigIn + b1 * sigInPrev - a1 * sigOutPrev;

    sigOutPrev = sigOut;
    sigInPrev = sigIn;
    return sigOut;
}
