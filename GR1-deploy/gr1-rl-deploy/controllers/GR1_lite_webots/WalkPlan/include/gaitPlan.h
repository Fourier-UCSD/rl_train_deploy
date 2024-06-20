#ifndef GAIT_PLAN_H
#define GAIT_PLAN_H

#include "Robot_Data.h"
#include "aeroWalkPlan.h"
#include "basicfunction.h"

class gaitPlan{
    private:
        double vZInit;
        double vZEnd;
        double zMid;
        int firstFlag;
        Eigen::VectorXd xStand;
        Eigen::Vector3d offSetL; 
        Eigen::Vector3d offSetR;
        Eigen::VectorXd qCmd = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd qDotCmd = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd qCmd_pre = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd qDotCmd_pre = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd jointP = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd jointD = Eigen::VectorXd::Zero(12);
        Eigen::Vector3d pFoot, vFoot;
        double hd;
        double lambda;                                         // lambda = sqrt(g/h); 
        std::vector<double> pOffset;
        Eigen::Vector3d pTorso_td, vTorso_td, vTorso_td_filt, pFootStrike;
    public:
        gaitPlan();
        void init(Eigen::VectorXd qCmd_, Eigen::VectorXd qDotCmd_, Eigen::VectorXd xStand_);
        bool predTouchState(Robot_Data *robotdata);
        bool walkPlan(Robot_Data *robotdata, Eigen::VectorXd &qCmd_, Eigen::VectorXd &qDotCmd_);
        bool stepPlan(Robot_Data *robotdata, Eigen::VectorXd &qCmd_, Eigen::VectorXd &qDotCmd_);
        bool swingPlan(Robot_Data *robotdata);
        bool torsoPlan(Robot_Data *robotdata);
        bool cart2Joint(Robot_Data *robotdata);
        bool smoothQCmd(Robot_Data *robotdata);

        Eigen::Vector3d matrixtoeulerxyz_(Eigen::Matrix3d R);
        void quaternionInterp(Eigen::Matrix3d R_start,
                                                Eigen::Matrix3d R_end, 
                                                double totaltime, double currenttime,
                                                Eigen::Matrix3d& R_d, Eigen::Vector3d& omiga_d, Eigen::Vector3d& acc_d);
        Eigen::Matrix3d QuanteiniontoMatrix(RigidBodyDynamics::Math::Quaternion Q);
        void FifthPoly(Eigen::VectorXd p0, Eigen::VectorXd p0_dot, Eigen::VectorXd p0_dotdot,     // start point states 
                     Eigen::VectorXd p1, Eigen::VectorXd p1_dot, Eigen::VectorXd p1_dotdot,     // end point states 
                     double totalTime,       // total permating time 
                     double currenttime,     //current time,from 0 to total time 
                     Eigen::VectorXd& pd, Eigen::VectorXd& pd_dot, Eigen::VectorXd& pd_dotdot );
};
#endif //GAIT_PLAN_H