#ifndef AEROWALKPLAN
#define AEROWALKPLAN
#include <Eigen/Dense>
#include <fstream>
bool quintic(double x1, double x2, double y1, double y2, double dy1, double dy2, double ddy1, double ddy2, double x, double dx, double& y, double& dy, double& ddy);
double clamp(double num, double lim1, double lim2);
double myPow(double x, int n);
void Thirdpoly(double p0, double p0_dot,double p1, double p1_dot,
                    double totalTime,       // total permating time 
                     double currenttime,     //current time,from 0 to total time 
                     double& pd, double& pd_dot);
double fact(int n);
void iniBezier(int M, Eigen::VectorXd &bezCoeft, Eigen::VectorXd &pBezCoeft, Eigen::VectorXd &p2BezCoeft);
void calBezier(double s, Eigen::MatrixXd para, Eigen::VectorXd bezCoeft, int N, int M, Eigen::VectorXd &theta);
void calpBezier(double s, Eigen::MatrixXd para, Eigen::VectorXd pBezCoeft,int N, int M, Eigen::VectorXd &ptheta);
void calp2Bezier(double s, Eigen::MatrixXd para, Eigen::VectorXd p2BezCoeft,int N, int M, Eigen::VectorXd &p2theta);
void oneLegTest(double t, double &tStepPre, int &stIndex, Eigen::VectorXd &qDiff, Eigen::VectorXd &qDotDiff, Eigen::VectorXd bezCoeft, Eigen::VectorXd pBezCoeft, Eigen::VectorXd &qCmd, Eigen::VectorXd &qDotCmd, Eigen::VectorXd &torCmd);
void oneLegTest2(double t, double &tStepPre, int &stIndex, Eigen::VectorXd &qDiff, Eigen::VectorXd &qDotDiff, 
                    Eigen::VectorXd &qDDotDiff, Eigen::VectorXd bezCoeft, Eigen::VectorXd pBezCoeft, Eigen::VectorXd p2BezCoeft, 
                    Eigen::VectorXd &qCmd, Eigen::VectorXd &qDotCmd, Eigen::VectorXd &qDDotCmd, Eigen::VectorXd &torCmd);
bool TriPointsQuintic(double T, double t, double x1, double v1, double x2, double v2, double x3, double v3, double &y, double &dy, double &ddy);
void TwoPointsCubic(double p0, double p0_dot,double p1, double p1_dot,
                    double totalTime,       // total permating time 
                     double currenttime,     //current time,from 0 to total time 
                     double& pd, double& pd_dot, double& pd_ddot);
void oneLegTest3(double t, double &vCmd, double &tStepPre, int &stIndex, Eigen::VectorXd xStand, Eigen::VectorXd &xInit, Eigen::VectorXd &xDotInit, 
                Eigen::VectorXd &xDDotInit, Eigen::VectorXd &xEnd, Eigen::VectorXd &xDotEnd, Eigen::VectorXd &xCmd, Eigen::VectorXd &xDotCmd, Eigen::VectorXd &xDDotCmd, Eigen::VectorXd &fCmd);
Eigen::Matrix3d rotX(double q);
Eigen::Matrix3d rotY(double q);
Eigen::Matrix3d rotZ(double q);
void oneLegIK(Eigen::VectorXd Pos, Eigen::VectorXd RPY, Eigen::VectorXd &qIK);
void oneLegIK2(Eigen::VectorXd Pos, Eigen::Matrix3d R, Eigen::VectorXd &qIK);
void wkSpace2Joint(Eigen::VectorXd xCmd, Eigen::VectorXd &qCmd, Eigen::VectorXd &qDotCmd, bool &firstFlag);
void wkSpace2Joint(Eigen::VectorXd xCmd, Eigen::VectorXd rpyCmd, Eigen::VectorXd &qCmd, Eigen::VectorXd &qDotCmd, bool &firstFlag);
bool dataLog(Eigen::VectorXd &v, std::ofstream &f);

#endif //AEROWALKPLAN
#pragma once