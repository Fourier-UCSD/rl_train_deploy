#include "aeroWalkPlan.h"
#include <iostream>
#include <stdlib.h>

bool quintic(double x1, double x2, double y1, double y2, double dy1, double dy2, double ddy1, double ddy2, double x, double dx, double& y, double& dy, double& ddy){
    // Limit range since curve fit is only valid within range
    x = clamp(x, x1, x2);

    // Declare variables
    double t, t1, t2, deltaY, deltaT, a0, a1, a2, a3, a4, a5;

    // variable substitution
    t = x/dx;
    t1 = x1/dx;
    t2 = x2/dx;
    // interpolate
    deltaY = y2 - y1;
    deltaT = t2 - t1;
    a0 = y1;
    a1 = dy1;
    a2 = 1.0/2.*ddy1;
    a3 = 1.0/(2.*deltaT*deltaT*deltaT)*(20.*deltaY - (8.*dy2 + 12.*dy1)*deltaT + (ddy2 - 3.*ddy1)*(deltaT*deltaT));
    a4 = 1.0/(2.*deltaT*deltaT*deltaT*deltaT)*(-30.*deltaY + (14.*dy2 + 16.*dy1)*deltaT + (3.*ddy1 - 2.*ddy2)*(deltaT*deltaT));
    a5 = 1.0/(2.*deltaT*deltaT*deltaT*deltaT*deltaT)*(12.*deltaY - 6.*(dy2 + dy1)*deltaT + (ddy2 - ddy1)*(deltaT*deltaT));

    // position
    y = a0 + a1*myPow((t - t1),1) + a2*myPow((t - t1),2) + a3*myPow((t - t1),3) + a4*myPow(t - t1,4) + a5*myPow(t - t1,5);
    // velocity
    dy = a1 + 2.*a2*myPow((t - t1),1) + 3.*a3*myPow((t - t1),2) + 4.*a4*myPow(t - t1,3) + 5.*a5*myPow(t - t1,4);
    // acceleration
    ddy = 2.*a2 +6.*a3*myPow((t - t1),1) + 12.*a4*myPow(t - t1,2) + 20.*a5*myPow(t - t1,3);

    return true;
}
double clamp(double num, double lim1, double lim2) {
    auto min = std::min(lim1, lim2);
    auto max = std::max(lim1, lim2);

    if (num < min)
        return min;

    if (max < num)
        return max;

    return num;
}
double myPow(double x, int n){
    if(n == 0)
        return 1.0;
    if(n < 0)
        return 1.0/myPow(x,-n);
    double half = myPow(x,n>>1);

    if(n%2 == 0)
        return half*half;
    else
    {
        return half*half*x;
    }

}

void Thirdpoly(double p0, double p0_dot,double p1, double p1_dot,
                    double totalTime,       // total permating time 
                     double currenttime,     //current time,from 0 to total time 
                     double& pd, double& pd_dot)
                     {
                         if(currenttime < totalTime){
                            double a0 = p0;
                            double a1 = p0_dot;
                            double m = p1 -p0 -p0_dot*totalTime;
                            double n = p1_dot - p0_dot;
                            double a2 = 3*m/(totalTime*totalTime) - n/totalTime;
                            double a3 = -2*m/(totalTime*totalTime*totalTime) + n/(totalTime*totalTime);
                            pd = a3*currenttime*currenttime*currenttime + a2*currenttime*currenttime + a1*currenttime +a0;
                            pd_dot = 3*a3*currenttime*currenttime + 2*a2*currenttime + a1;
                         }else{
                             pd = p1;
                             pd_dot = p1_dot;
                         }

                     }
double fact(int n)
{
    double result = 1;
    if (n == 0)
        result = 1;
    else
        for (int i = 1;i <= n;result *= i, i++);
    return result;
}

void iniBezier(int M, Eigen::VectorXd &bezCoeft, Eigen::VectorXd &pBezCoeft, Eigen::VectorXd &p2BezCoeft)
{
    for (int k = 0; k <= M; k++) {
        bezCoeft[k] =  fact(M) / (fact(k)*fact(M - k));
    }
    for (int k = 0; k < M; k++) {
        pBezCoeft[k] = fact(M) / (fact(k)*fact(M - k - 1));
    }
    for (int k = 0; k < M-1; k++) {
        p2BezCoeft[k] = fact(M) / (fact(k)*fact(M - k - 2));
    }
}

void calBezier(double s, Eigen::MatrixXd para, Eigen::VectorXd bezCoeft, int N, int M, Eigen::VectorXd &theta)
{
    for (int i = 0; i < N; i++) {
        theta[i] = 0.0;
        for (int k = 0; k <= M; k++) {
            theta[i] = theta[i] + para(i,k) * bezCoeft[k]*myPow(s, k)*myPow(1 - s, M - k);
        }
    }
}
void calpBezier(double s, Eigen::MatrixXd para, Eigen::VectorXd pBezCoeft,int N, int M, Eigen::VectorXd &ptheta)
{
    for (int i = 0; i < N; i++) {
        ptheta[i] = 0.0;
        for (int k = 0; k < M; k++) {
            ptheta[i] = ptheta[i] + (para(i,k+1) - para(i,k)) * pBezCoeft[k]*myPow(s, k)*myPow(1 - s, M - k - 1);
        }
    }
}
void calp2Bezier(double s, Eigen::MatrixXd para, Eigen::VectorXd p2BezCoeft,int N, int M, Eigen::VectorXd &p2theta)
{
    for (int i = 0; i < N; i++) {
        p2theta[i] = 0.0;
        for (int k = 0; k < M-1; k++) {
            p2theta[i] = p2theta[i] + (para(i,k+2) - 2.0*para(i,k+1) + para(i,k)) * p2BezCoeft[k]*myPow(s, k)*myPow(1 - s, M - k - 2);
        }
    }
}
void oneLegTest(double t, double &tStepPre, int &stIndex, Eigen::VectorXd &qDiff, Eigen::VectorXd &qDotDiff, Eigen::VectorXd bezCoeft, Eigen::VectorXd pBezCoeft, Eigen::VectorXd &qCmd, Eigen::VectorXd &qDotCmd, Eigen::VectorXd &torCmd)
{   
    double timeStep = 0.004;
    Eigen::MatrixXd paraL(12,7), paraR(12,7), para(12,7), paraForce(12,7);
    double tStep, s;
    double T = 0.36;
    double tTrans = 0.16;
    int M = 6, N= 12;
    bool startFlag = false;
    Eigen::VectorXd qBezier(N), qDotBezier(N), torBezier(N);
    // para for vx=0.0m/s
    // paraR << 0.0295,0.0366,0.0123,0.0550,0.0122,0.0373,0.0299,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3807,-0.3829,-0.4188,-0.3179,-0.4188,-0.3823,-0.3807,
    //         0.9790,0.9832,0.9665,0.9918,0.9679,0.9826,0.9796,
    //         -0.5947,-0.5960,-0.5943,-0.5957,-0.5954,-0.5953,-0.5949,
    //         -0.0291,-0.0440,-0.0392,-0.0437,-0.0389,-0.0438,-0.0293,
    //         -0.0299,-0.0220,-0.0918,-0.0716,-0.0928,-0.0276,-0.0295,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3807,-0.3787,-0.1958,-0.6903,-0.1958,-0.3776,-0.3807,
    //         0.9796,0.9787,1.0522,1.9932,1.0497,0.9876,0.9790,
    //         -0.5949,-0.6034,-0.8811,-1.2566,-0.8768,-0.6143,-0.5947,
    //         0.0293,0.0214,0.0486,0.1158,0.0382,0.0304,0.0291;
    // paraL << 0.0299,0.0220,0.0918,0.0716,0.0928,0.0276,0.0295,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3807,-0.3787,-0.1958,-0.6903,-0.1958,-0.3776,-0.3807,
    //         0.9796,0.9787,1.0522,1.9932,1.0497,0.9876,0.9790,
    //         -0.5949,-0.6034,-0.8811,-1.2566,-0.8768,-0.6143,-0.5947,
    //         -0.0293,-0.0214,-0.0486,-0.1158,-0.0382,-0.0304,-0.0291,
    //         -0.0295,-0.0366,-0.0123,-0.0550,-0.0122,-0.0373,-0.0299,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3807,-0.3829,-0.4188,-0.3179,-0.4188,-0.3823,-0.3807,
    //         0.9790,0.9832,0.9665,0.9918,0.9679,0.9826,0.9796,
    //         -0.5947,-0.5960,-0.5943,-0.5957,-0.5954,-0.5953,-0.5949,
    //         0.0291,0.0440,0.0392,0.0437,0.0389,0.0438,0.0293;

    //para for vx=0.3m/s
    // paraR << 0.0109,0.0294,-0.0244,0.0647,-0.0227,0.0295,0.0120,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.5217,-0.5220,-0.3964,-0.4306,-0.4089,-0.3560,-0.3484,
    //         0.8458,0.8623,0.8223,0.8793,0.8313,0.8594,0.8542,
    //         -0.3291,-0.3740,-0.3840,-0.4368,-0.4365,-0.4824,-0.5117,
    //         -0.0115,-0.0277,-0.0210,-0.0295,-0.0221,-0.0255,-0.0101,
    //         -0.0120,0.0069,-0.1262,0.0010,-0.1235,0.0030,-0.0109,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3484,-0.3410,-0.3643,-0.7271,-0.3391,-0.5203,-0.5217,
    //         0.8542,0.8502,1.2145,1.9081,0.9420,0.8411,0.8458,
    //         -0.5117,-0.5439,-0.7954,-1.1891,-0.5998,-0.3035,-0.3291,
    //         0.0101,0.0023,0.0709,0.0527,0.0565,0.0095,0.0115;
    // paraL << 0.0120,-0.0069,0.1262,-0.0010,0.1235,-0.0030,0.0109,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3484,-0.3410,-0.3643,-0.7271,-0.3391,-0.5203,-0.5217,
    //         0.8542,0.8502,1.2145,1.9081,0.9420,0.8411,0.8458,
    //         -0.5117,-0.5439,-0.7954,-1.1891,-0.5998,-0.3035,-0.3291,
    //         -0.0101,-0.0023,-0.0709,-0.0527,-0.0565,-0.0095,-0.0115,
    //         -0.0109,-0.0294,0.0244,-0.0647,0.0227,-0.0295,-0.0120,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.5217,-0.5220,-0.3964,-0.4306,-0.4089,-0.3560,-0.3484,
    //         0.8458,0.8623,0.8223,0.8793,0.8313,0.8594,0.8542,
    //         -0.3291,-0.3740,-0.3840,-0.4368,-0.4365,-0.4824,-0.5117,
    //         0.0115,0.0277,0.0210,0.0295,0.0221,0.0255,0.0101;

    //para for vx=0.6m/s
    // paraR << -0.0032,0.0386,-0.0631,0.0902,-0.0719,0.0440,0.0029,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.5821,-0.5883,-0.3503,-0.4611,-0.3605,-0.2765,-0.2353,
    //         0.7863,0.8517,0.7405,0.8697,0.7504,0.8275,0.8074,
    //         -0.2063,-0.3107,-0.3122,-0.4207,-0.4170,-0.5127,-0.5752,
    //         0.0004,-0.0181,-0.0040,-0.0197,-0.0060,-0.0168,0.0010,
    //         -0.0029,0.0406,-0.1990,0.0520,-0.1931,0.0389,0.0032,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.2353,-0.1951,-0.3697,-0.6993,-0.4875,-0.5769,-0.5821,
    //         0.8074,0.7883,1.2793,2.0615,1.0199,0.7329,0.7863,
    //         -0.5752,-0.6418,-0.8187,-1.3959,-0.5417,-0.1224,-0.2063,
    //         -0.0010,-0.0125,0.1158,0.0322,0.0973,-0.0062,-0.0004;
    // paraL << 0.0029,-0.0406,0.1990,-0.0520,0.1931,-0.0389,-0.0032,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.2353,-0.1951,-0.3697,-0.6993,-0.4875,-0.5769,-0.5821,
    //         0.8074,0.7883,1.2793,2.0615,1.0199,0.7329,0.7863,
    //         -0.5752,-0.6418,-0.8187,-1.3959,-0.5417,-0.1224,-0.2063,
    //         0.0010,0.0125,-0.1158,-0.0322,-0.0973,0.0062,0.0004,
    //         0.0032,-0.0386,0.0631,-0.0902,0.0719,-0.0440,-0.0029,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.5821,-0.5883,-0.3503,-0.4611,-0.3605,-0.2765,-0.2353,
    //         0.7863,0.8517,0.7405,0.8697,0.7504,0.8275,0.8074,
    //         -0.2063,-0.3107,-0.3122,-0.4207,-0.4170,-0.5127,-0.5752,
    //         -0.0004,0.0181,0.0040,0.0197,0.0060,0.0168,-0.0010;

    //para for vx=0.9m/s
    paraR << -0.0137,0.0563,-0.0483,0.0487,-0.0883,0.0623,-0.0059,
            0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
            -0.7073,-0.7057,-0.4648,-0.4289,-0.3589,-0.2758,-0.1613,
            0.8657,1.0005,0.8425,0.9136,0.7934,0.9469,0.8765,
            -0.1583,-0.3360,-0.3375,-0.4583,-0.4674,-0.6360,-0.7161,
            0.0113,-0.0147,0.0132,-0.0184,0.0153,-0.0160,0.0096,
            0.0059,0.0775,-0.2224,0.0286,-0.2504,0.0752,0.0137,
            0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
            -0.1613,-0.0498,-0.4996,-0.3891,-0.8339,-0.7207,-0.7073,
            0.8765,0.8081,1.4397,1.5075,1.3896,0.7585,0.8657,
            -0.7161,-0.8008,-0.8882,-1.1117,-0.5714,-0.0069,-0.1583,
            -0.0096,-0.0304,0.1805,0.0116,0.1607,-0.0238,-0.0113;
    paraL << -0.0059,-0.0775,0.2224,-0.0286,0.2504,-0.0752,-0.0137,
            0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
            -0.1613,-0.0498,-0.4996,-0.3891,-0.8339,-0.7207,-0.7073,
            0.8765,0.8081,1.4397,1.5075,1.3896,0.7585,0.8657,
            -0.7161,-0.8008,-0.8882,-1.1117,-0.5714,-0.0069,-0.1583,
            0.0096,0.0304,-0.1805,-0.0116,-0.1607,0.0238,0.0113,
            0.0137,-0.0563,0.0483,-0.0487,0.0883,-0.0623,0.0059,
            0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
            -0.7073,-0.7057,-0.4648,-0.4289,-0.3589,-0.2758,-0.1613,
            0.8657,1.0005,0.8425,0.9136,0.7934,0.9469,0.8765,
            -0.1583,-0.3360,-0.3375,-0.4583,-0.4674,-0.6360,-0.7161,
            -0.0113,0.0147,-0.0132,0.0184,-0.0153,0.0160,-0.0096;

    paraForce = Eigen::MatrixXd::Zero(12,7);

    tStep = fmod(t, T);
    if(tStep-tStepPre < -T/2){
        stIndex = 1 - stIndex;
        startFlag = true;
    }
    tStepPre = tStep;
    s = tStep/T;
    if(stIndex==0)
        para = paraL;
    else
        para = paraR;

    calBezier(s, para, bezCoeft, N, M, qBezier);
    calpBezier(s, para, pBezCoeft, N, M, qDotBezier); 
    calBezier(s, paraForce, bezCoeft, N, M, torBezier);

    if(startFlag){
        qDiff = qCmd - qBezier;
        qDotDiff = qDotCmd - qDotBezier/T;
    }

    double qTrans = 0.0, qDotTrans = 0.0;
    if(tStep<tTrans-0.5*timeStep){
        for(int i=0;i<N;++i){
            Thirdpoly(qDiff(i), qDotDiff(i), 0.0, 0.0, tTrans-timeStep, tStep+timeStep, qTrans, qDotTrans);
            qCmd(i) = qBezier(i) + qTrans;
            qDotCmd(i) = qDotBezier(i)/T + qDotTrans;
            torCmd(i) = torBezier(i);
        }        
    }
    else{
        qCmd = qBezier;
        qDotCmd = qDotBezier/T;
        torCmd = torBezier;
    }
    
}
void oneLegTest2(double t, double &tStepPre, int &stIndex, Eigen::VectorXd &qDiff, Eigen::VectorXd &qDotDiff, 
                Eigen::VectorXd &qDDotDiff, Eigen::VectorXd bezCoeft, Eigen::VectorXd pBezCoeft, Eigen::VectorXd p2BezCoeft, 
                Eigen::VectorXd &qCmd, Eigen::VectorXd &qDotCmd, Eigen::VectorXd &qDDotCmd, Eigen::VectorXd &torCmd)
{   
    double timeStep = 0.005;
    Eigen::MatrixXd paraL(12,7), paraR(12,7), para(12,7), paraForce(12,7);
    double tStep, s;
    double T = 0.36;
    double tTrans = 0.16;
    int M = 6, N= 12;
    bool startFlag = false;
    Eigen::VectorXd qBezier(N), qDotBezier(N), qDDotBezier(N), torBezier(N);
    // para for vx=0.0m/s
    // paraR << 0.0295,0.0366,0.0123,0.0550,0.0122,0.0373,0.0299,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3807,-0.3829,-0.4188,-0.3179,-0.4188,-0.3823,-0.3807,
    //         0.9790,0.9832,0.9665,0.9918,0.9679,0.9826,0.9796,
    //         -0.5947,-0.5960,-0.5943,-0.5957,-0.5954,-0.5953,-0.5949,
    //         -0.0291,-0.0440,-0.0392,-0.0437,-0.0389,-0.0438,-0.0293,
    //         -0.0299,-0.0220,-0.0918,-0.0716,-0.0928,-0.0276,-0.0295,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3807,-0.3787,-0.1958,-0.6903,-0.1958,-0.3776,-0.3807,
    //         0.9796,0.9787,1.0522,1.9932,1.0497,0.9876,0.9790,
    //         -0.5949,-0.6034,-0.8811,-1.2566,-0.8768,-0.6143,-0.5947,
    //         0.0293,0.0214,0.0486,0.1158,0.0382,0.0304,0.0291;
    // paraL << 0.0299,0.0220,0.0918,0.0716,0.0928,0.0276,0.0295,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3807,-0.3787,-0.1958,-0.6903,-0.1958,-0.3776,-0.3807,
    //         0.9796,0.9787,1.0522,1.9932,1.0497,0.9876,0.9790,
    //         -0.5949,-0.6034,-0.8811,-1.2566,-0.8768,-0.6143,-0.5947,
    //         -0.0293,-0.0214,-0.0486,-0.1158,-0.0382,-0.0304,-0.0291,
    //         -0.0295,-0.0366,-0.0123,-0.0550,-0.0122,-0.0373,-0.0299,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3807,-0.3829,-0.4188,-0.3179,-0.4188,-0.3823,-0.3807,
    //         0.9790,0.9832,0.9665,0.9918,0.9679,0.9826,0.9796,
    //         -0.5947,-0.5960,-0.5943,-0.5957,-0.5954,-0.5953,-0.5949,
    //         0.0291,0.0440,0.0392,0.0437,0.0389,0.0438,0.0293;

    //para for vx=0.3m/s
    // paraR << 0.0109,0.0294,-0.0244,0.0647,-0.0227,0.0295,0.0120,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.5217,-0.5220,-0.3964,-0.4306,-0.4089,-0.3560,-0.3484,
    //         0.8458,0.8623,0.8223,0.8793,0.8313,0.8594,0.8542,
    //         -0.3291,-0.3740,-0.3840,-0.4368,-0.4365,-0.4824,-0.5117,
    //         -0.0115,-0.0277,-0.0210,-0.0295,-0.0221,-0.0255,-0.0101,
    //         -0.0120,0.0069,-0.1262,0.0010,-0.1235,0.0030,-0.0109,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3484,-0.3410,-0.3643,-0.7271,-0.3391,-0.5203,-0.5217,
    //         0.8542,0.8502,1.2145,1.9081,0.9420,0.8411,0.8458,
    //         -0.5117,-0.5439,-0.7954,-1.1891,-0.5998,-0.3035,-0.3291,
    //         0.0101,0.0023,0.0709,0.0527,0.0565,0.0095,0.0115;
    // paraL << 0.0120,-0.0069,0.1262,-0.0010,0.1235,-0.0030,0.0109,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.3484,-0.3410,-0.3643,-0.7271,-0.3391,-0.5203,-0.5217,
    //         0.8542,0.8502,1.2145,1.9081,0.9420,0.8411,0.8458,
    //         -0.5117,-0.5439,-0.7954,-1.1891,-0.5998,-0.3035,-0.3291,
    //         -0.0101,-0.0023,-0.0709,-0.0527,-0.0565,-0.0095,-0.0115,
    //         -0.0109,-0.0294,0.0244,-0.0647,0.0227,-0.0295,-0.0120,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.5217,-0.5220,-0.3964,-0.4306,-0.4089,-0.3560,-0.3484,
    //         0.8458,0.8623,0.8223,0.8793,0.8313,0.8594,0.8542,
    //         -0.3291,-0.3740,-0.3840,-0.4368,-0.4365,-0.4824,-0.5117,
    //         0.0115,0.0277,0.0210,0.0295,0.0221,0.0255,0.0101;

    //para for vx=0.6m/s
    // paraR << -0.0032,0.0386,-0.0631,0.0902,-0.0719,0.0440,0.0029,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.5821,-0.5883,-0.3503,-0.4611,-0.3605,-0.2765,-0.2353,
    //         0.7863,0.8517,0.7405,0.8697,0.7504,0.8275,0.8074,
    //         -0.2063,-0.3107,-0.3122,-0.4207,-0.4170,-0.5127,-0.5752,
    //         0.0004,-0.0181,-0.0040,-0.0197,-0.0060,-0.0168,0.0010,
    //         -0.0029,0.0406,-0.1990,0.0520,-0.1931,0.0389,0.0032,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.2353,-0.1951,-0.3697,-0.6993,-0.4875,-0.5769,-0.5821,
    //         0.8074,0.7883,1.2793,2.0615,1.0199,0.7329,0.7863,
    //         -0.5752,-0.6418,-0.8187,-1.3959,-0.5417,-0.1224,-0.2063,
    //         -0.0010,-0.0125,0.1158,0.0322,0.0973,-0.0062,-0.0004;
    // paraL << 0.0029,-0.0406,0.1990,-0.0520,0.1931,-0.0389,-0.0032,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.2353,-0.1951,-0.3697,-0.6993,-0.4875,-0.5769,-0.5821,
    //         0.8074,0.7883,1.2793,2.0615,1.0199,0.7329,0.7863,
    //         -0.5752,-0.6418,-0.8187,-1.3959,-0.5417,-0.1224,-0.2063,
    //         0.0010,0.0125,-0.1158,-0.0322,-0.0973,0.0062,0.0004,
    //         0.0032,-0.0386,0.0631,-0.0902,0.0719,-0.0440,-0.0029,
    //         0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
    //         -0.5821,-0.5883,-0.3503,-0.4611,-0.3605,-0.2765,-0.2353,
    //         0.7863,0.8517,0.7405,0.8697,0.7504,0.8275,0.8074,
    //         -0.2063,-0.3107,-0.3122,-0.4207,-0.4170,-0.5127,-0.5752,
    //         -0.0004,0.0181,0.0040,0.0197,0.0060,0.0168,-0.0010;
    
    //para for vx = 0.9m/s
    paraR << -0.0137,0.0563,-0.0483,0.0487,-0.0883,0.0623,-0.0059,
            0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
            -0.7073,-0.7057,-0.4648,-0.4289,-0.3589,-0.2758,-0.1613,
            0.8657,1.0005,0.8425,0.9136,0.7934,0.9469,0.8765,
            -0.1583,-0.3360,-0.3375,-0.4583,-0.4674,-0.6360,-0.7161,
            0.0113,-0.0147,0.0132,-0.0184,0.0153,-0.0160,0.0096,
            0.0059,0.0775,-0.2224,0.0286,-0.2504,0.0752,0.0137,
            0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
            -0.1613,-0.0498,-0.4996,-0.3891,-0.8339,-0.7207,-0.7073,
            0.8765,0.8081,1.4397,1.5075,1.3896,0.7585,0.8657,
            -0.7161,-0.8008,-0.8882,-1.1117,-0.5714,-0.0069,-0.1583,
            -0.0096,-0.0304,0.1805,0.0116,0.1607,-0.0238,-0.0113;
    paraL << -0.0059,-0.0775,0.2224,-0.0286,0.2504,-0.0752,-0.0137,
            0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
            -0.1613,-0.0498,-0.4996,-0.3891,-0.8339,-0.7207,-0.7073,
            0.8765,0.8081,1.4397,1.5075,1.3896,0.7585,0.8657,
            -0.7161,-0.8008,-0.8882,-1.1117,-0.5714,-0.0069,-0.1583,
            0.0096,0.0304,-0.1805,-0.0116,-0.1607,0.0238,0.0113,
            0.0137,-0.0563,0.0483,-0.0487,0.0883,-0.0623,0.0059,
            0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
            -0.7073,-0.7057,-0.4648,-0.4289,-0.3589,-0.2758,-0.1613,
            0.8657,1.0005,0.8425,0.9136,0.7934,0.9469,0.8765,
            -0.1583,-0.3360,-0.3375,-0.4583,-0.4674,-0.6360,-0.7161,
            -0.0113,0.0147,-0.0132,0.0184,-0.0153,0.0160,-0.0096;



    paraForce = Eigen::MatrixXd::Zero(12,7);

    tStep = fmod(t, T);
    if(T - tStep < 0.5*timeStep || tStep < 0.5*timeStep){    
        tStep = 0.0;
        stIndex = 1 - stIndex;
        startFlag = true;
    }
    
    // if(tStep-tStepPre < -T/2){
    //     stIndex = 1 - stIndex;
    //     startFlag = true;
    // }
    tStepPre = tStep;
    s = tStep/T;
    if(stIndex==0)
        para = paraL;
    else
        para = paraR;

    calBezier(s, para, bezCoeft, N, M, qBezier);
    calpBezier(s, para, pBezCoeft, N, M, qDotBezier);
    calp2Bezier(s, para, p2BezCoeft, N, M, qDDotBezier);  
    calBezier(s, paraForce, bezCoeft, N, M, torBezier);

    if(startFlag){
        qDiff = qCmd - qBezier;
        qDotDiff = qDotCmd - qDotBezier/T;
        qDDotDiff = qDDotCmd - qDDotBezier/T/T;
    }

    double qTrans = 0.0, qDotTrans = 0.0, qDDotTrans = 0.0;
    if(tStep<tTrans-0.5*timeStep){
        for(int i=0;i<N;++i){
            quintic(0, tTrans, qDiff(i), 0.0, qDotDiff(i), 0.0, qDDotDiff(i), 0.0, tStep, 1.0, qTrans, qDotTrans, qDDotTrans);
            qCmd(i) = qBezier(i) + qTrans;
            qDotCmd(i) = qDotBezier(i)/T + qDotTrans;
            qDDotCmd(i) = qDDotBezier(i)/T/T + qDDotTrans;
            torCmd(i) = torBezier(i);
        }        
    }
    else{
        qCmd = qBezier;
        qDotCmd = qDotBezier/T;
        qDDotCmd = qDDotBezier/T/T;
        torCmd = torBezier;
    }
    
}
bool TriPointsQuintic(double T, double t, double x1, double v1, double x2, double v2, double x3, double v3, double &y, double &dy, double &ddy){
    double a0, a1, a2, a3, a4, a5;
    a0 = x1;
    a1 = v1;
    a2 = -(23.*x1 - 16.*x2 - 7.*x3 + 6.*T*v1 + 8*T*v2 + T*v3)/myPow(T,2);
    a3 = (66.*x1 - 32.*x2 - 34.*x3 + 13.*T*v1 + 32.*T*v2 + 5.*T*v3)/myPow(T,3);
    a4 = -4.*(17.*x1 - 4.*x2 - 13.*x3 + 3.*T*v1 + 10.*T*v2 + 2.*T*v3)/myPow(T,4);
    a5 = 4.*(6.*x1 - 6.*x3 + T*v1 + 4.*T*v2 + T*v3)/myPow(T,5);

    y = a0 + a1*myPow(t,1) + a2*myPow(t,2) + a3*myPow(t,3) + a4*myPow(t,4) + a5*myPow(t,5);
    dy = a1 + 2.*a2*myPow(t,1) + 3.*a3*myPow(t,2) + 4.*a4*myPow(t,3) + 5.*a5*myPow(t,4);
    ddy = 2.*a2 + 6.*a3*t + 12.*a4*myPow(t,2) + 20.*a5*myPow(t,3);
    return true;
};
void TwoPointsCubic(double p0, double p0_dot,double p1, double p1_dot,
                    double totalTime,       // total permating time 
                     double currenttime,     //current time,from 0 to total time 
                     double& pd, double& pd_dot, double& pd_ddot)
                     {  
                        double a0 = p0;
                        double a1 = p0_dot;
                        double m = p1 -p0 -p0_dot*totalTime;
                        double n = p1_dot - p0_dot;
                        double a2 = 3.*m/(totalTime*totalTime) - n/totalTime;
                        double a3 = -2.*m/(totalTime*totalTime*totalTime) + n/(totalTime*totalTime);
                        
                        if(currenttime < totalTime){
                            pd = a3*currenttime*currenttime*currenttime + a2*currenttime*currenttime + a1*currenttime +a0;
                            pd_dot = 3.*a3*currenttime*currenttime + 2.*a2*currenttime + a1;
                            pd_ddot = 6.*a3*currenttime + 2.*a2;
                        }else{
                            pd = p1;
                            pd_dot = p1_dot;
                            pd_ddot = 0.0;//6.*a3*totalTime + 2.*a2;
                        }

                     }
void oneLegTest3(double t, double &vCmd, double &tStepPre, int &stIndex, Eigen::VectorXd xStand, Eigen::VectorXd &xInit, Eigen::VectorXd &xDotInit, 
                Eigen::VectorXd &xDDotInit, Eigen::VectorXd &xEnd, Eigen::VectorXd &xDotEnd, Eigen::VectorXd &xCmd, Eigen::VectorXd &xDotCmd, Eigen::VectorXd &xDDotCmd, Eigen::VectorXd &fCmd)
{   
    double timeStep = 0.0025;
    double tStep, s;
    double T = 0.25;
    double tTrans = 0.2;
    int M = 6, N= 6;
    bool startFlag = false;
    Eigen::VectorXd xPoly(N), xDotPoly(N), xDDotPoly(N), torPoly(N);
    double vZInit = 0.0, vZEnd = -0.0, zMid = 0.08;
    double fZ = -350.;
    double tForce = 0.02;

    tStep = fmod(t, T);
    if(T - tStep < 0.5*timeStep || tStep < 0.5*timeStep){    
        tStep = 0.0;
        stIndex = 1 - stIndex;
        startFlag = true;
        vCmd += 0.02;
        if (vCmd > 1.5){
            vCmd = 1.5;
        }
    }
    
    tStepPre = tStep;
    s = tStep/T;

    if(startFlag){
        xInit = xCmd;
        xDotInit = xDotCmd;
        xDDotInit = xDDotCmd;
    }

    if(stIndex==0){
        xEnd(0) = xStand(0)-vCmd*T*0.5; xDotEnd(0) = -vCmd;
        xEnd(1) = xStand(1); xDotEnd(1) = 0.0;
        xEnd(2) = xStand(2); xDotEnd(2) = vZInit;
        xEnd(3) = xStand(3)+vCmd*T*0.5; xDotEnd(3) = -vCmd;
        xEnd(4) = xStand(4); xDotEnd(4) = 0.0;
        xEnd(5) = xStand(5); xDotEnd(5) = vZEnd;
    }else{
        xEnd(3) = xStand(3)-vCmd*T*0.5; xDotEnd(3) = -vCmd;
        xEnd(4) = xStand(4); xDotEnd(4) = 0.0;
        xEnd(5) = xStand(5); xDotEnd(5) = vZInit;
        xEnd(0) = xStand(0)+vCmd*T*0.5; xDotEnd(0) = -vCmd;
        xEnd(1) = xStand(1); xDotEnd(1) = 0.0;
        xEnd(2) = xStand(2); xDotEnd(2) = vZEnd;
    }
    double xTrans = 0.0, xDotTrans = 0.0, xDDotTrans = 0.0;
    for(int i=0;i<N;++i){
        // Thirdpoly(xInit(i), xDotInit(i), xEnd(i), xDotEnd(i), T, tStep+timeStep, xTrans , xDotTrans);
        TwoPointsCubic(xInit(i), xDotInit(i), xEnd(i), xDotEnd(i), T, tStep+timeStep, xTrans , xDotTrans, xDDotTrans);
        xCmd(i) = xTrans;
        xDotCmd(i) = xDotTrans;
        xDDotCmd(i) = xDDotTrans;
        fCmd(i) = 0.0;
    }
    if(stIndex == 0){
        TriPointsQuintic(T, tStep+timeStep, xInit(5), xDotInit(5), zMid+xStand(5), 0., xStand(5), vZEnd, xTrans, xDotTrans, xDDotTrans);
        xCmd(5) = xTrans;
        xDotCmd(5) = xDotTrans;
        xDDotCmd(5) = xDDotTrans;
        // Thirdpoly(xInit(2), xDotInit(2), xEnd(2), 0.0, 0.1*T, tStep+timeStep, xTrans , xDotTrans);
        TwoPointsCubic(xInit(2), xDotInit(2), xEnd(2), 0.0, 0.1*T, tStep+timeStep, xTrans , xDotTrans, xDDotTrans);
        xCmd(2) = xTrans;
        xDotCmd(2) = xDotTrans;
        xDDotCmd(2) = xDDotTrans;
    }else{
        TriPointsQuintic(T, tStep+timeStep, xInit(2), xDotInit(2), zMid+xStand(2), 0., xStand(2), vZEnd, xTrans, xDotTrans, xDDotTrans);
        xCmd(2) = xTrans;
        xDotCmd(2) = xDotTrans;
        xDDotCmd(2) = xDDotTrans;
        // Thirdpoly(xInit(5), xDotInit(5), xEnd(5), 0.0, 0.1*T, tStep+timeStep, xTrans , xDotTrans);
        TwoPointsCubic(xInit(5), xDotInit(5), xEnd(5), 0.0, 0.1*T, tStep+timeStep, xTrans , xDotTrans, xDDotTrans);
        xCmd(5) = xTrans;
        xDotCmd(5) = xDotTrans;
        xDDotCmd(5) = xDDotTrans;
    }

    if(tStep < tForce){
        fCmd(3*stIndex+2) = tStep/tForce*fZ;
        fCmd(3*(1-stIndex)+2) = fZ - tStep/tForce*fZ;
    }
    else{
        fCmd(3*stIndex+2) = fZ;
        fCmd(3*(1-stIndex)+2) = 0.0;
    }
    
}
Eigen::Matrix3d rotX(double q) {
    Eigen::Matrix3d m_ret;
    double s = sin(q), c = cos(q);
    m_ret << 1.0, 0.0, 0.0,
             0.0,   c,  -s,
             0.0,   s,   c;
    return m_ret;
}
Eigen::Matrix3d rotY(double q) {
    Eigen::Matrix3d m_ret;
    double s = sin(q), c = cos(q);
    m_ret <<   c,  0.0,   s,
             0.0,  1.0, 0.0,
              -s,  0.0,   c;
    return m_ret;
}
Eigen::Matrix3d rotZ(double q) {
    Eigen::Matrix3d m_ret;
    double s = sin(q), c = cos(q);
    m_ret <<  c,  -s,  0.0,
              s,   c,  0.0,
            0.0,  0.0, 1.0;
    return m_ret;
}
void oneLegIK(Eigen::VectorXd Pos, Eigen::VectorXd RPY, Eigen::VectorXd &qIK){
    double l2 = 0.1405, l3 = 0.36, l4 = 0.34, l6 = 0.039; 

    double px = Pos(0), py = Pos(1), pz = Pos(2);
    Eigen::Matrix3d R = rotZ(RPY(2))*rotY(RPY(1))*rotX(RPY(0));
    double r11 = R(0,0), r22 = R(1,1), r33 = R(2,2);
    double r12 = R(0,1), r13 = R(0,2), r23 = R(1,2);
    double r21 = R(1,0), r31 = R(2,0), r32 = R(2,1);

    double q1,q2,q3,q4,q5,q6;
    double s1,s2,s3,s4,s5,s6;
    double c1,c2,c3,c4,c5,c6;
    
    q6 = atan((py*r22 + px*r12 + pz*r32) / (py*r23 + px*r13 + pz*r33 + l6));
    s6 = sin(q6); c6 = cos(q6);

    q1 = atan((r32*c6 - r33*s6) / (r22*c6 - r23*s6));
    s1 = sin(q1); c1 = cos(q1);

    q2 = asin(r13*s6 - r12*c6);
    s2 = sin(q2); c2 = cos(q2);

    Eigen::Vector3d PosHip(0.0, 0.0, -l2);
    Eigen::Vector3d PosFoot(0.0, 0.0, -l6);

    Eigen::Vector3d v1 = Pos - R*PosFoot;
    Eigen::Vector3d v2 = rotX(q1)*PosHip;
    Eigen::Vector3d v3 = v1 - v2;

    double dd = v3.transpose()*v3;
    q4 = M_PI- acos((l3*l3 + l4*l4 - dd)/(2*l3*l4));
    s4 = sin(q4); c4 = cos(q4);

    double A = l4*r11 + l3*r11*c4 + l3*r13*c6*s4 + l3*r12*s4*s6;
    double B = l3*r11*s4 - l4*r13*c6 - l4*r12*s6 - l3*r13*c4*c6 - l3*r12*c4*s6;
    double C = px + l6*r13;
    double D = sqrt(A*A + B*B - C*C);
    q5 = 2.0*atan2((A-D),(B+C));
    s5 = sin(q5); c5 = cos(q5);

    q3 = asin((r13*c4*c5*c6 - r11*c5*s4 -r11*c4*s5 + r12*c4*c5*s6 -r13*c6*s4*s5 - r12*s4*s5*s6)/(c2));
    
    qIK(0) = q1; qIK(1) = q2; qIK(2) = q3;
    qIK(3) = q4; qIK(4) = q5; qIK(5) = q6;
}
void oneLegIK2(Eigen::VectorXd Pos, Eigen::Matrix3d R, Eigen::VectorXd &qIK){
    double l2 = 0.1405, l3 = 0.36, l4 = 0.34, l6 = 0.039;

    double px = Pos(0), py = Pos(1), pz = Pos(2);

    double r11 = R(0,0), r22 = R(1,1), r33 = R(2,2);
    double r12 = R(0,1), r13 = R(0,2), r23 = R(1,2);
    double r21 = R(1,0), r31 = R(2,0), r32 = R(2,1);

    double q1,q2,q3,q4,q5,q6;
    double s1,s2,s3,s4,s5,s6;
    double c1,c2,c3,c4,c5,c6;
    
    q6 = atan((py*r22 + px*r12 + pz*r32) / (py*r23 + px*r13 + pz*r33 + l6));
    s6 = sin(q6); c6 = cos(q6);

    q1 = atan((r32*c6 - r33*s6) / (r22*c6 - r23*s6));
    s1 = sin(q1); c1 = cos(q1);

    q2 = asin(r13*s6 - r12*c6);
    s2 = sin(q2); c2 = cos(q2);

    Eigen::Vector3d PosHip(0.0, 0.0, -l2);
    Eigen::Vector3d PosFoot(0.0, 0.0, -l6);

    Eigen::Vector3d v1 = Pos - R*PosFoot;
    Eigen::Vector3d v2 = rotX(q1)*PosHip;
    Eigen::Vector3d v3 = v1 - v2;

    double dd = v3.transpose()*v3;
    q4 = M_PI- acos((l3*l3 + l4*l4 - dd)/(2*l3*l4));
    s4 = sin(q4); c4 = cos(q4);

    double A = l4*r11 + l3*r11*c4 + l3*r13*c6*s4 + l3*r12*s4*s6;
    double B = l3*r11*s4 - l4*r13*c6 - l4*r12*s6 - l3*r13*c4*c6 - l3*r12*c4*s6;
    double C = px + l6*r13;
    double D = sqrt(A*A + B*B - C*C);
    q5 = 2.0*atan2((A-D),(B+C));
    s5 = sin(q5); c5 = cos(q5);

    q3 = asin((r13*c4*c5*c6 - r11*c5*s4 -r11*c4*s5 + r12*c4*c5*s6 -r13*c6*s4*s5 - r12*s4*s5*s6)/(c2));
    
    qIK(0) = q1; qIK(1) = q2; qIK(2) = q3;
    qIK(3) = q4; qIK(4) = q5; qIK(5) = q6;
}
void wkSpace2Joint(Eigen::VectorXd xCmd, Eigen::VectorXd &qCmd, Eigen::VectorXd &qDotCmd, bool &firstFlag){
    double timeStep = 0.0025;
    Eigen::Vector3d offSetL(0.0, 0.105, 0.0), offSetR(0.0, -0.105, 0.0);
    Eigen::Vector3d PosL = xCmd.head(3) - offSetL, PosR = xCmd.tail(3) - offSetR;
    Eigen::VectorXd qIKL = Eigen::VectorXd::Zero(6), qIKR = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d RPY(0.0, 0.0, 0.0);
    Eigen::VectorXd qCmd_pre = qCmd;
    oneLegIK(PosL, RPY, qIKL);
    oneLegIK(PosR, RPY, qIKR);
    qCmd.head(6) = qIKL;
    qCmd.tail(6) = qIKR;
    if(!firstFlag)
        qDotCmd = (qCmd - qCmd_pre)/timeStep;
    firstFlag = false;
}
void wkSpace2Joint(Eigen::VectorXd xCmd, Eigen::VectorXd rpyCmd, Eigen::VectorXd &qCmd, Eigen::VectorXd &qDotCmd, bool &firstFlag){
    double timeStep = 0.0025;
    Eigen::Vector3d offSetL(0.0, 0.105, 0.0), offSetR(0.0, -0.105, 0.0);
    Eigen::Vector3d PosL, PosR;
    Eigen::VectorXd qIKL = Eigen::VectorXd::Zero(6), qIKR = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd qCmd_pre = qCmd;
    Eigen::Matrix3d rotR, rotL;
    rotL = (rotX(rpyCmd(0))*rotY(rpyCmd(1))*rotZ(rpyCmd(2))).transpose();
    rotR = (rotX(rpyCmd(3))*rotY(rpyCmd(4))*rotZ(rpyCmd(5))).transpose();
    PosL = rotL*xCmd.head(3) - offSetL;
    PosR = rotR*xCmd.tail(3) - offSetR;
    oneLegIK2(PosL, rotL, qIKL);
    oneLegIK2(PosR, rotR, qIKR);
    qCmd.head(6) = qIKL;
    qCmd.tail(6) = qIKR;
    if(!firstFlag)
        qDotCmd = (qCmd - qCmd_pre)/timeStep;
    firstFlag = false;
}
//logData
bool dataLog(Eigen::VectorXd &v, std::ofstream &f){
    for(int i=0;i<v.size();i++){
          f<<v[i]<<" ";
        }
        f<<std::endl;
    return true;
}

