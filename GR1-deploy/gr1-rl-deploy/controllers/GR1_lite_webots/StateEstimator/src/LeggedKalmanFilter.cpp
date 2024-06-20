#include "LeggedKalmanFilter.h"
#include <iostream>

LeggedKalmanFilter::LeggedKalmanFilter()
{

}

LeggedKalmanFilter::LeggedKalmanFilter ( Eigen::VectorXd d_SysNoise, Eigen::VectorXd d_MeaNoise, double dt, int nState, int nOutput, int nInput )
{
        numState = nState;
        numOutput = nOutput;
        numInput = nInput;    
        sysNoise = d_SysNoise;		    //standard deviation of system noise
        meaNoise = d_MeaNoise;		//standard deviation of measurement noise
        high_suspect_number = 100.0;

        estimation_last = Eigen::VectorXd::Zero(numState);
        estimation_now  = Eigen::VectorXd::Zero(numState);
        estimation_noise_std = 0.0*Eigen::MatrixXd::Identity(numState,numState); //P
        filter_noise_per = Eigen::MatrixXd::Identity(numState,numState); //P-
        filter_noise = Eigen::MatrixXd::Identity(numState,numOutput); //K

        A = Eigen::MatrixXd::Identity(numState,numState);
        A(3,2) = dt; 

        B = Eigen::MatrixXd::Zero(numState, numInput);
        B.block(0,0,numInput,numInput) = Eigen::MatrixXd::Identity(numInput,numInput)*dt;

        C = Eigen::MatrixXd::Zero(numOutput,numState);
        C.block(0,0,numState,numState) = Eigen::MatrixXd::Identity(numState,numState);
        C.block(numState,0,numState,numState) = Eigen::MatrixXd::Identity(numState,numState);

        Q = Eigen::MatrixXd::Identity(numState,numState);
        for(int i=0; i<numState; ++i){
            Q(i,i) = sysNoise(i)*dt;
        }

        R = Eigen::MatrixXd::Identity(numOutput,numOutput);
        for(int i=0; i<numOutput; ++i){
            R(i,i) = meaNoise(i);
        }

}

LeggedKalmanFilter::~LeggedKalmanFilter()
{

}

Eigen::VectorXd LeggedKalmanFilter::mFilter ( Eigen::VectorXd sigIn, Eigen::VectorXd aIn, Eigen::VectorXd trust)
{	

	//the first time ,set the origin values as the filter ones
	if (nit == 0)
	{
		estimation_last = sigIn.head(numState);
		nit++;
	}

    Eigen::MatrixXd Q_ = Q;
    Eigen::MatrixXd R_ = R;

    R_.block(0,0,4,4) = (1. + (1.-trust(0))*high_suspect_number)*R.block(0,0,4,4);
    R_.block(4,4,4,4) = (1. + (1.-trust(1))*high_suspect_number)*R.block(4,4,4,4);

    estimation_now = A*estimation_last + B*aIn;
    filter_noise_per = A*estimation_noise_std*A.transpose() + Q_;
    filter_noise = (filter_noise_per*C.transpose())*(C*filter_noise_per*C.transpose() + R_).completeOrthogonalDecomposition().pseudoInverse();
    estimation_now = estimation_now + filter_noise*(sigIn-C*estimation_now);
    estimation_noise_std = (Eigen::MatrixXd::Identity(numState,numState) - filter_noise*C)*filter_noise_per;
    estimation_noise_std = 0.5*estimation_noise_std + 0.5*estimation_noise_std.transpose();

	//update the last filter value for the next estimation
	estimation_last = estimation_now;

    //std::cout << "err: " << (sigIn- estimation_now).transpose()<< std::endl;
    return  estimation_now;

}//LeggedKalmanFilter
