#include "AccKalmanFilter.h"
#include <iostream>

AccKalmanFilter::AccKalmanFilter()
{

}

AccKalmanFilter::AccKalmanFilter ( double d_SysNoise, double d_MeaNoise, double dt, int nFilter )
{
        numFilter = nFilter;    
        sysNoise = d_SysNoise;		    //standard deviation of system noise
        meaNoise = d_MeaNoise;		//standard deviation of measurement noise
        high_suspect_number = 100.0;

        estimation_last = Eigen::VectorXd::Zero(numFilter);
        estimation_now  = Eigen::VectorXd::Zero(numFilter);
        estimation_noise_std = 0.0*Eigen::MatrixXd::Identity(numFilter,numFilter);
        filter_noise_per = Eigen::MatrixXd::Identity(numFilter,numFilter);
        filter_noise = Eigen::MatrixXd::Identity(numFilter,numFilter);

        A = Eigen::MatrixXd::Identity(numFilter,numFilter);
        B = Eigen::MatrixXd::Identity(numFilter,numFilter)*dt;
        C = Eigen::MatrixXd::Identity(numFilter,numFilter);
        Q = Eigen::MatrixXd::Identity(numFilter,numFilter)*sysNoise*dt*9.81/55.0;
        Q(3,3) = sysNoise*dt*9.81;
        R = Eigen::MatrixXd::Identity(numFilter,numFilter)*meaNoise;

}

AccKalmanFilter::~AccKalmanFilter()
{

}

Eigen::VectorXd AccKalmanFilter::mFilter ( Eigen::VectorXd sigIn, Eigen::VectorXd aIn, double trust)
{	

	//the first time ,set the origin values as the filter ones
	if (nit == 0)
	{
		estimation_last = sigIn;
		nit++;
	}

	//estimate the next velocity
    // estimation_now = A*estimation_last + B*aIn;
    // filter_noise_per = A*estimation_noise_std*A.transpose() + Q;
    // filter_noise = (filter_noise_per*C.transpose()).lu().solve(C*filter_noise_per*C.transpose() + R);
    // estimation_now = estimation_now + filter_noise*(sigIn-C*estimation_now);
    // estimation_noise_std = (Eigen::MatrixXd::Identity(numFilter,numFilter) - filter_noise*C)*filter_noise_per;

    Eigen::MatrixXd Q_ = (1. + (1.-trust)*high_suspect_number)*Q;
    Eigen::MatrixXd R_ = (1. + (1.-trust)*high_suspect_number)*R;

    estimation_now = estimation_last + B*aIn;
    filter_noise_per = estimation_noise_std + Q_;
    filter_noise = (filter_noise_per + R_).completeOrthogonalDecomposition().pseudoInverse()*(filter_noise_per);
    estimation_now = estimation_now + filter_noise*(sigIn-estimation_now);
    estimation_noise_std = (Eigen::MatrixXd::Identity(numFilter,numFilter) - filter_noise)*filter_noise_per;

	//update the last filter value for the next estimation
	estimation_last = estimation_now;

    //std::cout << "err: " << (sigIn- estimation_now).transpose()<< std::endl;
    return  estimation_now;

}//AccKalmanFilter 
