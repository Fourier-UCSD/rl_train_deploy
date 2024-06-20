#include "../include/KalmanFilter.h"
// #include "KalmanFilter.h"
#include <iostream>

KalmanFilter::KalmanFilter()
{

}

KalmanFilter::KalmanFilter ( double d_SysNoise, double d_MeaNoise, int nFilter )
{
        numFilter = nFilter;    
        sysNoise = d_SysNoise;		    //standard deviation of system noise
        meaNoise = d_MeaNoise;		//standard deviation of measurement noise

        estimation_last = Eigen::VectorXd::Zero(numFilter);
        estimation_now  = Eigen::VectorXd::Zero(numFilter);
        estimation_noise_std = Eigen::VectorXd::Zero(numFilter);
}

KalmanFilter::~KalmanFilter()
{

}

Eigen::VectorXd KalmanFilter::mFilter ( Eigen::VectorXd sigIn )
{	

	//the first time ,set the origin values as the filter ones
	if (nit == 0)
	{
		estimation_last = sigIn;
		nit++;
	}

	//estimate the next velocity
	for (int i = 0; i < numFilter; i++)
	{
		estimation_now(i) = estimation_last(i);
		filter_noise_per = estimation_noise_std(i) + sysNoise;
		filter_noise = filter_noise_per / (filter_noise_per + meaNoise);
		estError = sigIn(i) - estimation_now(i);
		estimation_now(i) = estimation_now(i) + filter_noise * estError;
		estimation_noise_std(i) = (1 - filter_noise) * filter_noise_per;
	}

	//update the last filter value for the next estimation
	estimation_last = estimation_now;

    //std::cout << "err: " << (sigIn- estimation_now).transpose()<< std::endl;
    return  estimation_now;


}//KalmanFilter 
