#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <math.h>
#include <Eigen/Dense>


class KalmanFilter {
	public:
        KalmanFilter();
		KalmanFilter ( double d_SysNoise, double d_MeaNoise, int nFilter );
        ~KalmanFilter();
		Eigen::VectorXd mFilter ( Eigen::VectorXd sigIn );
		
	private:
        int numFilter = 1;    
        double sysNoise = 0.00f;		  //standard deviation of system noise
        double meaNoise = 0.2f;		//standard deviation of measurement noise
        double estError = 0.0f;		        	//estimation error

        double filter_noise_per = 0.0f;			
        double filter_noise = 0.0f;
         int nit = 0;

        Eigen::VectorXd estimation_last;
        Eigen::VectorXd estimation_now;
        Eigen::VectorXd estimation_noise_std;

};

#endif