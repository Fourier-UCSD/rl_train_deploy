#ifndef ACCKALMANFILTER_H
#define ACCKALMANFILTER_H

#include <math.h>
#include <Eigen/Dense>


class AccKalmanFilter {
	public:
        AccKalmanFilter();
		AccKalmanFilter ( double d_SysNoise, double d_MeaNoise, double dt, int nFilter );
        ~AccKalmanFilter();
		Eigen::VectorXd mFilter ( Eigen::VectorXd sigIn, Eigen::VectorXd aIn, double trust );
		
	private:
        int numFilter = 1;    
        double sysNoise = 0.00f;		  //standard deviation of system noise
        double meaNoise = 0.2f;		//standard deviation of measurement noise
        double estError = 0.0f;		        	//estimation error
        double high_suspect_number = 100.0;
        int nit = 0;

        Eigen::MatrixXd filter_noise_per;			
        Eigen::MatrixXd filter_noise;
        
        Eigen::VectorXd estimation_last;
        Eigen::VectorXd estimation_now;
        Eigen::MatrixXd estimation_noise_std;

        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd C;

};

#endif