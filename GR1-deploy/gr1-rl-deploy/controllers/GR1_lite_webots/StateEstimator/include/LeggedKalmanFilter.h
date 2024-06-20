#ifndef LEGGEDKALMANFILTER_H
#define LEGGEDKALMANFILTER_H

#include <math.h>
#include <Eigen/Dense>


class LeggedKalmanFilter {
	public:
        LeggedKalmanFilter();
		LeggedKalmanFilter( Eigen::VectorXd d_SysNoise, Eigen::VectorXd d_MeaNoise, double dt, int nState, int nOutput, int nInput );
        ~LeggedKalmanFilter();
		Eigen::VectorXd mFilter ( Eigen::VectorXd sigIn, Eigen::VectorXd aIn, Eigen::VectorXd trust );
		
	private:
        int numState = 1;
        int numOutput = 1;
        int numInput = 1;    
        Eigen::VectorXd sysNoise;		  //standard deviation of system noise
        Eigen::VectorXd meaNoise;		//standard deviation of measurement noise
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