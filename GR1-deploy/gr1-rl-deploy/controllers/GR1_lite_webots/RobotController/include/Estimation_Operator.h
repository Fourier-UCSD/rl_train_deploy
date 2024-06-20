#ifndef ESTIMATION_OPERATOR_H
#define ESTIMATION_OPERATOR_H
#include "Robot_Data.h"
#include "LowPassFilter.h"
#include "../../StateEstimator/include/StateEstimate.h"
// add some filter method
//...
/**
 * @brief The Estimation_Operator class
 */
class Estimation_Operator
{
public:
    // construct function
    Estimation_Operator();
    ~Estimation_Operator();
    void init(Robot_Data * robotdata);
    // tasks' state update
    void task_state_update(Robot_Data * robotdata);
    
    // task actual state estimate
    void task_state_update_x_a(Robot_Data * robotdata);
    void task_state_update_x_a_walk(Robot_Data * robotdata);
    // task desired state estimate
    void task_state_update_x_d(Robot_Data * robotdata);

    // 
    // filter
    void lowpass();
    void FFT();
    void kalman();
    // residual external torque observer
    void externaltorqueobserver(Robot_Data * robotdata);
private:
    // external joint torque observer begin
    // external torque ovserver r_last and M_last
    Eigen::MatrixXd r_last;
    Eigen::MatrixXd H_last;
    // external torque observer gain
    double K;
    // integration
    Eigen::VectorXd Integ;
    //end
    LowPassFilter *q_dot_a_filter;
    // 
    StateEstimate *stateestimator;
};

#endif // ESTIMATION_OPERATOR_H
