#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H
#include "Robot_Data.h"
#include "Robot_Constructor.h"
#include "Estimation_Operator.h"
#include "basicfunction.h"
// #include "wbc_sovler_hqp.h"
#include "DataPackage.h"
// // #include "../../MPC_Landing/include/Landing_mpc.h"
// #include "Landing_mpc.h"
// #include "PlaningData.h"
#include <QString>
/**
 * @brief The Robot_Controller class: Perform the whole progress of the WBC control
 */
class Robot_Controller
{
public:
    Robot_Data *_robot_data;
    Robot_Constructor *_robot_data_constructor;
    Estimation_Operator *_estimation_operator;
    // wbc_solver_hqp * _wbc_solver;
    // Yaw
    bool yaw_init = false;
    double yaw_zero = 0.0;

    // jason path
    QString _jason_path;
    // init flag
    bool init_flag = false;
    // imu_init
    bool imu_first_input_flag;
    // imu_data
    Eigen::VectorXd imu_R_init;

public:
    // construct function
    Robot_Controller();
    // init function
    void init(QString path, double _dt);
    // init datapackage
    void init_DataPackage(DataPackage *data);
    // change the para of the controller
    void changecontrollerpara(int task_id, Eigen::MatrixXd _parachange);
    // input data
    void set_inputdata(DataPackage *data);
    // output data
    void get_outputdata(DataPackage *data);
    // run ----
    // void run();
};

#endif // ROBOT_CONTROLLER_H
