#include "../include/Robot_Controller.h"
// #include "Robot_Controller.h"
// # define DEBUG

Robot_Controller::Robot_Controller() {
}

void Robot_Controller::init(QString path, double _dt) {
    _jason_path = path;
    // construct robot
    _robot_data = new Robot_Data();
    _robot_data->dt = _dt;
    // _robot_data->net_contact_force = Eigen::MatrixXd::Zero(6,1);
    // construct robotdata from jason
    _robot_data_constructor = new Robot_Constructor();
    _robot_data_constructor->robotconstructor(_jason_path, _robot_data);
    // estimator and solver
    _estimation_operator = new Estimation_Operator();
    _estimation_operator->init(_robot_data);
    // _landing_mpc = new Landing_mpc();
    // _landing_mpc->init(_robot_data);
    // to do :select wbc solver
    // _robot_data->wbcsolver = Wbc_Solver_type::WBIC;
    // _robot_data->wbcsolver = Wbc_Solver_type::WQP;
    // _wbc_solver =new wbc_solver_hqp();
    // _wbc_solver->init(_robot_data);
    // init_flag
    init_flag = false;
}

void Robot_Controller::init_DataPackage(DataPackage *data) {
    data->dim = _robot_data->ndof;
    data->dt = _robot_data->dt;
    data->q_a.setZero(_robot_data->ndof, 1);
    data->q_dot_a.setZero(_robot_data->ndof, 1);
    data->q_ddot_a.setZero(_robot_data->ndof, 1);
    data->tau_a.setZero(_robot_data->ndof, 1);
    //
    data->q_a_Waist.setZero();
    data->q_dot_a_Waist.setZero();
    data->tau_a_Waist.setZero();
    //
    data->q_a_Arm.setZero();
    data->q_dot_a_Arm.setZero();
    data->tau_a_Arm.setZero();
    // ft_sensor and imu
    int _nftsensor = 0;
    int _nimu = 0;
    std::vector<Sensor *>::iterator iter;
    for (iter = _robot_data->sensor_set.begin(); iter != _robot_data->sensor_set.end(); iter++) {
        if ((*iter)->type == sensor_type::FT_sensor) {
            _nftsensor = _nftsensor + 1;
        }

        if ((*iter)->type == sensor_type::Imu_sensor) {
            _nimu = _nimu + 1;
        }
    }
    data->ft_sensor.setZero(6, _nftsensor);
    data->imu_sensor.setZero(18, _nimu);
    std::vector<Task *>::iterator task_iter;
    for (task_iter = _robot_data->task_card_set.begin(); task_iter != _robot_data->task_card_set.end(); task_iter++) {
        Eigen::MatrixXd _n_task_desired;
        _n_task_desired.setZero(4, (*task_iter)->dim);
        data->task_desired_value.push_back(_n_task_desired);
        data->task_desired_contact_state.push_back(false);
    }
    data->q_c.setZero(_robot_data->ndof, 1);
    data->q_dot_c.setZero(_robot_data->ndof, 1);
    data->q_ddot_c.setZero(_robot_data->ndof, 1);
    data->tau_c.setZero(_robot_data->ndof, 1);
    data->q_factor.setZero(_robot_data->ndof - 6, 1);
    data->q_dot_factor.setZero(_robot_data->ndof - 6, 1);
    data->xyz_R_init = _robot_data->imu_init;
    data->NED_R_YPR0.setIdentity();
    data->NED_R_YPRa.setIdentity();
    data->imu_init_flag = false;
    //
    data->q_waist_c.setZero();
    data->q_dot_c_Waist.setZero();
    data->q_ddot_c_Waist.setZero();
    data->tau_c_Waist.setZero();
    //
    data->q_c_Arm.setZero();
    data->q_dot_c_Arm.setZero();
    data->q_ddot_c_Arm.setZero();
    data->tau_c_Arm.setZero();
}
void Robot_Controller::changecontrollerpara(int task_id, Eigen::MatrixXd _parachange) {
    std::vector<Task *>::iterator task_iter;
    for (task_iter = _robot_data->task_card_set.begin(); task_iter != _robot_data->task_card_set.end(); task_iter++) {
        if ((*task_iter)->task_id == task_id) {
            (*task_iter)->controller->para = _parachange;
            std::cout << " task " << task_id << ": controller para has been changed!" << std::endl;
        }
    }
}
// void Robot_Controller::run()
// {
//     // #ifdef DEBUG
//     // std::cout<<" run "<<std::endl;
//     // #endif
//     // // estimate update x_a
//     // _estimation_operator->task_state_update(_robot_data);
//     // #ifdef DEBUG
//     // std::cout<<std::endl<<" tau_ext "<< _robot_data->tau_ext.transpose()<<std::endl;
//     // // std::cout<<" _estimation_operator "<<std::endl;
//     // #endif
//     // // planner update x_d
//     // //..
//     // // solve update q_c
//     // _wbc_solver->wbc_sovle_(_robot_data);
//     // #ifdef DEBUG
//     //  std::cout<<" _wbc_solver "<<std::endl;
//     //  #endif

//     /******task update******/
//     _estimation_operator->task_state_update(_robot_data);
//     // todo
//     // _estimation_operator->task_state_update_x_a(_robot_data);
//     // _estimation_operator->task_state_update_x_c(_robot_data);
//     /*******qp problem construct******/

//     /*****hqp solve*****/
//     _wbc_solver->wbc_sovle_(_robot_data);
// }

void Robot_Controller::set_inputdata(DataPackage *data) {
    _robot_data->dt = data->dt;
    _robot_data->q_a = data->q_a;
    _robot_data->q_dot_a = data->q_dot_a;
    _robot_data->q_ddot_a = data->q_ddot_a;
    _robot_data->tau_a = data->tau_a;
    //
    _robot_data->q_a_Waist = data->q_a_Waist;
    _robot_data->q_dot_a_Waist = data->q_dot_a_Waist;
    _robot_data->tau_a_Waist = data->tau_a_Waist;
    //
    _robot_data->q_a_Arm = data->q_a_Arm;
    _robot_data->q_dot_a_Arm = data->q_dot_a_Arm;
    _robot_data->tau_a_Arm = data->tau_a_Arm;

    // integrate init
    if (init_flag == false) {
        _robot_data->q_c = _robot_data->q_a;
        _robot_data->q_dot_c = _robot_data->q_dot_a;
        _robot_data->q_ddot_c = _robot_data->q_ddot_a;
        _robot_data->tau_c = _robot_data->tau_a;
        //
        _robot_data->q_waist_c = _robot_data->q_a_Waist;
        _robot_data->q_dot_c_Waist = _robot_data->q_dot_a_Waist;
        _robot_data->tau_c_Waist = _robot_data->tau_a_Waist;
        //
        _robot_data->q_c_Arm = _robot_data->q_a_Arm;
        _robot_data->q_dot_c_Arm = _robot_data->q_dot_a_Arm;
        _robot_data->tau_c_Arm = _robot_data->tau_a_Arm;
        init_flag = true;
        std::cout << "init!" << std::endl;
    }

    std::vector<Sensor *>::iterator sensor_iter;
    int FTSensor_col = 0;
    int IMUSensor_col = 0;
    for (sensor_iter = _robot_data->sensor_set.begin();
         sensor_iter != _robot_data->sensor_set.end();
         sensor_iter++) {
        if ((*sensor_iter)->type == sensor_type::FT_sensor) {
            if (FTSensor_col < data->ft_sensor.cols()) {
                (*sensor_iter)->_data = data->ft_sensor.col(FTSensor_col);
                FTSensor_col++;
            }
        } else if ((*sensor_iter)->type == sensor_type::Imu_sensor) {
            if (IMUSensor_col < data->imu_sensor.cols()) {
                (*sensor_iter)->_data = data->imu_sensor.col(IMUSensor_col);
                IMUSensor_col++;
            }
        } else {
        }
    }

    std::vector<Task *>::iterator task_iter;
    for (task_iter = _robot_data->task_card_set.begin();
         task_iter != _robot_data->task_card_set.end();
         task_iter++) {
        (*task_iter)->X_d = data->task_desired_value[(*task_iter)->task_id - 1];
        (*task_iter)->contact_state_d = data->task_desired_contact_state[(*task_iter)->task_id - 1];
    }

    // imu data
    data->NED_R_YPR0 = basicfunction::RotX(M_PI);
    Eigen::Vector3d ypra = data->imu_sensor.block(0, 0, 3, 1);

    // Old version of yaw processing
    // yaw set 0
    // ypra(0) = 0.0;

    // New version of yaw processing
    // std::cout << "Before processing: " << ypra(0) << std::endl;
    // std::cout << "yaw_init: " << _robot_data->nlp_init << std::endl;
    // std::cout << "yaw_zero: " << yaw_zero << std::endl;
    // if (yaw_init == false)
    // {
    //     yaw_zero = ypra(0);
    //     yaw_init = true;
    // }
    // ypra(0) = ypra(0) - yaw_zero;

    if (_robot_data->nlp_init == true) {
        yaw_zero = ypra(0);
        _robot_data->nlp_init = false;
    }
    ypra(0) = ypra(0) - yaw_zero;

    // std::cout << "After processing: " << ypra(0) << std::endl;

    basicfunction::Euler_ZYXToMatrix(data->NED_R_YPRa, ypra);

    Eigen::Matrix3d xyz_delt_R_act_init = data->xyz_R_init * data->NED_R_YPR0.transpose() * data->NED_R_YPRa * data->xyz_R_init.transpose();

    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    basicfunction::matrixtoeulerxyz_(xyz_delt_R_act_init, rpy);

    // imu anzhaung pianzhi
    // rpy(0) = rpy(0) - 0.0;
    rpy(2) += _robot_data->rCmd_joystick_last(2);
    _robot_data->q_a.block(3, 0, 3, 1) = rpy;

    // std::cout<<"rpy: "<<std::endl<<rpy.transpose()<<std::endl;
    //
    Eigen::Matrix3d xyz_R_act = data->xyz_R_init * data->NED_R_YPR0.transpose() * data->NED_R_YPRa;
    // Eigen::Matrix3d xyz_R_act = data->xyz_R_init*data->NED_R_YPRa*data->NED_R_YPR0.transpose();
    Eigen::Matrix3d R_xyz_omega = Eigen::Matrix3d::Identity();
    R_xyz_omega.row(1) = basicfunction::RotX(rpy(0)).row(1);
    R_xyz_omega.row(2) = (basicfunction::RotX(rpy(0)) * basicfunction::RotY(rpy(1))).row(2);
    R_xyz_omega.row(3) = (basicfunction::RotX(rpy(0)) * basicfunction::RotY(rpy(1)) * basicfunction::RotZ(rpy(2))).row(3);

    _robot_data->q_dot_a.block(3, 0, 3, 1) = R_xyz_omega.transpose() * xyz_R_act * data->imu_sensor.block(3, 0, 3, 1);

    // _robot_data->q_dot_a.block(3, 0, 3, 1) = R_xyz_omega.transpose() * data->imu_sensor.block(3, 0, 3, 1);

    //
    _robot_data->imuAcc = data->imu_sensor.block(6, 0, 3, 1);
    // }
}

void Robot_Controller::get_outputdata(DataPackage *data) {
    data->q_a = _robot_data->q_a;
    data->q_dot_a = _robot_data->q_dot_a;
    data->q_ddot_a = _robot_data->q_ddot_a;
    //
    data->q_a_Waist = _robot_data->q_a_Waist;
    data->q_dot_a_Waist = _robot_data->q_dot_a_Waist;
    //
    data->q_a_Arm = _robot_data->q_a_Arm;
    data->q_dot_a_Arm = _robot_data->q_dot_a_Arm;
    //
    data->q_c = _robot_data->q_c;
    data->q_dot_c = _robot_data->q_dot_c;
    data->q_ddot_c = _robot_data->q_ddot_c;
    data->tau_c = _robot_data->tau_c;
    data->contact_force = _robot_data->contactforce;
    //
    data->q_waist_c = _robot_data->q_waist_c;
    data->q_dot_c_Waist = _robot_data->q_dot_c_Waist;
    data->tau_c_Waist = _robot_data->tau_c_Waist;
    //
    data->q_c_Arm = _robot_data->q_c_Arm;
    data->q_dot_c_Arm = _robot_data->q_dot_c_Arm;
    data->tau_c_Arm = _robot_data->tau_c_Arm;
    //
    data->q_factor = _robot_data->q_factor;
    data->q_dot_factor = _robot_data->q_dot_factor;
    //
    data->q_factor_Waist = _robot_data->q_factor_Waist;
    data->q_dot_factor_Waist = _robot_data->q_dot_factor_Waist;
    data->q_factor_Arm = _robot_data->q_factor_Arm;
    data->q_dot_factor_Arm = _robot_data->q_dot_factor_Arm;
    // std::cout<<"?"<<std::endl;
    // test ,gravity compensation + task space control
    // RigidBodyDynamics::Math::VectorNd CG = RigidBodyDynamics::Math::VectorNd::Zero(_robot_data->ndof, 1);
    // RigidBodyDynamics::NonlinearEffects(*(_robot_data->robot_model),data->q_a, data->q_dot_a,CG);
    // RigidBodyDynamics::Math::MatrixNd M = RigidBodyDynamics::Math::MatrixNd::Zero(_robot_data->ndof,_robot_data->ndof);
    // RigidBodyDynamics::CompositeRigidBodyAlgorithm(*(_robot_data->robot_model),data->q_a, M);
    // data->var_temp = CG; //+  _robot_data->task_card_set[0]->jacobi.transpose() *_robot_data->task_card_set[0]->X_c.row(2).transpose() +  _robot_data->task_card_set[1]->jacobi.transpose() *_robot_data->task_card_set[1]->X_c.row(2).transpose();
    //  data->var_temp = CG + M*data->q_ddot_c;
    //  std::cout << "CG : " << CG.transpose() << std::endl;
    //  std::cout << "tau_c : " << data->tau_c.transpose() << std::endl;
    //  std::cout << "q_ddot_c : " << data->q_ddot_c.transpose() << std::endl;
    //  std::cout << "M*data->q_ddot_c : " << ( M*data->q_ddot_c).transpose() << std::endl;
}
