#include "../include/Estimation_Operator.h"
#include "../include/basicfunction.h"
// #include "Estimation_Operator.h"
// #include "basicfunction.h"
#include <iostream>
Estimation_Operator::Estimation_Operator()
{
}

Estimation_Operator::~Estimation_Operator()
{
    delete q_dot_a_filter;
}

void Estimation_Operator::init(Robot_Data *robotdata)
{

    r_last = Eigen::MatrixXd::Zero(robotdata->ndof, 1);

    H_last = Eigen::MatrixXd::Zero(robotdata->ndof, robotdata->ndof);

    K = 10.0;

    Integ = Eigen::MatrixXd::Zero(robotdata->ndof, 1);
    // remeber to change the sampling time when change
    int fs = floor(1 / (robotdata->dt * 2));
    std::cout << "fs: " << fs << std::endl;
    q_dot_a_filter = new LowPassFilter(fs, 0.707, robotdata->dt, 18);
    stateestimator = new StateEstimate();
}
// to be delete
void Estimation_Operator::task_state_update(Robot_Data *robotdata)
{
    // filter
    // robotdata->q_dot_a = q_dot_a_filter->mFilter(robotdata->q_dot_a);
    // kinematics update : compare the effectiveness lately
    RigidBodyDynamics::UpdateKinematicsCustom(*(robotdata->robot_model), &(robotdata->q_a), &(robotdata->q_dot_a), nullptr);
    // external torque observer
    externaltorqueobserver(robotdata);

    // update task card not for joint task
    for (std::vector<Task *>::iterator iter = robotdata->task_card_set.begin();
         iter != robotdata->task_card_set.end(); iter++)
    {
        switch ((*iter)->type)
        {
        case general_task:
        {
            // reference frame is 1 world
            // update x_a position
            (*iter)->X_a.setZero(4, (*iter)->dim);
            Eigen::Vector3d pa;
            pa = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                              (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Eigen::Matrix3d Ra;
            // Eigen::Vector3d Euler_ZYX;
            Eigen::Vector3d Euler_XYZ;
            Ra = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                             (*iter)->joint_id, false)
                     .transpose();
            // basicfunction::MatrixToEuler_ZYX(Ra,Euler_ZYX);
            basicfunction::MatrixToEuler_XYZ(Ra, Euler_XYZ);
            Eigen::Matrix<double, 6, 1> x_6d;
            // x_6d.block(0,0,3,1) = Euler_ZYX;
            x_6d.block(0, 0, 3, 1) = Euler_XYZ;
            x_6d.block(3, 0, 3, 1) = pa;
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_6d((*iter)->task_selection_matrix[i], 0);
            }
            // update jaccobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = J_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update jacobi_dotq_dot
            RigidBodyDynamics::Math::VectorNd Jdotqdot_6D = RigidBodyDynamics::CalcPointAcceleration6D(*(robotdata->robot_model), robotdata->q_a,
                                                                                                       robotdata->q_dot_a, RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof),
                                                                                                       (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Jdotqdot_6D.block(3, 0, 3, 1) = Jdotqdot_6D.block(3, 0, 3, 1) + robotdata->robot_model->gravity;
            //            (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi_dot_q_dot.row(i) = Jdotqdot_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            (*iter)->X_a.row(2) = ((*iter)->jacobi * robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // std::cout<<"jacobi:"<< std::endl<<(*iter)->jacobi<<std::endl;
            // std::cout<<"q_ddot_a:"<< std::endl<<robotdata->q_ddot_a.transpose()<<std::endl;
            // std::cout<<"jacobi_dot_q_dot:"<< std::endl<<(*iter)->jacobi_dot_q_dot.transpose()<<std::endl;
            // update F: if the task has no sensor , F will  keep zero
            // compute controller value
            (*iter)->controller->setinput_data((*iter)->X_a, (*iter)->X_d, (*iter)->dim, robotdata->dt);
            (*iter)->controller->controller_run();
            (*iter)->controller->getoutput_data((*iter)->X_c);
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        case com_task:
        {
            // reference frame is 1 world
            // std::cout<<"??";
            // compute rotation matrix from floating base to world
            Eigen::Matrix3d Ra_F2I;
            Eigen::Vector3d Euler_XYZ;
            Ra_F2I = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                                 (*iter)->joint_id, false)
                         .transpose();
            basicfunction::MatrixToEuler_XYZ(Ra_F2I, Euler_XYZ);
            // compute jacobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            // compute PS1
            Eigen::MatrixXd PH1;
            PH1.setZero(6, 6);
            PH1.block(0, 0, 3, 3) = Ra_F2I.transpose();
            PH1.block(3, 3, 3, 3) = Ra_F2I.transpose();
            PH1 = PH1 * J_6D.block(0, 0, 6, 6);
            Eigen::MatrixXd PS1 = PH1.inverse();

            // compute H
            RigidBodyDynamics::Math::MatrixNd H;
            H.setZero(robotdata->ndof, robotdata->ndof);
            RigidBodyDynamics::Math::VectorNd _q_a = robotdata->q_a;
            RigidBodyDynamics::CompositeRigidBodyAlgorithm(*(robotdata->robot_model), _q_a, H, false);
            // define U1
            Eigen::MatrixXd U1;
            U1.setZero(6, robotdata->ndof);
            U1.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
            // H11
            Eigen::MatrixXd H11 = U1 * H * U1.transpose();
            // I_1^C
            Eigen::MatrixXd I1C = PS1.transpose() * H11 * PS1;
            // M
            double M = I1C(5, 5);
            // 1^P_G
            Eigen::Vector3d _1PG;
            _1PG(0) = I1C(2, 4) / M;
            _1PG(1) = I1C(0, 5) / M;
            _1PG(2) = I1C(1, 3) / M;
            // I^P_G
            Eigen::Vector3d _IP1;
            Eigen::Vector3d _IPG;
            _IP1 = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                                (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            _IPG = _IP1 + Ra_F2I * _1PG;
            // _1XTG
            Eigen::MatrixXd _1XTG;
            _1XTG.setZero(6, 6);
            _1XTG.block(0, 0, 3, 3) = PH1.block(0, 3, 3, 3);
            _1XTG.block(3, 3, 3, 3) = PH1.block(3, 0, 3, 3);
            _1XTG.block(0, 3, 3, 3) = PH1.block(0, 3, 3, 3) * (basicfunction::skew(_1PG)).transpose();
            // AG
            Eigen::MatrixXd AG = _1XTG * PS1.transpose() * U1 * H;
            // AG_dot*q_dot
            // Cq_dot G
            //                Eigen::VectorXd CG;
            RigidBodyDynamics::Math::VectorNd CG;
            CG.setZero(robotdata->ndof);
            //                Eigen::VectorXd G;
            RigidBodyDynamics::Math::VectorNd G;
            G.setZero(robotdata->ndof);
            //                Eigen::VectorXd C;
            RigidBodyDynamics::Math::VectorNd C;
            C.setZero(robotdata->ndof);
            //                Eigen::MatrixXd QDotZero = robotdata->q_dot_a;
            RigidBodyDynamics::Math::VectorNd QDotZero = robotdata->q_dot_a;
            QDotZero.setZero();
            _q_a = robotdata->q_a;
            RigidBodyDynamics::Math::VectorNd _q_dot_a = robotdata->q_dot_a;
            RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, _q_dot_a, CG);
            RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, QDotZero, G);
            C = CG - G;
            Eigen::MatrixXd AGdotqdot = _1XTG * PS1.transpose() * U1 * C;
            // I_G^3
            Eigen::MatrixXd IGC = _1XTG.transpose() * I1C * _1XTG;
            (*iter)->IG = IGC;
            // update x_a
            Eigen::Matrix<double, 1, 6> x_a_row;
            x_a_row.block(0, 0, 3, 1) = Euler_XYZ;
            x_a_row.block(0, 3, 1, 3) = _IPG.transpose();
            (*iter)->X_a.setZero(4, (*iter)->dim);
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_a_row(0, (*iter)->task_selection_matrix[i]);
            }
            // update jaccobi
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // std::cout<<"??";

            // whole model error: should use IG
            Eigen::MatrixXd IG_inv = IGC.inverse();
            AG = IG_inv * AG;
            AGdotqdot = IG_inv * AGdotqdot;
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = AG.row((*iter)->task_selection_matrix[i]);
            }
            // std::cout<<"??";
            // update jacobi_dot
            // select the right direction
            (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim, 1);
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi_dot_q_dot(i) = AGdotqdot((*iter)->task_selection_matrix[i]);
            }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            (*iter)->X_a.row(2) = ((*iter)->jacobi * robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // update F: if the task has no sensor , F will  keep zero
            (*iter)->X_a.row(3).setZero();
            // compute controller value
            (*iter)->controller->setinput_data((*iter)->X_a, (*iter)->X_d, (*iter)->dim, robotdata->dt);
            // linear moment control

            // todo change to kinematics
            (*iter)->controller->controller_run();
            // angular moment control
            Eigen::MatrixXd para_ = (*iter)->controller->para;
            // if((*iter)->task_selection_matrix[0] == task_direction::task_x_theta){
            //     (*iter)->X_c.row(2).block(0,0,1,3) = (*iter)->X_d.row(2).block(0,0,1,3) + (*iter)->controller->para(1)*((*iter)->X_d.row(1).block(0,0,1,3) - (*iter)->X_a.row(1).block(0,0,1,3));
            // }
            (*iter)->X_c.row(2)(0) = (*iter)->X_d.row(2)(0) + para_(6) * ((*iter)->X_d.row(1)(0) - (*iter)->X_a.row(1)(0));
            (*iter)->X_c.row(2)(1) = (*iter)->X_d.row(2)(1) + para_(7) * ((*iter)->X_d.row(1)(1) - (*iter)->X_a.row(1)(1));
            (*iter)->X_c.row(2)(2) = (*iter)->X_d.row(2)(2) + para_(8) * ((*iter)->X_d.row(1)(2) - (*iter)->X_a.row(1)(2));
            // linear moment control
            // (*iter)->X_c.row(2).block(0,3,1,3) = (*iter)->X_d.row(2).block(0,3,1,3) + (*iter)->controller->para(2)*((*iter)->X_d.row(1).block(0,3,1,3) - (*iter)->X_a.row(1).block(0,3,1,3))
            // + M * (*iter)->controller->para(0)*((*iter)->X_d.row(0).block(0,3,1,3) - (*iter)->X_a.row(0).block(0,3,1,3));
            (*iter)->X_c.row(2)(3) = (*iter)->X_d.row(2)(3) + para_(9) * ((*iter)->X_d.row(1)(3) - (*iter)->X_a.row(1)(3)) + M * para_(3) * ((*iter)->X_d.row(0)(3) - (*iter)->X_a.row(0)(3));
            (*iter)->X_c.row(2)(4) = (*iter)->X_d.row(2)(4) + para_(10) * ((*iter)->X_d.row(1)(4) - (*iter)->X_a.row(1)(4)) + M * para_(4) * ((*iter)->X_d.row(0)(4) - (*iter)->X_a.row(0)(4));
            (*iter)->X_c.row(2)(5) = (*iter)->X_d.row(2)(5) + para_(11) * ((*iter)->X_d.row(1)(5) - (*iter)->X_a.row(1)(5)) + M * para_(5) * ((*iter)->X_d.row(0)(5) - (*iter)->X_a.row(0)(5));

            // transform to momentum
            // simple model
            // (*iter)->X_c.row(2).block(0,3,1,3) = M*(*iter)->X_c.row(2).block(0,3,1,3);
            // whole model
            // (*iter)->X_c.row(2) = (I1C*((*iter)->X_c.row(2).transpose())).transpose();

            // y linear direction reduce
            // (*iter)->X_c.row(2).block(0,4,1,1) = 0.5*(*iter)->X_c.row(2).block(0,4,1,1);
            // (*iter)->controller->getoutput_data((*iter)->X_c);
            // compute desired contact wrench
            Eigen::MatrixXd bias = Eigen::MatrixXd::Zero(6, 1);
            bias.block(3, 0, 3, 1) = robotdata->robot_model->gravity;
            robotdata->net_contact_force = (*iter)->X_c.row(2).transpose() - M * bias;
            robotdata->G = M * bias;
            // std::cout<<"X_a(com): "<<std::endl<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d(com): "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c(com): "<<(*iter)->X_c<<std::endl;
            break;
            //            break;
        }
        case contact_task:
        {
            // reference frame is 1 world
            // update x_a position
            (*iter)->X_a.setZero(4, (*iter)->dim);
            Eigen::Vector3d pa;
            pa = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                              (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Eigen::Matrix3d Ra;
            // Eigen::Vector3d Euler_ZYX;
            Eigen::Vector3d Euler_XYZ;
            Ra = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                             (*iter)->joint_id, false)
                     .transpose();
            // basicfunction::MatrixToEuler_ZYX(Ra,Euler_ZYX);
            basicfunction::MatrixToEuler_XYZ(Ra, Euler_XYZ);
            Eigen::Matrix<double, 6, 1> x_6d;
            // x_6d.block(0,0,3,1) = Euler_ZYX;
            x_6d.block(0, 0, 3, 1) = Euler_XYZ;
            x_6d.block(3, 0, 3, 1) = pa;
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_6d((*iter)->task_selection_matrix[i], 0);
            }
            // update jaccobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = J_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update jacobi_dotq_dot
            RigidBodyDynamics::Math::VectorNd Jdotqdot_6D = RigidBodyDynamics::CalcPointAcceleration6D(*(robotdata->robot_model), robotdata->q_a,
                                                                                                       robotdata->q_dot_a, RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof),
                                                                                                       (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Jdotqdot_6D.block(3, 0, 3, 1) = Jdotqdot_6D.block(3, 0, 3, 1) + robotdata->robot_model->gravity;
            //             (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi_dot_q_dot.row(i) = Jdotqdot_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            (*iter)->X_a.row(2) = ((*iter)->jacobi * robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // update F: if the task has no sensor , F will  keep zero
            if ((*iter)->sensor_id != 0)
            {
                int id = 0;
                for (uint sensor_i = 0; sensor_i < robotdata->sensor_set.size(); sensor_i++)
                {
                    if (robotdata->sensor_set[sensor_i]->ID == (*iter)->sensor_id)
                    {
                        id = sensor_i;
                    }
                }
                switch (robotdata->sensor_set[id]->type)
                {
                case FT_sensor:
                {
                    if (robotdata->sensor_set[id]->link_num != (*iter)->joint_id)
                    {
                        std::cout << "Warning: FT_sensor link num does not match the task joint_id!" << std::endl;
                    }
                    else
                    {
                        Eigen::Matrix<double, 6, 1> wrench = robotdata->sensor_set[id]->_data;
                        RigidBodyDynamics::Math::SpatialTransform X_sensortolink(robotdata->sensor_set[id]->T_offset.block(0, 0, 3, 3),
                                                                                 -robotdata->sensor_set[id]->T_offset.block(0, 0, 3, 3).transpose() * robotdata->sensor_set[id]->T_offset.block(0, 3, 3, 1));
                        RigidBodyDynamics::Math::SpatialTransform X_tasktolink((*iter)->T_offset.block(0, 0, 3, 3),
                                                                               -(*iter)->T_offset.block(0, 0, 3, 3).transpose() * ((*iter)->T_offset.block(0, 3, 3, 1)));
                        RigidBodyDynamics::Math::SpatialTransform X_sensortotask = X_sensortolink * (X_tasktolink.inverse());
                        // compute wrench in task
                        wrench = X_sensortotask.applyAdjoint(wrench);
                        // represent the wrench in world frame
                        Eigen::Matrix3d Ra_task = Ra * (*iter)->T_offset.block(0, 0, 3, 3);
                        Eigen::Matrix<double, 6, 6> R_tasktoworld;
                        R_tasktoworld.setZero();
                        R_tasktoworld.block(0, 0, 3, 3) = Ra_task;
                        R_tasktoworld.block(3, 3, 3, 3) = Ra_task;
                        Eigen::Matrix<double, 6, 1> Force = R_tasktoworld * wrench;
                        // select the right direction
                        for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
                        {
                            (*iter)->X_a(3, i) = Force((*iter)->task_selection_matrix[i], 0);
                        }
                    }
                    break;
                }
                case Joint_external_torque_observer:
                {
                    Eigen::MatrixXd jacobi_T_inv = ((*iter)->jacobi * (*iter)->jacobi.transpose()).completeOrthogonalDecomposition().pseudoInverse() * (*iter)->jacobi;
                    (*iter)->X_a.row(3) = (jacobi_T_inv * robotdata->sensor_set[id]->_data).transpose();
                    //                        std::cout<<"(*iter)->X_a.row(3): "<<(*iter)->X_a.row(3)<<std::endl;
                    break;
                }
                default:
                {
                    std::cout << " Sensor type does not match!" << std::endl;
                    break;
                }
                }
            }
            // update tau external
            robotdata->tau_ext = robotdata->tau_ext - (*iter)->jacobi.transpose() * (*iter)->X_a.row(3).transpose();
            // compute controller value
            (*iter)->controller->setinput_data((*iter)->X_a, (*iter)->X_d, (*iter)->dim, robotdata->dt);
            (*iter)->controller->controller_run();
            (*iter)->controller->getoutput_data((*iter)->X_c);
            // if((*iter)->contact_state_d == true){
            //     (*iter)->X_c.row(2).setZero();
            // }
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        default:
        {
            break;
        }
        }
    }
    // update joint task
    for (std::vector<Task *>::iterator iter = robotdata->task_card_set.begin(); iter != robotdata->task_card_set.end(); iter++)
    {
        switch ((*iter)->type)
        {
        case joint_task:
        {
            if (robotdata->robottype == robot_type::Fixed_Base_Open_Chain)
            {
                // update jacobi
                (*iter)->jacobi.setIdentity();
                // update jacobi_dot
                //            (*iter)->jacobi_dot_q_dot.setZero(robotdata->ndof,robotdata->ndof);
                // update x_a position velocity acc
                (*iter)->X_a.row(0) = robotdata->q_a.transpose();
                (*iter)->X_a.row(1) = robotdata->q_dot_a.transpose();
                (*iter)->X_a.row(2) = robotdata->q_ddot_a.transpose();
                // update F
                (*iter)->X_a.row(3) = robotdata->tau_ext.transpose();
            }
            else if ((robotdata->robottype == robot_type::Float_Base_Open_Chain) || (robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain))
            {
                // update jacobi
                int _dim = robotdata->ndof - 6;
                int start_row = 6;
                (*iter)->jacobi.setZero();
                (*iter)->jacobi.block(0, start_row, _dim, _dim).setIdentity();
                // for dynamic herachical
                if (robotdata->wbcsolver == Wbc_Solver_type::hqp_dynamics)
                {
                    (*iter)->jacobi = (*iter)->jacobi * sqrt(1.0 + robotdata->dt * robotdata->dt);
                    if (robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain)
                    {
                        (*iter)->jacobi.block(0, start_row, robotdata->ndof_wheel, robotdata->ndof_wheel) = Eigen::MatrixXd::Identity(robotdata->ndof_wheel, robotdata->ndof_wheel) * robotdata->dt;
                    }
                }
                // end
                // update jacobi_dot
                //            (*iter)->jacobi_dot_q_dot.setZero(robotdata->ndof,robotdata->ndof);
                // update x_a position velocity acc
                (*iter)->X_a.row(0) = robotdata->q_a.block(start_row, 0, _dim, 1).transpose();
                (*iter)->X_a.row(1) = robotdata->q_dot_a.block(start_row, 0, _dim, 1).transpose();
                (*iter)->X_a.row(2) = robotdata->q_ddot_a.block(start_row, 0, _dim, 1).transpose();
                // update F
                (*iter)->X_a.row(3) = robotdata->tau_ext.block(start_row, 0, _dim, 1).transpose();
            }

            //             // update jacobi
            //             (*iter)->jacobi.setIdentity(robotdata->ndof,robotdata->ndof);
            //             // update jacobi_dot
            // //            (*iter)->jacobi_dot_q_dot.setZero(robotdata->ndof,robotdata->ndof);
            //             // update x_a position velocity acc
            //             (*iter)->X_a.row(0) = robotdata->q_a.transpose();
            //             (*iter)->X_a.row(1) = robotdata->q_dot_a.transpose();
            //             (*iter)->X_a.row(2) = robotdata->q_ddot_a.transpose();
            //             // update F
            //             (*iter)->X_a.row(3) = robotdata->tau_ext.transpose();
            //            if((*iter)->sensor_id != 0)
            //            {
            //                if(robotdata->sensor_set[(*iter)->sensor_id]->type == Joint_external_torque_observer)
            //                {
            //                   (*iter)->X_a.row(3) = robotdata->tau_ext.transpose();
            //                }else{
            //                    (*iter)->X_a.row(3) = robotdata->tau_ext.transpose();
            //                }
            //            }else{
            //                (*iter)->X_a.row(3) = robotdata->tau_ext.transpose();
            //            }
            // compute controller value
            // compute controller value
            (*iter)->controller->setinput_data((*iter)->X_a, (*iter)->X_d, (*iter)->dim, robotdata->dt);
            (*iter)->controller->controller_run();
            (*iter)->controller->getoutput_data((*iter)->X_c);
            // for dynamic herachical
            if (robotdata->wbcsolver == Wbc_Solver_type::hqp_dynamics)
            {
                if (robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain)
                {
                    (*iter)->X_c.block(2, 0, 1, robotdata->ndof_wheel).setZero();
                }
                (*iter)->X_c.row(2) = (*iter)->X_c.row(2) + (*iter)->X_a.row(1) * robotdata->dt;
            }
            // end
            std::cout << "X_a(joint): " << (*iter)->X_a << std::endl;
            std::cout << "X_d(joint): " << (*iter)->X_d << std::endl;
            std::cout << "X_c(joint): " << (*iter)->X_c << std::endl;
            break;
        }
        default:
            break;
        }
    }
}

void Estimation_Operator::externaltorqueobserver(Robot_Data *robotdata)
{
    // H
    RigidBodyDynamics::Math::MatrixNd H = RigidBodyDynamics::Math::MatrixNd::Zero(robotdata->ndof, robotdata->ndof);
    RigidBodyDynamics::Math::VectorNd _q_a = robotdata->q_a;
    RigidBodyDynamics::Math::VectorNd _q_dot_a = robotdata->q_dot_a;

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*(robotdata->robot_model), _q_a, H, false);
    // momentum P = Hq_dot
    Eigen::VectorXd P;
    P = H * robotdata->q_dot_a;
#ifdef DEBUG
    std::cout << " P  " << P.transpose() << std::endl;
#endif
    // C and G
    RigidBodyDynamics::Math::VectorNd CG = RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof, 1);
    RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, _q_dot_a, CG);
#ifdef DEBUG
    std::cout << " CG  " << CG.transpose() << std::endl;
    std::cout << " _q_dot_a  " << _q_dot_a.transpose() << std::endl;
#endif
    // H_dot
    Eigen::MatrixXd H_dot;
    static bool flag_update = 0;
    if (flag_update == 0)
    {
        H_last = H;
        flag_update = 1;
    }
    H_dot = (H - H_last) / robotdata->dt;
    H_last = H;
    // U
    Eigen::VectorXd U;
    U = H_dot * robotdata->q_dot_a + robotdata->tau_a - CG;
#ifdef DEBUG
    std::cout << " U  " << U.transpose() << std::endl;
#endif
    // joint external torque
    Integ += (U + r_last) * robotdata->dt;
    robotdata->tau_ext = K * (P - Integ);
    r_last = robotdata->tau_ext;
    // std::cout<<"tau_ext: "<<robotdata->tau_ext.transpose()<<std::endl;
    // test
    robotdata->tau_ext.setZero();
}

void Estimation_Operator::task_state_update_x_a(Robot_Data *robotdata)
{
    // filter
    // robotdata->q_dot_a = q_dot_a_filter->mFilter(robotdata->q_dot_a);
    // kinematics update : compare the effectiveness lately
    //    RigidBodyDynamics::UpdateKinematics(robotdata->robot_model,robotdata->q_a,robotdata->q_dot_a,robotdata->q_ddot_a);
    // Eigen::Quaternion()
    // update foot jacobi
    for (std::vector<Task *>::iterator iter = robotdata->task_card_set.begin();
         iter != robotdata->task_card_set.end(); iter++)
    {
        switch ((*iter)->type)
        {
        case general_task:
        {
            // reference frame is 1 world
            // update x_a position
            (*iter)->X_a.setZero(4, (*iter)->dim);
            Eigen::Vector3d pa;
            pa = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                              (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Eigen::Matrix3d Ra;
            Eigen::Vector3d Euler_XYZ;
            Ra = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                             (*iter)->joint_id, false)
                     .transpose();
            // std::cout<<"Ra: "<<std::endl<<Ra<<std::endl;
            // basicfunction::MatrixToEuler_ZYX(Ra,Euler_ZYX);
            basicfunction::MatrixToEuler_XYZ(Ra, Euler_XYZ);
            // std::cout<<"Euler_XYZ: "<<std::endl<<Euler_XYZ.transpose()<<std::endl;
            // Eigen::Matrix3d R_test;
            // basicfunction::Euler_XYZToMatrix(R_test,Euler_XYZ);
            // std::cout<<"R_test: "<<std::endl<<R_test<<std::endl;
            Eigen::Matrix<double, 6, 1> x_6d;
            // x_6d.block(0,0,3,1) = Euler_ZYX;
            x_6d.block(0, 0, 3, 1) = Euler_XYZ;
            x_6d.block(3, 0, 3, 1) = pa;
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_6d((*iter)->task_selection_matrix[i], 0);
            }
            // update jaccobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            // std::cout<<"q_a: "<<std::endl<<robotdata->q_a.transpose()<<std::endl;
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            // std::cout<<"J_6D: "<<std::endl<<J_6D<<std::endl;
            // std::cout<<"T_offset: "<<std::endl<<(*iter)->T_offset<<std::endl;
            // std::cout<<"joint_id: "<<std::endl<<(*iter)->joint_id<<std::endl;
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = J_6D.row((*iter)->task_selection_matrix[i]);
            }
            //          // update jacobi_dotq_dot
            //             RigidBodyDynamics::Math::VectorNd Jdotqdot_6D = RigidBodyDynamics::CalcPointAcceleration6D(*(robotdata->robot_model),robotdata->q_a,
            //                                                        robotdata->q_dot_a,RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof),
            //                                                        (*iter)->joint_id,(*iter)->T_offset.block(0,3,3,1),false);
            //             Jdotqdot_6D.block(3,0,3,1) = Jdotqdot_6D.block(3,0,3,1) + robotdata->robot_model->gravity;
            // //            (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,robotdata->ndof);
            //             // select the right direction
            //             for(uint i = 0; i < (*iter)->task_selection_matrix.size();i++)
            //             {
            //                 (*iter)->jacobi_dot_q_dot.row(i) = Jdotqdot_6D.row((*iter)->task_selection_matrix[i]);
            //             }
            // update x_a velocity
            //  std::cout<<"J: "<<std::endl<<(*iter)->jacobi<<std::endl;
            //  std::cout<<"q_dot_a: "<<std::endl<<robotdata->q_dot_a.transpose()<<std::endl;
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            // (*iter)->X_a.row(2) = ((*iter)->jacobi*robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // std::cout<<"jacobi:"<< std::endl<<(*iter)->jacobi<<std::endl;
            // std::cout<<"q_ddot_a:"<< std::endl<<robotdata->q_ddot_a.transpose()<<std::endl;
            // std::cout<<"jacobi_dot_q_dot:"<< std::endl<<(*iter)->jacobi_dot_q_dot.transpose()<<std::endl;
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        case com_task:
        {
            // reference frame is 1 world
            // std::cout<<"??";
            // compute rotation matrix from floating base to world
            Eigen::Matrix3d Ra_F2I;
            Eigen::Vector3d Euler_XYZ;
            Ra_F2I = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                                 (*iter)->joint_id, false)
                         .transpose();
            basicfunction::MatrixToEuler_XYZ(Ra_F2I, Euler_XYZ);
            // compute jacobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            // compute PS1
            Eigen::MatrixXd PH1;
            PH1.setZero(6, 6);
            // PH1.block(0,0,3,3) = Ra_F2I.transpose();
            // PH1.block(3,3,3,3) = Ra_F2I.transpose();
            PH1 = J_6D.block(0, 0, 6, 6);
            Eigen::MatrixXd PS1 = PH1.inverse();

            // compute H
            RigidBodyDynamics::Math::MatrixNd H;
            H.setZero(robotdata->ndof, robotdata->ndof);
            RigidBodyDynamics::Math::VectorNd _q_a = robotdata->q_a;
            RigidBodyDynamics::CompositeRigidBodyAlgorithm(*(robotdata->robot_model), _q_a, H, false);
            // define U1
            Eigen::MatrixXd U1;
            U1.setZero(6, robotdata->ndof);
            U1.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
            // H11
            Eigen::MatrixXd H11 = U1 * H * U1.transpose();
            // I_1^C
            Eigen::MatrixXd I1C = PS1.transpose() * H11 * PS1;
            // M
            double M = I1C(5, 5);
            // robotdata->robot_M = M;
            // 1^P_G
            Eigen::Vector3d _1PG;
            _1PG(0) = I1C(2, 4) / M;
            _1PG(1) = I1C(0, 5) / M;
            _1PG(2) = I1C(1, 3) / M;
            // I^P_G
            Eigen::Vector3d _IP1;
            Eigen::Vector3d _IPG;
            _IP1 = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                                (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            _IPG = _IP1 + Ra_F2I * _1PG;
            // _1XTG
            Eigen::MatrixXd _1XTG;
            _1XTG.setZero(6, 6);
            _1XTG.block(0, 0, 3, 3) = PH1.block(0, 3, 3, 3);
            _1XTG.block(3, 3, 3, 3) = PH1.block(3, 0, 3, 3);
            _1XTG.block(0, 3, 3, 3) = PH1.block(0, 3, 3, 3) * (basicfunction::skew(_1PG)).transpose();
            // AG
            Eigen::MatrixXd AG = _1XTG * PS1.transpose() * U1 * H;
            // AG_dot*q_dot
            // Cq_dot G
            //                Eigen::VectorXd CG;
            RigidBodyDynamics::Math::VectorNd CG;
            CG.setZero(robotdata->ndof);
            //                Eigen::VectorXd G;
            RigidBodyDynamics::Math::VectorNd G;
            G.setZero(robotdata->ndof);
            //                Eigen::VectorXd C;
            RigidBodyDynamics::Math::VectorNd C;
            C.setZero(robotdata->ndof);
            //                Eigen::MatrixXd QDotZero = robotdata->q_dot_a;
            RigidBodyDynamics::Math::VectorNd QDotZero = robotdata->q_dot_a;
            QDotZero.setZero();
            _q_a = robotdata->q_a;
            RigidBodyDynamics::Math::VectorNd _q_dot_a = robotdata->q_dot_a;
            RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, _q_dot_a, CG);
            RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, QDotZero, G);
            C = CG - G;
            Eigen::MatrixXd AGdotqdot = _1XTG * PS1.transpose() * U1 * C;
            // I_G^3
            // std::cout<<"I1C: "<<std::endl<<I1C<<std::endl;
            Eigen::MatrixXd IGC = _1XTG * I1C * _1XTG.transpose();
            (*iter)->IG = IGC;
            // update x_a
            Eigen::Matrix<double, 1, 6> x_a_row;
            x_a_row.block(0, 0, 1, 3) = Euler_XYZ.transpose();
            x_a_row.block(0, 3, 1, 3) = _IPG.transpose();
            (*iter)->X_a.setZero(4, (*iter)->dim);
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_a_row(0, (*iter)->task_selection_matrix[i]);
            }
            // update jaccobi
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);

            // whole model
            // std::cout<<"IG: "<<std::endl<<IGC<<std::endl;
            Eigen::MatrixXd IG_inv = IGC.inverse();
            IG_inv.setIdentity();
            // std::cout<<"IG_inv: "<<std::endl<<IG_inv<<std::endl;
            // std::cout<<"h_dot: "<<std::endl<<AG*robotdata->q_dot_a<<std::endl;
            AG = IG_inv * AG;
            AGdotqdot = IG_inv * AGdotqdot;
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = AG.row((*iter)->task_selection_matrix[i]);
            }
            // std::cout<<"??";
            // update jacobi_dot
            // select the right direction
            (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim, 1);
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi_dot_q_dot(i) = AGdotqdot((*iter)->task_selection_matrix[i]);
            }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            // (*iter)->X_a.row(2) = ((*iter)->jacobi*robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // update F: if the task has no sensor , F will  keep zero
            // (*iter)->X_a.row(3).setZero();

            // std::cout<<"X_a(com): "<<std::endl<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d(com): "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c(com): "<<(*iter)->X_c<<std::endl;
            break;
            //            break;
        }
        case contact_task:
        {
            // reference frame is 1 world
            // update x_a position
            (*iter)->X_a.setZero(4, (*iter)->dim);
            Eigen::Vector3d pa;
            pa = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                              (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Eigen::Matrix3d Ra;
            // Eigen::Vector3d Euler_ZYX;
            Eigen::Vector3d Euler_XYZ;
            Ra = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                             (*iter)->joint_id, false)
                     .transpose();
            // basicfunction::MatrixToEuler_ZYX(Ra,Euler_ZYX);
            basicfunction::MatrixToEuler_XYZ(Ra, Euler_XYZ);
            Eigen::Matrix<double, 6, 1> x_6d;
            // x_6d.block(0,0,3,1) = Euler_ZYX;
            x_6d.block(0, 0, 3, 1) = Euler_XYZ;
            x_6d.block(3, 0, 3, 1) = pa;
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_6d((*iter)->task_selection_matrix[i], 0);
            }
            // update jaccobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = J_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update jacobi_dotq_dot
            //                 RigidBodyDynamics::Math::VectorNd Jdotqdot_6D = RigidBodyDynamics::CalcPointAcceleration6D(*(robotdata->robot_model),robotdata->q_a,
            //                                                            robotdata->q_dot_a,RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof),
            //                                                            (*iter)->joint_id,(*iter)->T_offset.block(0,3,3,1),false);
            //                 Jdotqdot_6D.block(3,0,3,1) = Jdotqdot_6D.block(3,0,3,1) + robotdata->robot_model->gravity;
            // //             (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,robotdata->ndof);
            //                 // select the right direction
            //                 for(uint i = 0; i < (*iter)->task_selection_matrix.size();i++)
            //                 {
            //                     (*iter)->jacobi_dot_q_dot.row(i) = Jdotqdot_6D.row((*iter)->task_selection_matrix[i]);
            //                 }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            // (*iter)->X_a.row(2) = ((*iter)->jacobi*robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        default:
        {
            break;
        }
        }
    }

    // compute GRF
    // std::cout<<"error!!"<<std::endl;
    stateestimator->grfEstimating(robotdata);
    // std::cout<<"error!!"<<std::endl;
    // compute stance phases
    stateestimator->stateMachine(robotdata);
    stateestimator->estWaistPosVelInWorld(robotdata);
    stateestimator->rlStateMachine(robotdata);
    // stateestimator->estWaistPosVelInWorld(robotdata);
    RigidBodyDynamics::UpdateKinematicsCustom(*(robotdata->robot_model), &(robotdata->q_a), &(robotdata->q_dot_a), nullptr);
    // external torque observer
    // externaltorqueobserver(robotdata);

    // update task card not for joint task
    for (std::vector<Task *>::iterator iter = robotdata->task_card_set.begin();
         iter != robotdata->task_card_set.end(); iter++)
    {
        switch ((*iter)->type)
        {
        case general_task:
        {
            // reference frame is 1 world
            // update x_a position
            (*iter)->X_a.setZero(4, (*iter)->dim);
            Eigen::Vector3d pa;
            pa = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                              (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Eigen::Matrix3d Ra;
            Eigen::Vector3d Euler_XYZ;
            Ra = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                             (*iter)->joint_id, false)
                     .transpose();
            // std::cout<<"Ra: "<<std::endl<<Ra<<std::endl;
            // basicfunction::MatrixToEuler_ZYX(Ra,Euler_ZYX);
            // basicfunction::MatrixToEuler_XYZ(Ra,Euler_XYZ);
            basicfunction::matrixtoeulerxyz_(Ra, Euler_XYZ);
            // std::cout<<"Euler_XYZ: "<<std::endl<<Euler_XYZ.transpose()<<std::endl;
            // Eigen::Matrix3d R_test;
            // basicfunction::Euler_XYZToMatrix(R_test,Euler_XYZ);
            // std::cout<<"R_test: "<<std::endl<<R_test<<std::endl;
            Eigen::Matrix<double, 6, 1> x_6d;
            // x_6d.block(0,0,3,1) = Euler_ZYX;
            x_6d.block(0, 0, 3, 1) = Euler_XYZ;
            x_6d.block(3, 0, 3, 1) = pa;
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_6d((*iter)->task_selection_matrix[i], 0);
            }
            // update jaccobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            // std::cout<<"q_a: "<<std::endl<<robotdata->q_a.transpose()<<std::endl;
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            // std::cout<<"J_6D: "<<std::endl<<J_6D<<std::endl;
            // std::cout<<"T_offset: "<<std::endl<<(*iter)->T_offset<<std::endl;
            // std::cout<<"joint_id: "<<std::endl<<(*iter)->joint_id<<std::endl;
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = J_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update jacobi_dotq_dot
            RigidBodyDynamics::Math::VectorNd Jdotqdot_6D = RigidBodyDynamics::CalcPointAcceleration6D(*(robotdata->robot_model), robotdata->q_a,
                                                                                                       robotdata->q_dot_a, RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof),
                                                                                                       (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Jdotqdot_6D.block(3, 0, 3, 1) = Jdotqdot_6D.block(3, 0, 3, 1) + robotdata->robot_model->gravity;
            //            (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi_dot_q_dot.row(i) = Jdotqdot_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update x_a velocity
            //  std::cout<<"J: "<<std::endl<<(*iter)->jacobi<<std::endl;
            //  std::cout<<"q_dot_a: "<<std::endl<<robotdata->q_dot_a.transpose()<<std::endl;
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            (*iter)->X_a.row(2) = ((*iter)->jacobi * robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // std::cout<<"jacobi:"<< std::endl<<(*iter)->jacobi<<std::endl;
            // std::cout<<"q_ddot_a:"<< std::endl<<robotdata->q_ddot_a.transpose()<<std::endl;
            // std::cout<<"jacobi_dot_q_dot:"<< std::endl<<(*iter)->jacobi_dot_q_dot.transpose()<<std::endl;
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        case com_task:
        {
            // reference frame is 1 world
            // std::cout<<"??";
            // compute rotation matrix from floating base to world
            Eigen::Matrix3d Ra_F2I;
            Eigen::Vector3d Euler_XYZ;
            Ra_F2I = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                                 (*iter)->joint_id, false)
                         .transpose();
            basicfunction::MatrixToEuler_XYZ(Ra_F2I, Euler_XYZ);
            // compute jacobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            // compute PS1
            Eigen::MatrixXd PH1;
            PH1.setZero(6, 6);
            // PH1.block(0,0,3,3) = Ra_F2I.transpose();
            // PH1.block(3,3,3,3) = Ra_F2I.transpose();
            PH1 = J_6D.block(0, 0, 6, 6);
            Eigen::MatrixXd PS1 = PH1.inverse();

            // compute H
            RigidBodyDynamics::Math::MatrixNd H;
            H.setZero(robotdata->ndof, robotdata->ndof);
            RigidBodyDynamics::Math::VectorNd _q_a = robotdata->q_a;
            RigidBodyDynamics::CompositeRigidBodyAlgorithm(*(robotdata->robot_model), _q_a, H, false);
            // define U1
            Eigen::MatrixXd U1;
            U1.setZero(6, robotdata->ndof);
            U1.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
            // H11
            Eigen::MatrixXd H11 = U1 * H * U1.transpose();
            // I_1^C
            Eigen::MatrixXd I1C = PS1.transpose() * H11 * PS1;
            // M
            double M = I1C(5, 5);
            // robotdata->robot_M = M;
            // 1^P_G
            Eigen::Vector3d _1PG;
            _1PG(0) = I1C(2, 4) / M;
            _1PG(1) = I1C(0, 5) / M;
            _1PG(2) = I1C(1, 3) / M;
            // I^P_G
            Eigen::Vector3d _IP1;
            Eigen::Vector3d _IPG;
            _IP1 = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                                (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            _IPG = _IP1 + Ra_F2I * _1PG;
            // _1XTG
            Eigen::MatrixXd _1XTG;
            _1XTG.setZero(6, 6);
            _1XTG.block(0, 0, 3, 3) = PH1.block(0, 3, 3, 3);
            _1XTG.block(3, 3, 3, 3) = PH1.block(3, 0, 3, 3);
            _1XTG.block(0, 3, 3, 3) = PH1.block(0, 3, 3, 3) * (basicfunction::skew(_1PG)).transpose();
            // AG
            Eigen::MatrixXd AG = _1XTG * PS1.transpose() * U1 * H;
            // AG_dot*q_dot
            // Cq_dot G
            //                Eigen::VectorXd CG;
            RigidBodyDynamics::Math::VectorNd CG;
            CG.setZero(robotdata->ndof);
            //                Eigen::VectorXd G;
            RigidBodyDynamics::Math::VectorNd G;
            G.setZero(robotdata->ndof);
            //                Eigen::VectorXd C;
            RigidBodyDynamics::Math::VectorNd C;
            C.setZero(robotdata->ndof);
            //                Eigen::MatrixXd QDotZero = robotdata->q_dot_a;
            RigidBodyDynamics::Math::VectorNd QDotZero = robotdata->q_dot_a;
            QDotZero.setZero();
            _q_a = robotdata->q_a;
            RigidBodyDynamics::Math::VectorNd _q_dot_a = robotdata->q_dot_a;
            RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, _q_dot_a, CG);
            RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, QDotZero, G);
            C = CG - G;
            Eigen::MatrixXd AGdotqdot = _1XTG * PS1.transpose() * U1 * C;
            // I_G^3
            // std::cout<<"I1C: "<<std::endl<<I1C<<std::endl;
            Eigen::MatrixXd IGC = _1XTG * I1C * _1XTG.transpose();
            (*iter)->IG = IGC;
            // update x_a
            Eigen::Matrix<double, 1, 6> x_a_row;
            x_a_row.block(0, 0, 1, 3) = Euler_XYZ.transpose();
            x_a_row.block(0, 3, 1, 3) = _IPG.transpose();
            (*iter)->X_a.setZero(4, (*iter)->dim);
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_a_row(0, (*iter)->task_selection_matrix[i]);
            }
            // update jaccobi
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);

            // whole model
            // std::cout<<"IG: "<<std::endl<<IGC<<std::endl;
            Eigen::MatrixXd IG_inv = IGC.inverse();
            // std::cout<<"IG_inv: "<<std::endl<<IG_inv<<std::endl;
            // std::cout<<"h_dot: "<<std::endl<<AG*robotdata->q_dot_a<<std::endl;
            AG = IG_inv * AG;
            AGdotqdot = IG_inv * AGdotqdot;
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = AG.row((*iter)->task_selection_matrix[i]);
            }
            // std::cout<<"??";
            // update jacobi_dot
            // select the right direction
            (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim, 1);
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi_dot_q_dot(i) = AGdotqdot((*iter)->task_selection_matrix[i]);
            }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            (*iter)->X_a.row(2) = ((*iter)->jacobi * robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // update F: if the task has no sensor , F will  keep zero
            (*iter)->X_a.row(3).setZero();

            // std::cout<<"X_a(com): "<<std::endl<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d(com): "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c(com): "<<(*iter)->X_c<<std::endl;
            break;
            //            break;
        }
        case contact_task:
        {
            // reference frame is 1 world
            // update x_a position
            (*iter)->X_a.setZero(4, (*iter)->dim);
            Eigen::Vector3d pa;
            pa = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                              (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Eigen::Matrix3d Ra;
            // Eigen::Vector3d Euler_ZYX;
            Eigen::Vector3d Euler_XYZ;
            Ra = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                             (*iter)->joint_id, false)
                     .transpose();
            // basicfunction::MatrixToEuler_ZYX(Ra,Euler_ZYX);
            basicfunction::MatrixToEuler_XYZ(Ra, Euler_XYZ);
            Eigen::Matrix<double, 6, 1> x_6d;
            // x_6d.block(0,0,3,1) = Euler_ZYX;
            x_6d.block(0, 0, 3, 1) = Euler_XYZ;
            x_6d.block(3, 0, 3, 1) = pa;
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_6d((*iter)->task_selection_matrix[i], 0);
            }
            // update jaccobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = J_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update jacobi_dotq_dot
            RigidBodyDynamics::Math::VectorNd Jdotqdot_6D = RigidBodyDynamics::CalcPointAcceleration6D(*(robotdata->robot_model), robotdata->q_a,
                                                                                                       robotdata->q_dot_a, RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof),
                                                                                                       (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Jdotqdot_6D.block(3, 0, 3, 1) = Jdotqdot_6D.block(3, 0, 3, 1) + robotdata->robot_model->gravity;
            //             (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi_dot_q_dot.row(i) = Jdotqdot_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            (*iter)->X_a.row(2) = ((*iter)->jacobi * robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        default:
        {
            break;
        }
        }
    }
    // update joint task
    for (std::vector<Task *>::iterator iter = robotdata->task_card_set.begin(); iter != robotdata->task_card_set.end(); iter++)
    {
        switch ((*iter)->type)
        {
        case joint_task:
        {
            if (robotdata->robottype == robot_type::Fixed_Base_Open_Chain)
            {
                // update jacobi
                (*iter)->jacobi.setIdentity();
                // update jacobi_dot
                //            (*iter)->jacobi_dot_q_dot.setZero(robotdata->ndof,robotdata->ndof);
                // update x_a position velocity acc
                (*iter)->X_a.row(0) = robotdata->q_a.transpose();
                (*iter)->X_a.row(1) = robotdata->q_dot_a.transpose();
                (*iter)->X_a.row(2) = robotdata->q_ddot_a.transpose();
                // update F
                (*iter)->X_a.row(3) = robotdata->tau_ext.transpose();
            }
            else if ((robotdata->robottype == robot_type::Float_Base_Open_Chain) || (robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain))
            {
                // update jacobi
                int _dim = robotdata->ndof - 6;
                int start_row = 6;
                (*iter)->jacobi.setZero();
                (*iter)->jacobi.block(0, start_row, _dim, _dim).setIdentity();
                // for dynamic herachical
                if (robotdata->wbcsolver == Wbc_Solver_type::hqp_dynamics)
                {
                    (*iter)->jacobi = (*iter)->jacobi * sqrt(1.0 + robotdata->dt * robotdata->dt);
                    if (robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain)
                    {
                        (*iter)->jacobi.block(0, start_row, robotdata->ndof_wheel, robotdata->ndof_wheel) = Eigen::MatrixXd::Identity(robotdata->ndof_wheel, robotdata->ndof_wheel) * robotdata->dt;
                    }
                }
                // end
                // update jacobi_dot
                //            (*iter)->jacobi_dot_q_dot.setZero(robotdata->ndof,robotdata->ndof);
                // update x_a position velocity acc
                (*iter)->X_a.row(0) = robotdata->q_a.block(start_row, 0, _dim, 1).transpose();
                (*iter)->X_a.row(1) = robotdata->q_dot_a.block(start_row, 0, _dim, 1).transpose();
                (*iter)->X_a.row(2) = robotdata->q_ddot_a.block(start_row, 0, _dim, 1).transpose();
                // update F
                (*iter)->X_a.row(3) = robotdata->tau_ext.block(start_row, 0, _dim, 1).transpose();
            }
            // std::cout << "X_a(joint): " << (*iter)->X_a << std::endl;
            // std::cout << "X_d(joint): " << (*iter)->X_d << std::endl;
            // std::cout << "X_c(joint): " << (*iter)->X_c << std::endl;
            break;
        }
        default:
            break;
        }
    }
}

void Estimation_Operator::task_state_update_x_a_walk(Robot_Data *robotdata)
{
    // filter
    // robotdata->q_dot_a = q_dot_a_filter->mFilter(robotdata->q_dot_a);
    // kinematics update : compare the effectiveness lately
    //    RigidBodyDynamics::UpdateKinematics(robotdata->robot_model,robotdata->q_a,robotdata->q_dot_a,robotdata->q_ddot_a);
    // Eigen::Quaternion()
    // update foot jacobi
    for (std::vector<Task *>::iterator iter = robotdata->task_card_set.begin();
         iter != robotdata->task_card_set.end(); iter++)
    {
        switch ((*iter)->type)
        {
        case general_task:
        {
            // reference frame is 1 world
            // update x_a position
            (*iter)->X_a.setZero(4, (*iter)->dim);
            Eigen::Vector3d pa;
            pa = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                              (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Eigen::Matrix3d Ra;
            Eigen::Vector3d Euler_XYZ;
            Ra = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                             (*iter)->joint_id, false)
                     .transpose();
            // std::cout<<"Ra: "<<std::endl<<Ra<<std::endl;
            // basicfunction::MatrixToEuler_ZYX(Ra,Euler_ZYX);
            basicfunction::MatrixToEuler_XYZ(Ra, Euler_XYZ);
            // std::cout<<"Euler_XYZ: "<<std::endl<<Euler_XYZ.transpose()<<std::endl;
            // Eigen::Matrix3d R_test;
            // basicfunction::Euler_XYZToMatrix(R_test,Euler_XYZ);
            // std::cout<<"R_test: "<<std::endl<<R_test<<std::endl;
            Eigen::Matrix<double, 6, 1> x_6d;
            // x_6d.block(0,0,3,1) = Euler_ZYX;
            x_6d.block(0, 0, 3, 1) = Euler_XYZ;
            x_6d.block(3, 0, 3, 1) = pa;
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_6d((*iter)->task_selection_matrix[i], 0);
            }
            // update jaccobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            // std::cout<<"q_a: "<<std::endl<<robotdata->q_a.transpose()<<std::endl;
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            // std::cout<<"J_6D: "<<std::endl<<J_6D<<std::endl;
            // std::cout<<"T_offset: "<<std::endl<<(*iter)->T_offset<<std::endl;
            // std::cout<<"joint_id: "<<std::endl<<(*iter)->joint_id<<std::endl;
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = J_6D.row((*iter)->task_selection_matrix[i]);
            }
            //          // update jacobi_dotq_dot
            //             RigidBodyDynamics::Math::VectorNd Jdotqdot_6D = RigidBodyDynamics::CalcPointAcceleration6D(*(robotdata->robot_model),robotdata->q_a,
            //                                                        robotdata->q_dot_a,RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof),
            //                                                        (*iter)->joint_id,(*iter)->T_offset.block(0,3,3,1),false);
            //             Jdotqdot_6D.block(3,0,3,1) = Jdotqdot_6D.block(3,0,3,1) + robotdata->robot_model->gravity;
            // //            (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,robotdata->ndof);
            //             // select the right direction
            //             for(uint i = 0; i < (*iter)->task_selection_matrix.size();i++)
            //             {
            //                 (*iter)->jacobi_dot_q_dot.row(i) = Jdotqdot_6D.row((*iter)->task_selection_matrix[i]);
            //             }
            // update x_a velocity
            //  std::cout<<"J: "<<std::endl<<(*iter)->jacobi<<std::endl;
            //  std::cout<<"q_dot_a: "<<std::endl<<robotdata->q_dot_a.transpose()<<std::endl;
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            // (*iter)->X_a.row(2) = ((*iter)->jacobi*robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // std::cout<<"jacobi:"<< std::endl<<(*iter)->jacobi<<std::endl;
            // std::cout<<"q_ddot_a:"<< std::endl<<robotdata->q_ddot_a.transpose()<<std::endl;
            // std::cout<<"jacobi_dot_q_dot:"<< std::endl<<(*iter)->jacobi_dot_q_dot.transpose()<<std::endl;
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        case com_task:
        {
            // reference frame is 1 world
            // std::cout<<"??";
            // compute rotation matrix from floating base to world
            Eigen::Matrix3d Ra_F2I;
            Eigen::Vector3d Euler_XYZ;
            Ra_F2I = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                                 (*iter)->joint_id, false)
                         .transpose();
            basicfunction::MatrixToEuler_XYZ(Ra_F2I, Euler_XYZ);
            // compute jacobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            // compute PS1
            Eigen::MatrixXd PH1;
            PH1.setZero(6, 6);
            // PH1.block(0,0,3,3) = Ra_F2I.transpose();
            // PH1.block(3,3,3,3) = Ra_F2I.transpose();
            PH1 = J_6D.block(0, 0, 6, 6);
            Eigen::MatrixXd PS1 = PH1.inverse();

            // compute H
            RigidBodyDynamics::Math::MatrixNd H;
            H.setZero(robotdata->ndof, robotdata->ndof);
            RigidBodyDynamics::Math::VectorNd _q_a = robotdata->q_a;
            RigidBodyDynamics::CompositeRigidBodyAlgorithm(*(robotdata->robot_model), _q_a, H, false);
            // define U1
            Eigen::MatrixXd U1;
            U1.setZero(6, robotdata->ndof);
            U1.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
            // H11
            Eigen::MatrixXd H11 = U1 * H * U1.transpose();
            // I_1^C
            Eigen::MatrixXd I1C = PS1.transpose() * H11 * PS1;
            // M
            double M = I1C(5, 5);
            // robotdata->robot_M = M;
            // 1^P_G
            Eigen::Vector3d _1PG;
            _1PG(0) = I1C(2, 4) / M;
            _1PG(1) = I1C(0, 5) / M;
            _1PG(2) = I1C(1, 3) / M;
            // I^P_G
            Eigen::Vector3d _IP1;
            Eigen::Vector3d _IPG;
            _IP1 = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                                (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            _IPG = _IP1 + Ra_F2I * _1PG;
            // _1XTG
            Eigen::MatrixXd _1XTG;
            _1XTG.setZero(6, 6);
            _1XTG.block(0, 0, 3, 3) = PH1.block(0, 3, 3, 3);
            _1XTG.block(3, 3, 3, 3) = PH1.block(3, 0, 3, 3);
            _1XTG.block(0, 3, 3, 3) = PH1.block(0, 3, 3, 3) * (basicfunction::skew(_1PG)).transpose();
            // AG
            Eigen::MatrixXd AG = _1XTG * PS1.transpose() * U1 * H;
            // AG_dot*q_dot
            // Cq_dot G
            //                Eigen::VectorXd CG;
            RigidBodyDynamics::Math::VectorNd CG;
            CG.setZero(robotdata->ndof);
            //                Eigen::VectorXd G;
            RigidBodyDynamics::Math::VectorNd G;
            G.setZero(robotdata->ndof);
            //                Eigen::VectorXd C;
            RigidBodyDynamics::Math::VectorNd C;
            C.setZero(robotdata->ndof);
            //                Eigen::MatrixXd QDotZero = robotdata->q_dot_a;
            RigidBodyDynamics::Math::VectorNd QDotZero = robotdata->q_dot_a;
            QDotZero.setZero();
            _q_a = robotdata->q_a;
            RigidBodyDynamics::Math::VectorNd _q_dot_a = robotdata->q_dot_a;
            RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, _q_dot_a, CG);
            RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, QDotZero, G);
            C = CG - G;
            Eigen::MatrixXd AGdotqdot = _1XTG * PS1.transpose() * U1 * C;
            // I_G^3
            // std::cout<<"I1C: "<<std::endl<<I1C<<std::endl;
            Eigen::MatrixXd IGC = _1XTG * I1C * _1XTG.transpose();
            (*iter)->IG = IGC;
            // update x_a
            Eigen::Matrix<double, 1, 6> x_a_row;
            x_a_row.block(0, 0, 1, 3) = Euler_XYZ.transpose();
            x_a_row.block(0, 3, 1, 3) = _IPG.transpose();
            (*iter)->X_a.setZero(4, (*iter)->dim);
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_a_row(0, (*iter)->task_selection_matrix[i]);
            }
            // update jaccobi
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);

            // whole model
            // std::cout<<"IG: "<<std::endl<<IGC<<std::endl;
            Eigen::MatrixXd IG_inv = IGC.inverse();
            // std::cout<<"IG_inv: "<<std::endl<<IG_inv<<std::endl;
            // std::cout<<"h_dot: "<<std::endl<<AG*robotdata->q_dot_a<<std::endl;
            AG = IG_inv * AG;
            AGdotqdot = IG_inv * AGdotqdot;
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = AG.row((*iter)->task_selection_matrix[i]);
            }
            // std::cout<<"??";
            // update jacobi_dot
            // // select the right direction
            // (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,1);
            // for(uint i = 0; i < (*iter)->task_selection_matrix.size();i++)
            // {
            //     (*iter)->jacobi_dot_q_dot(i) = AGdotqdot((*iter)->task_selection_matrix[i]);
            // }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            // (*iter)->X_a.row(2) = ((*iter)->jacobi*robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // update F: if the task has no sensor , F will  keep zero
            // (*iter)->X_a.row(3).setZero();

            // std::cout<<"X_a(com): "<<std::endl<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d(com): "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c(com): "<<(*iter)->X_c<<std::endl;
            break;
            //            break;
        }
        case contact_task:
        {
            // reference frame is 1 world
            // update x_a position
            (*iter)->X_a.setZero(4, (*iter)->dim);
            Eigen::Vector3d pa;
            pa = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                              (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Eigen::Matrix3d Ra;
            // Eigen::Vector3d Euler_ZYX;
            Eigen::Vector3d Euler_XYZ;
            Ra = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                             (*iter)->joint_id, false)
                     .transpose();
            // basicfunction::MatrixToEuler_ZYX(Ra,Euler_ZYX);
            basicfunction::MatrixToEuler_XYZ(Ra, Euler_XYZ);
            Eigen::Matrix<double, 6, 1> x_6d;
            // x_6d.block(0,0,3,1) = Euler_ZYX;
            x_6d.block(0, 0, 3, 1) = Euler_XYZ;
            x_6d.block(3, 0, 3, 1) = pa;
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_6d((*iter)->task_selection_matrix[i], 0);
            }
            // update jaccobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = J_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update jacobi_dotq_dot
            //                 RigidBodyDynamics::Math::VectorNd Jdotqdot_6D = RigidBodyDynamics::CalcPointAcceleration6D(*(robotdata->robot_model),robotdata->q_a,
            //                                                            robotdata->q_dot_a,RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof),
            //                                                            (*iter)->joint_id,(*iter)->T_offset.block(0,3,3,1),false);
            //                 Jdotqdot_6D.block(3,0,3,1) = Jdotqdot_6D.block(3,0,3,1) + robotdata->robot_model->gravity;
            // //             (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,robotdata->ndof);
            //                 // select the right direction
            //                 for(uint i = 0; i < (*iter)->task_selection_matrix.size();i++)
            //                 {
            //                     (*iter)->jacobi_dot_q_dot.row(i) = Jdotqdot_6D.row((*iter)->task_selection_matrix[i]);
            //                 }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            // (*iter)->X_a.row(2) = ((*iter)->jacobi*robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        default:
        {
            break;
        }
        }
    }

    // compute GRF
    // std::cout<<"error!!"<<std::endl;
    stateestimator->grfEstimating(robotdata);
    // std::cout<<"error!!"<<std::endl;
    // compute stance phases
    stateestimator->stateMachine(robotdata);
    stateestimator->estWaistPosVelInWorld(robotdata);
    stateestimator->rlStateMachine(robotdata);
    // stateestimator->estWaistPosVelInWorld(robotdata);
    RigidBodyDynamics::UpdateKinematicsCustom(*(robotdata->robot_model), &(robotdata->q_a), &(robotdata->q_dot_a), nullptr);
    // external torque observer
    // externaltorqueobserver(robotdata);

    // update task card not for joint task
    for (std::vector<Task *>::iterator iter = robotdata->task_card_set.begin();
         iter != robotdata->task_card_set.end(); iter++)
    {
        switch ((*iter)->type)
        {
        case general_task:
        {
            // reference frame is 1 world
            // update x_a position
            (*iter)->X_a.setZero(4, (*iter)->dim);
            Eigen::Vector3d pa;
            pa = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                              (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Eigen::Matrix3d Ra;
            Eigen::Vector3d Euler_XYZ;
            Ra = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                             (*iter)->joint_id, false)
                     .transpose();
            // std::cout<<"Ra: "<<std::endl<<Ra<<std::endl;
            // basicfunction::MatrixToEuler_ZYX(Ra,Euler_ZYX);
            // basicfunction::MatrixToEuler_XYZ(Ra,Euler_XYZ);
            basicfunction::matrixtoeulerxyz_(Ra, Euler_XYZ);
            // std::cout<<"Euler_XYZ: "<<std::endl<<Euler_XYZ.transpose()<<std::endl;
            // Eigen::Matrix3d R_test;
            // basicfunction::Euler_XYZToMatrix(R_test,Euler_XYZ);
            // std::cout<<"R_test: "<<std::endl<<R_test<<std::endl;
            Eigen::Matrix<double, 6, 1> x_6d;
            // x_6d.block(0,0,3,1) = Euler_ZYX;
            x_6d.block(0, 0, 3, 1) = Euler_XYZ;
            x_6d.block(3, 0, 3, 1) = pa;
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_6d((*iter)->task_selection_matrix[i], 0);
            }
            // update jaccobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            // std::cout<<"q_a: "<<std::endl<<robotdata->q_a.transpose()<<std::endl;
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            // std::cout<<"J_6D: "<<std::endl<<J_6D<<std::endl;
            // std::cout<<"T_offset: "<<std::endl<<(*iter)->T_offset<<std::endl;
            // std::cout<<"joint_id: "<<std::endl<<(*iter)->joint_id<<std::endl;
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = J_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update jacobi_dotq_dot
            RigidBodyDynamics::Math::VectorNd Jdotqdot_6D = RigidBodyDynamics::CalcPointAcceleration6D(*(robotdata->robot_model), robotdata->q_a,
                                                                                                       robotdata->q_dot_a, RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof),
                                                                                                       (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Jdotqdot_6D.block(3, 0, 3, 1) = Jdotqdot_6D.block(3, 0, 3, 1) + robotdata->robot_model->gravity;
            //            (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi_dot_q_dot.row(i) = Jdotqdot_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update x_a velocity
            //  std::cout<<"J: "<<std::endl<<(*iter)->jacobi<<std::endl;
            //  std::cout<<"q_dot_a: "<<std::endl<<robotdata->q_dot_a.transpose()<<std::endl;
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            (*iter)->X_a.row(2) = ((*iter)->jacobi * robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // std::cout<<"jacobi:"<< std::endl<<(*iter)->jacobi<<std::endl;
            // std::cout<<"q_ddot_a:"<< std::endl<<robotdata->q_ddot_a.transpose()<<std::endl;
            // std::cout<<"jacobi_dot_q_dot:"<< std::endl<<(*iter)->jacobi_dot_q_dot.transpose()<<std::endl;
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        case com_task:
        {
            // reference frame is 1 world
            // std::cout<<"??";
            // compute rotation matrix from floating base to world
            Eigen::Matrix3d Ra_F2I;
            Eigen::Vector3d Euler_XYZ;
            Ra_F2I = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                                 (*iter)->joint_id, false)
                         .transpose();
            basicfunction::MatrixToEuler_XYZ(Ra_F2I, Euler_XYZ);
            // compute jacobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            // compute PS1
            Eigen::MatrixXd PH1;
            PH1.setZero(6, 6);
            // PH1.block(0,0,3,3) = Ra_F2I.transpose();
            // PH1.block(3,3,3,3) = Ra_F2I.transpose();
            PH1 = J_6D.block(0, 0, 6, 6);
            Eigen::MatrixXd PS1 = PH1.inverse();

            // compute H
            RigidBodyDynamics::Math::MatrixNd H;
            H.setZero(robotdata->ndof, robotdata->ndof);
            RigidBodyDynamics::Math::VectorNd _q_a = robotdata->q_a;
            RigidBodyDynamics::CompositeRigidBodyAlgorithm(*(robotdata->robot_model), _q_a, H, false);
            // define U1
            Eigen::MatrixXd U1;
            U1.setZero(6, robotdata->ndof);
            U1.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
            // H11
            Eigen::MatrixXd H11 = U1 * H * U1.transpose();
            // I_1^C
            Eigen::MatrixXd I1C = PS1.transpose() * H11 * PS1;
            // M
            double M = I1C(5, 5);
            // robotdata->robot_M = M;
            // 1^P_G
            Eigen::Vector3d _1PG;
            _1PG(0) = I1C(2, 4) / M;
            _1PG(1) = I1C(0, 5) / M;
            _1PG(2) = I1C(1, 3) / M;
            // I^P_G
            Eigen::Vector3d _IP1;
            Eigen::Vector3d _IPG;
            _IP1 = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                                (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            _IPG = _IP1 + Ra_F2I * _1PG;
            // _1XTG
            Eigen::MatrixXd _1XTG;
            _1XTG.setZero(6, 6);
            _1XTG.block(0, 0, 3, 3) = PH1.block(0, 3, 3, 3);
            _1XTG.block(3, 3, 3, 3) = PH1.block(3, 0, 3, 3);
            _1XTG.block(0, 3, 3, 3) = PH1.block(0, 3, 3, 3) * (basicfunction::skew(_1PG)).transpose();
            // AG
            Eigen::MatrixXd AG = _1XTG * PS1.transpose() * U1 * H;
            // AG_dot*q_dot
            // Cq_dot G
            //                Eigen::VectorXd CG;
            RigidBodyDynamics::Math::VectorNd CG;
            CG.setZero(robotdata->ndof);
            //                Eigen::VectorXd G;
            RigidBodyDynamics::Math::VectorNd G;
            G.setZero(robotdata->ndof);
            //                Eigen::VectorXd C;
            RigidBodyDynamics::Math::VectorNd C;
            C.setZero(robotdata->ndof);
            //                Eigen::MatrixXd QDotZero = robotdata->q_dot_a;
            RigidBodyDynamics::Math::VectorNd QDotZero = robotdata->q_dot_a;
            QDotZero.setZero();
            _q_a = robotdata->q_a;
            RigidBodyDynamics::Math::VectorNd _q_dot_a = robotdata->q_dot_a;
            RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, _q_dot_a, CG);
            RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), _q_a, QDotZero, G);
            C = CG - G;
            Eigen::MatrixXd AGdotqdot = _1XTG * PS1.transpose() * U1 * C;
            // I_G^3
            // std::cout<<"I1C: "<<std::endl<<I1C<<std::endl;
            Eigen::MatrixXd IGC = _1XTG * I1C * _1XTG.transpose();
            (*iter)->IG = IGC;
            // update x_a
            Eigen::Matrix<double, 1, 6> x_a_row;
            x_a_row.block(0, 0, 1, 3) = Euler_XYZ.transpose();
            x_a_row.block(0, 3, 1, 3) = _IPG.transpose();
            (*iter)->X_a.setZero(4, (*iter)->dim);
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_a_row(0, (*iter)->task_selection_matrix[i]);
            }
            // update jaccobi
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);

            // whole model
            // std::cout<<"IG: "<<std::endl<<IGC<<std::endl;
            Eigen::MatrixXd IG_inv = IGC.inverse();
            IG_inv.setIdentity();
            // std::cout<<"IG_inv: "<<std::endl<<IG_inv<<std::endl;
            // std::cout<<"h_dot: "<<std::endl<<AG*robotdata->q_dot_a<<std::endl;
            AG = IG_inv * AG;
            AGdotqdot = IG_inv * AGdotqdot;
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = AG.row((*iter)->task_selection_matrix[i]);
            }
            // std::cout<<"??";
            // update jacobi_dot
            // select the right direction
            (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim, 1);
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi_dot_q_dot(i) = AGdotqdot((*iter)->task_selection_matrix[i]);
            }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            (*iter)->X_a.row(2) = ((*iter)->jacobi * robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // update F: if the task has no sensor , F will  keep zero
            (*iter)->X_a.row(3).setZero();
            robotdata->v_com = ((*iter)->X_a.row(1).tail(3).transpose() + (*iter)->X_a.row(1).head(3).transpose() / (*iter)->X_a(0, 5)) / M;
            // std::cout<<"X_a(com): "<<std::endl<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d(com): "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c(com): "<<(*iter)->X_c<<std::endl;
            break;
            //            break;
        }
        case contact_task:
        {
            // reference frame is 1 world
            // update x_a position
            (*iter)->X_a.setZero(4, (*iter)->dim);
            Eigen::Vector3d pa;
            pa = RigidBodyDynamics::CalcBodyToBaseCoordinates(*(robotdata->robot_model), robotdata->q_a,
                                                              (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Eigen::Matrix3d Ra;
            // Eigen::Vector3d Euler_ZYX;
            Eigen::Vector3d Euler_XYZ;
            Ra = RigidBodyDynamics::CalcBodyWorldOrientation(*(robotdata->robot_model), robotdata->q_a,
                                                             (*iter)->joint_id, false)
                     .transpose();
            // basicfunction::MatrixToEuler_ZYX(Ra,Euler_ZYX);
            basicfunction::MatrixToEuler_XYZ(Ra, Euler_XYZ);
            Eigen::Matrix<double, 6, 1> x_6d;
            // x_6d.block(0,0,3,1) = Euler_ZYX;
            x_6d.block(0, 0, 3, 1) = Euler_XYZ;
            x_6d.block(3, 0, 3, 1) = pa;
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->X_a(0, i) = x_6d((*iter)->task_selection_matrix[i], 0);
            }
            // update jaccobi
            Eigen::MatrixXd J_6D;
            J_6D.setZero(6, robotdata->ndof);
            RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model), robotdata->q_a,
                                                   (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1),
                                                   J_6D, false);
            (*iter)->jacobi.setZero((*iter)->dim, robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi.row(i) = J_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update jacobi_dotq_dot
            RigidBodyDynamics::Math::VectorNd Jdotqdot_6D = RigidBodyDynamics::CalcPointAcceleration6D(*(robotdata->robot_model), robotdata->q_a,
                                                                                                       robotdata->q_dot_a, RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof),
                                                                                                       (*iter)->joint_id, (*iter)->T_offset.block(0, 3, 3, 1), false);
            Jdotqdot_6D.block(3, 0, 3, 1) = Jdotqdot_6D.block(3, 0, 3, 1) + robotdata->robot_model->gravity;
            //             (*iter)->jacobi_dot_q_dot.setZero((*iter)->dim,robotdata->ndof);
            // select the right direction
            for (uint i = 0; i < (*iter)->task_selection_matrix.size(); i++)
            {
                (*iter)->jacobi_dot_q_dot.row(i) = Jdotqdot_6D.row((*iter)->task_selection_matrix[i]);
            }
            // update x_a velocity
            (*iter)->X_a.row(1) = ((*iter)->jacobi * robotdata->q_dot_a).transpose();
            // update x_a acc
            (*iter)->X_a.row(2) = ((*iter)->jacobi * robotdata->q_ddot_a + (*iter)->jacobi_dot_q_dot).transpose();
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        default:
        {
            break;
        }
        }
    }
    // update joint task
    for (std::vector<Task *>::iterator iter = robotdata->task_card_set.begin(); iter != robotdata->task_card_set.end(); iter++)
    {
        switch ((*iter)->type)
        {
        case joint_task:
        {
            if (robotdata->robottype == robot_type::Fixed_Base_Open_Chain)
            {
                // update jacobi
                (*iter)->jacobi.setIdentity();
                // update jacobi_dot
                //            (*iter)->jacobi_dot_q_dot.setZero(robotdata->ndof,robotdata->ndof);
                // update x_a position velocity acc
                (*iter)->X_a.row(0) = robotdata->q_a.transpose();
                (*iter)->X_a.row(1) = robotdata->q_dot_a.transpose();
                (*iter)->X_a.row(2) = robotdata->q_ddot_a.transpose();
                // update F
                (*iter)->X_a.row(3) = robotdata->tau_ext.transpose();
            }
            else if ((robotdata->robottype == robot_type::Float_Base_Open_Chain) || (robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain))
            {
                // update jacobi
                int _dim = robotdata->ndof - 6;
                int start_row = 6;
                (*iter)->jacobi.setZero();
                (*iter)->jacobi.block(0, start_row, _dim, _dim).setIdentity();
                // for dynamic herachical
                if (robotdata->wbcsolver == Wbc_Solver_type::hqp_dynamics)
                {
                    (*iter)->jacobi = (*iter)->jacobi * sqrt(1.0 + robotdata->dt * robotdata->dt);
                    if (robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain)
                    {
                        (*iter)->jacobi.block(0, start_row, robotdata->ndof_wheel, robotdata->ndof_wheel) = Eigen::MatrixXd::Identity(robotdata->ndof_wheel, robotdata->ndof_wheel) * robotdata->dt;
                    }
                }
                // end
                // update jacobi_dot
                //            (*iter)->jacobi_dot_q_dot.setZero(robotdata->ndof,robotdata->ndof);
                // update x_a position velocity acc
                (*iter)->X_a.row(0) = robotdata->q_a.block(start_row, 0, _dim, 1).transpose();
                (*iter)->X_a.row(1) = robotdata->q_dot_a.block(start_row, 0, _dim, 1).transpose();
                (*iter)->X_a.row(2) = robotdata->q_ddot_a.block(start_row, 0, _dim, 1).transpose();
                // update F
                (*iter)->X_a.row(3) = robotdata->tau_ext.block(start_row, 0, _dim, 1).transpose();
            }
            std::cout << "X_a(joint): " << (*iter)->X_a << std::endl;
            std::cout << "X_d(joint): " << (*iter)->X_d << std::endl;
            std::cout << "X_c(joint): " << (*iter)->X_c << std::endl;
            break;
        }
        default:
            break;
        }
    }
}

void Estimation_Operator::task_state_update_x_d(Robot_Data *robotdata)
{
    // update task card not for joint task
    for (std::vector<Task *>::iterator iter = robotdata->task_card_set.begin();
         iter != robotdata->task_card_set.end(); iter++)
    {
        switch ((*iter)->type)
        {
        case general_task:
        {
            // compute controller value
            (*iter)->controller->setinput_data((*iter)->X_a, (*iter)->X_d, (*iter)->dim, robotdata->dt);
            (*iter)->controller->controller_run();
            (*iter)->controller->getoutput_data((*iter)->X_c);
            // std::cout<<"X_a: "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        case com_task:
        {
            // compute controller value
            (*iter)->controller->setinput_data((*iter)->X_a, (*iter)->X_d, (*iter)->dim, robotdata->dt);
            // linear moment control
            // todo change to kinematics
            // angular moment control
            Eigen::MatrixXd para_ = (*iter)->controller->para;
            // if((*iter)->task_selection_matrix[0] == task_direction::task_x_theta){
            //     (*iter)->X_c.row(2).block(0,0,1,3) = (*iter)->X_d.row(2).block(0,0,1,3) + (*iter)->controller->para(1)*((*iter)->X_d.row(1).block(0,0,1,3) - (*iter)->X_a.row(1).block(0,0,1,3));
            // }
            // x_
            // (*iter)->X_c.row(0) = 100.0*((*iter)->X_d.row(0) - (*iter)->X_a.row(0));
            (*iter)->X_c.block(0, 0, 1, 3).setZero();
            // (*iter)->X_c.row(0)(3) =para_(3)*((*iter)->X_d.row(0)(3) - (*iter)->X_a.row(0)(3));
            // (*iter)->X_c.row(0)(4) =para_(4)*((*iter)->X_d.row(0)(4) - (*iter)->X_a.row(0)(4));
            // (*iter)->X_c.row(0)(5) =para_(5)*((*iter)->X_d.row(0)(5) - (*iter)->X_a.row(0)(5));
            (*iter)->X_c.row(0)(3) = ((*iter)->X_d.row(0)(3) - (*iter)->X_a.row(0)(3));
            (*iter)->X_c.row(0)(4) = ((*iter)->X_d.row(0)(4) - (*iter)->X_a.row(0)(4));
            (*iter)->X_c.row(0)(5) = ((*iter)->X_d.row(0)(5) - (*iter)->X_a.row(0)(5));
            // x_dot
            (*iter)->X_c.row(1) = (*iter)->X_d.row(1);
            (*iter)->X_c.row(1)(3) = (*iter)->X_d.row(1)(3); // + para_(3)*((*iter)->X_d.row(0)(3) - (*iter)->X_a.row(0)(3)) ;//+ para_(9)*((*iter)->X_d.row(1)(3) - (*iter)->X_a.row(1)(3));
            (*iter)->X_c.row(1)(4) = (*iter)->X_d.row(1)(4); // + para_(4)*((*iter)->X_d.row(0)(4) - (*iter)->X_a.row(0)(4)) ;//+ para_(10)*((*iter)->X_d.row(1)(4) - (*iter)->X_a.row(1)(4));
            (*iter)->X_c.row(1)(5) = (*iter)->X_d.row(1)(5); // + para_(5)*((*iter)->X_d.row(0)(5) - (*iter)->X_a.row(0)(5)) ;//+ para_(11)*((*iter)->X_d.row(1)(5) - (*iter)->X_a.row(1)(5));
            // x_ddot
            (*iter)->X_c.row(2)(0) = (*iter)->X_d.row(2)(0) + para_(6) * ((*iter)->X_d.row(1)(0) - (*iter)->X_a.row(1)(0));
            (*iter)->X_c.row(2)(1) = (*iter)->X_d.row(2)(1) + para_(7) * ((*iter)->X_d.row(1)(1) - (*iter)->X_a.row(1)(1));
            (*iter)->X_c.row(2)(2) = (*iter)->X_d.row(2)(2) + para_(8) * ((*iter)->X_d.row(1)(2) - (*iter)->X_a.row(1)(2));
            // linear moment control
            // (*iter)->X_c.row(2).block(0,3,1,3) = (*iter)->X_d.row(2).block(0,3,1,3) + (*iter)->controller->para(2)*((*iter)->X_d.row(1).block(0,3,1,3) - (*iter)->X_a.row(1).block(0,3,1,3))
            // + M * (*iter)->controller->para(0)*((*iter)->X_d.row(0).block(0,3,1,3) - (*iter)->X_a.row(0).block(0,3,1,3));
            (*iter)->X_c.row(2)(3) = (*iter)->X_d.row(2)(3) + para_(9) * ((*iter)->X_d.row(1)(3) - (*iter)->X_a.row(1)(3)) + para_(3) * ((*iter)->X_d.row(0)(3) - (*iter)->X_a.row(0)(3));
            (*iter)->X_c.row(2)(4) = (*iter)->X_d.row(2)(4) + para_(10) * ((*iter)->X_d.row(1)(4) - (*iter)->X_a.row(1)(4)) + para_(4) * ((*iter)->X_d.row(0)(4) - (*iter)->X_a.row(0)(4));
            (*iter)->X_c.row(2)(5) = (*iter)->X_d.row(2)(5) + para_(11) * ((*iter)->X_d.row(1)(5) - (*iter)->X_a.row(1)(5)) + para_(5) * ((*iter)->X_d.row(0)(5) - (*iter)->X_a.row(0)(5));

            // compute desired contact wrench
            Eigen::MatrixXd bias = Eigen::MatrixXd::Zero(6, 1);
            bias.block(3, 0, 3, 1) = robotdata->robot_model->gravity;
            robotdata->net_contact_force = (*iter)->IG * ((*iter)->X_c.row(2).transpose() - bias);
            robotdata->G = (*iter)->IG * bias;
            // std::cout<<"X_a(com): "<<std::endl<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d(com): "<<std::endl<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c(com): "<<std::endl<<(*iter)->X_c<<std::endl;
            break;
            //            break;
        }
        case contact_task:
        {
            // compute controller value
            (*iter)->controller->setinput_data((*iter)->X_a, (*iter)->X_d, (*iter)->dim, robotdata->dt);
            (*iter)->controller->controller_run();
            (*iter)->controller->getoutput_data((*iter)->X_c);
            // if((*iter)->contact_state_d == true){
            //     (*iter)->X_c.row(2).setZero();
            // }
            // std::cout<<"X_a: "<<std::endl<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d: "<<std::endl<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c: "<<(*iter)->X_c<<std::endl;
            break;
        }
        default:
        {
            break;
        }
        }
    }
    // update joint task
    for (std::vector<Task *>::iterator iter = robotdata->task_card_set.begin(); iter != robotdata->task_card_set.end(); iter++)
    {
        switch ((*iter)->type)
        {
        case joint_task:
        {
            (*iter)->controller->setinput_data((*iter)->X_a, (*iter)->X_d, (*iter)->dim, robotdata->dt);
            (*iter)->controller->controller_run();
            (*iter)->controller->getoutput_data((*iter)->X_c);
            // for dynamic herachical
            if (robotdata->wbcsolver == Wbc_Solver_type::hqp_dynamics)
            {
                if (robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain)
                {
                    (*iter)->X_c.block(2, 0, 1, robotdata->ndof_wheel).setZero();
                }
                (*iter)->X_c.row(2) = (*iter)->X_c.row(2) + (*iter)->X_a.row(1) * robotdata->dt;
            }
            // end
            // std::cout<<"X_a(joint): "<<(*iter)->X_a<<std::endl;
            // std::cout<<"X_d(joint): "<<(*iter)->X_d<<std::endl;
            // std::cout<<"X_c(joint): "<<(*iter)->X_c<<std::endl;
            break;
        }
        default:
            break;
        }
    }
}
