#include "StateEstimate.h"
#include "Robot_Data.h"

StateEstimate::StateEstimate()
{
    lowpass = new LowPassFilter(50.0, 0.707, 0.0025, 3);
    accKalman = new AccKalmanFilter(0.001, 0.01, 0.0025, 3);
    Eigen::VectorXd d_SysNoise = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd d_MeaNoise = Eigen::VectorXd::Zero(8);
    d_SysNoise << 0.001 * 9.8 / 55.0, 0.001 * 9.8 / 55.0, 0.001 * 9.8 / 55.0, 0.001 / 55.0;
    d_MeaNoise << 0.01, 0.01, 0.01, 0.001, 0.01, 0.01, 0.01, 0.001;
    leggedKalman = new LeggedKalmanFilter(d_SysNoise, d_MeaNoise, 0.0025, 4, 8, 3);
    left_foot_velocity_last = 0.0;
    right_foot_velocity_last = 0.0;
}

bool StateEstimate::estWaistPosVelInWorld(Robot_Data *robotdata, int FootType)
{

    Vector3d posTorso = Vector3d::Zero();
    Vector3d velTorso = Vector3d::Zero();
    Vector3d velTorso_filt = Vector3d::Zero();
    Vector3d velTorso_filt2 = Vector3d::Zero();
    switch (FootType)
    {
    case 0: // Left Sole Stance
        posTorso = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose() - robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 3).transpose();
        velTorso = (robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose() - robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(1, 3, 1, 3).transpose());
        break;
    case 1: // Right Sole Stance
        posTorso = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose() - robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 3).transpose();
        velTorso = (robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose() - robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(1, 3, 1, 3).transpose());
        break;
    default:
        break;
    }

    // std::cout << "RBDL result: " << velTorso.transpose() << std::endl;
    int filterType = 1;
    if (filterType == 0)
    {
        velTorso_filt = lowpass->mFilter(velTorso);
    }
    else
    {
        Eigen::Vector3d aWorld = Vector3d::Zero();
        double trust;
        trust = std::min(robotdata->t / robotdata->Tc, 1.0);
        trust = std::min(trust, std::min(1.0 + (robotdata->T - robotdata->t) / robotdata->Tc, 1.0));
        trust = std::max(trust, 0.0);
        trust = 1.0;
        aWorld = rotX(robotdata->q_a(3)) * rotY(robotdata->q_a(4)) * rotZ(robotdata->q_a(5)) * robotdata->imuAcc;
        aWorld(2) = aWorld(2) - 9.81;
        velTorso_filt = accKalman->mFilter(velTorso, aWorld, trust);
        // std::cout << "velTorso filtered: " << velTorso_filt.transpose() << std::endl;
        robotdata->temp_worldacc = aWorld;
    }
    // Eigen::Vector3d aWorld = Vector3d::Zero();
    // double trust;
    // trust = 1.0;
    // aWorld = rotX(robotdata->q_a(3))*rotY(robotdata->q_a(4))*rotZ(robotdata->q_a(5))*robotdata->imuAcc;
    // aWorld(2) = aWorld(2) - 9.81;
    // // velTorso_filt = accKalman->mFilter(velTorso, aWorld, trust);
    // robotdata->temp_kalman.head(3) = velTorso;
    velTorso_filt2 = lowpass->mFilter(velTorso);
    // robotdata->temp_kalman.tail(3) = velTorso_filt2;
    // robotdata->temp.tail(3) = accKalman->mFilter(velTorso, aWorld, trust);

    robotdata->q_a.head(3) = posTorso;
    robotdata->q_dot_a.head(3) = velTorso_filt;

    // robotdata->temp = velTorso;
    robotdata->odometer = posTorso + robotdata->foot_odometer;
    return true;
}

bool StateEstimate::estWaistPosVelInWorld(Robot_Data *robotdata)
{

    Vector3d posTorso_l = Vector3d::Zero();
    Vector3d velTorso_l = Vector3d::Zero();
    Vector3d posTorso_r = Vector3d::Zero();
    Vector3d velTorso_r = Vector3d::Zero();
    Vector3d velTorso_filt = Vector3d::Zero();

    posTorso_l = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose() - robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 3).transpose();
    velTorso_l = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose() - robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(1, 3, 1, 3).transpose();

    posTorso_r = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose() - robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 3).transpose();
    velTorso_r = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose() - robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(1, 3, 1, 3).transpose();

    Eigen::Vector3d aWorld = Vector3d::Zero();
    Eigen::Vector2d trust;

    // if(robotdata->t < robotdata->Tc){
    //     if(robotdata->stance_index==0){
    //         trust(0) = robotdata->t/robotdata->Tc;
    //         trust(1) = 1.0 - trust(0);
    //     }
    //     else{
    //         trust(1) = robotdata->t/robotdata->Tc;
    //         trust(0) = 1.0 - trust(1);
    //     }
    // }
    // else{
    //     if(robotdata->stance_index==0){
    //         trust(0) = 1.0;
    //         trust(1) = 0.0;
    //     }
    //     else{
    //         trust(1) = 1.0;
    //         trust(0) = 0.0;
    //     }
    // }

    for (int i = 0; i < 2; ++i)
    {
        if (robotdata->t_rl(i) < robotdata->Tc)
        {
            if (robotdata->st_rl(i) == 1)
            {
                trust(i) = robotdata->t_rl(i) / robotdata->Tc;
            }
            else
            {
                trust(i) = 1.0 - robotdata->t_rl(i) / robotdata->Tc;
            }
        }
        else
        {
            if (robotdata->st_rl(i) == 1)
            {
                trust(i) = 1.0;
            }
            else
            {
                trust(i) = 0.0;
            }
        }
    }

    aWorld = rotX(robotdata->q_a(3)) * rotY(robotdata->q_a(4)) * rotZ(robotdata->q_a(5)) * robotdata->imuAcc;

    // std::cout << "--------------------------------------------" << std::endl;
    // std::cout << "Acc before rotation: " << robotdata->imuAcc.transpose() << std::endl;
    // std::cout << "Acc after rotation: " << aWorld.transpose() << std::endl;

    // std::cout << "Check: " << cos(robotdata->q_a(4)) << std::endl;
    // std::cout << "Check: " << sin(robotdata->q_a(4)) << std::endl;
    // std::cout << "Acc(X) by LCB: " << robotdata->imuAcc(0) * cos(robotdata->q_a(4)) + robotdata->imuAcc(2) * sin(robotdata->q_a(4)) << std::endl;

    aWorld(2) = aWorld(2) - 9.81;
    // aWorld(2) = 0.0;
    Eigen::VectorXd meas_Output = Eigen::VectorXd::Zero(8);
    meas_Output << velTorso_l, posTorso_l(2), velTorso_r, posTorso_r(2);
    Eigen::VectorXd est_State = Eigen::VectorXd::Zero(4);
    est_State = leggedKalman->mFilter(meas_Output, aWorld, trust);
    velTorso_filt = est_State.head(3);

    // robotdata->temp_worldacc = aWorld;
    robotdata->temp_kalman.head(4) = est_State;
    // robotdata->temp_kalman.tail(3) = velTorso_r;

    // if(robotdata->stance_index==0){
    //     robotdata->q_a.head(3) = posTorso_l;
    // }
    // else{
    //     robotdata->q_a.head(3) = posTorso_r;
    // }
    robotdata->q_a(2) = est_State(3);
    robotdata->q_dot_a.head(3) = velTorso_filt;
    // std::cout << "velTorso_filt: " << velTorso_filt.transpose() << std::endl;
    // // robotdata->temp = velTorso;
    // robotdata->odometer = robotdata->q_a.head(3) + robotdata->foot_odometer;
    return true;
}

bool StateEstimate::grfEstimating(Robot_Data *robotdata)
{

    robotdata->grf_pre = robotdata->grf;

    Eigen::MatrixXd J_grf = Eigen::MatrixXd::Zero(12, 12);
    Eigen::VectorXd nonlinear = Eigen::VectorXd::Zero(18);
    J_grf.block(0, 0, 6, 12) = robotdata->task_card_set[robotdata->left_foot_id]->jacobi.rightCols(12);
    J_grf.block(6, 0, 6, 12) = robotdata->task_card_set[robotdata->right_foot_id]->jacobi.rightCols(12);
    RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), robotdata->q_a, robotdata->q_dot_a, nonlinear);
    robotdata->grf = J_grf.transpose().colPivHouseholderQr().solve(nonlinear.tail(12) - robotdata->tau_a.tail(12));
    // std::cout<<"grf"<<robotdata->grf.transpose()<<std::endl;
    return true;
}

bool StateEstimate::stateMachine(Robot_Data *robotdata)
{

    // grfEstimating(robotdata);

    // ---------------------------------------- GAIT PLAN STEPPING STATE MACHINE ----------------------------------------

    robotdata->t = robotdata->time - robotdata->t_switch;
    robotdata->touch_index_pre = robotdata->touch_index;

    if ((robotdata->touch_index_pre == 4 && (robotdata->grf(11) - robotdata->grf_pre(11) >= robotdata->grf_lb || robotdata->grf(11) > 0.6 * robotdata->MG) && robotdata->t > 0.7 * robotdata->T + robotdata->Tc) || (robotdata->touch_index_pre > 2 && robotdata->t > robotdata->T + robotdata->Tc - 0.5 * robotdata->dt))
    {
        robotdata->T = robotdata->Td;
        robotdata->t_switch = robotdata->time;
        robotdata->stance_index = 1;
        robotdata->t = robotdata->time - robotdata->t_switch;
        robotdata->touch_index = 1;
        robotdata->step = robotdata->step + 1;
        robotdata->t_ftd = robotdata->T;
        robotdata->foot_odometer = robotdata->foot_odometer + robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 3).transpose();
        robotdata->flag_ankle = 0;
    }
    if (robotdata->touch_index_pre == 1 && robotdata->grf(11) >= robotdata->grf_ub)
    {
        robotdata->touch_index = 2;
        robotdata->t_ftd = robotdata->t;
    }
    if ((robotdata->touch_index_pre == 2 && (robotdata->grf(5) - robotdata->grf_pre(5) >= robotdata->grf_lb || robotdata->grf(5) > 0.6 * robotdata->MG) && robotdata->t > 0.7 * robotdata->T + robotdata->Tc) || (robotdata->touch_index_pre < 3 && robotdata->t > robotdata->T + robotdata->Tc - 0.5 * robotdata->dt))
    {
        robotdata->T = robotdata->Td;
        robotdata->t_switch = robotdata->time;
        robotdata->stance_index = 0;
        robotdata->t = robotdata->time - robotdata->t_switch;
        robotdata->touch_index = 3;
        robotdata->step = robotdata->step + 1;
        robotdata->t_ftd = robotdata->T;
        robotdata->foot_odometer = robotdata->foot_odometer + robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 3).transpose();
        robotdata->flag_ankle = 0;
    }
    if (robotdata->touch_index_pre == 3 && robotdata->grf(5) >= robotdata->grf_ub)
    {
        robotdata->touch_index = 4;
        robotdata->t_ftd = robotdata->t;
    }

    if ((robotdata->touch_index == 4 && fabs(robotdata->grf(7) - robotdata->grf_pre(7)) >= 2.0) || (robotdata->touch_index == 2 && fabs(robotdata->grf(1) - robotdata->grf_pre(1)) >= 2.0))
    {
        if (robotdata->t > 0.7 * robotdata->T + robotdata->Tc)
        {
            robotdata->flag_ankle = 1;
        }
    }

    // if ((robotdata->touch_index_pre == 4 && robotdata->grf(11) - robotdata->grf_pre(11) >= robotdata->grf_lb && robotdata->t > 0.5*robotdata->T )){
    //     robotdata->t_switch = robotdata->time;
    //     robotdata->stance_index = 1;
    //     robotdata->t = robotdata->time - robotdata->t_switch;
    //     robotdata->touch_index = 1;
    //     robotdata->step = robotdata->step + 1;
    // }
    // if (robotdata->touch_index_pre == 1 && robotdata->grf(11) >= robotdata->grf_ub){
    //     robotdata->touch_index = 2;
    // }
    // if ((robotdata->touch_index_pre == 2 && robotdata->grf(5) - robotdata->grf_pre(5) >= robotdata->grf_lb && robotdata->t > 0.5*robotdata->T )){
    //     robotdata->t_switch = robotdata->time;
    //     robotdata->stance_index = 0;
    //     robotdata->t = robotdata->time - robotdata->t_switch;
    //     robotdata->touch_index = 3;
    //     robotdata->step = robotdata->step + 1;
    // }
    // if (robotdata->touch_index_pre == 3 && robotdata->grf(5) >= robotdata->grf_ub){
    //     robotdata->touch_index = 4;
    // }

    robotdata->s = (robotdata->t - robotdata->Tc) / robotdata->T;
    robotdata->s = std::max(std::min(robotdata->s, 1.0), 0.0);

    // ---------------------------------------- STATE ESTIMATOR STEPPING STATE MACHINE ----------------------------------------
    if (robotdata->touch_type == 4 && robotdata->grf(5) > 200.0)
    {
        robotdata->touch_type = 1;
    }
    else if (robotdata->touch_type == 1 && abs(robotdata->task_card_set[robotdata->left_foot_id]->X_a(1, 3) - left_foot_velocity_last) < 0.01)
    {
        robotdata->touch_type = 2;
        robotdata->foot_type = 0;
    }
    else if (robotdata->touch_type == 2 && robotdata->grf(11) > 200.0)
    {
        robotdata->touch_type = 3;
    }
    else if (robotdata->touch_type == 3 && abs(robotdata->task_card_set[robotdata->right_foot_id]->X_a(1, 3) - right_foot_velocity_last) < 0.01)
    {
        robotdata->touch_type = 4;
        robotdata->foot_type = 1;
    }

    left_foot_velocity_last = robotdata->task_card_set[robotdata->left_foot_id]->X_a(1, 3);
    right_foot_velocity_last = robotdata->task_card_set[robotdata->right_foot_id]->X_a(1, 3);

    // std::cout << "grf" << robotdata->grf(5) << ", " << robotdata->grf(11) << std::endl;
    // std::cout << "vel foot" << left_foot_velocity_last << std::endl;
    // std::cout << "Foottype: " << robotdata->foot_type << std::endl;

    return true;
}

bool StateEstimate::rlStateMachine(Robot_Data *robotdata)
{

    robotdata->st_rl_pre = robotdata->st_rl;

    for (int i = 0; i < 2; ++i)
    {
        robotdata->t_rl(i) = robotdata->time - robotdata->tsw_rl(i);
        if (robotdata->st_rl_pre(i) == 1 && robotdata->grf(5 + i * 6) < 30. && robotdata->t_rl(i) > robotdata->Tc)
        {
            robotdata->st_rl(i) = 0;
            robotdata->tsw_rl(i) = robotdata->time;
            robotdata->t_rl(i) = robotdata->time - robotdata->tsw_rl(i);
        }
        if (robotdata->st_rl_pre(i) == 0 && robotdata->grf(5 + i * 6) > 110. && robotdata->t_rl(i) > robotdata->Tc)
        {
            robotdata->st_rl(i) = 1;
            robotdata->tsw_rl(i) = robotdata->time;
            robotdata->t_rl(i) = robotdata->time - robotdata->tsw_rl(i);
        }
    }

    return true;
}