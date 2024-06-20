# include "gaitPlan.h"

gaitPlan::gaitPlan(){

}
void gaitPlan::init(Eigen::VectorXd qCmd_, Eigen::VectorXd qDotCmd_, Eigen::VectorXd xStand_){
    qCmd = qCmd_;
    qDotCmd = qDotCmd_;
    firstFlag = 1;
    vZInit = 0.0;
    vZEnd = -0.0;
    zMid = 0.06;
    xStand.resize(6);
    xStand = xStand_;
    offSetL << 0.0, 0.105, 0.0;
    offSetR << 0.0, -0.105, 0.0;
    jointP << 3000., 3000., 467., 312., 582., 582.;
    jointD << 30., 30., 15., 15., 11.64, 11.64;

    hd = 0.82;
    pOffset.resize(3);                                                
    pOffset[0] = -0.000; pOffset[1] = -0.000; pOffset[2] = -0.033;// stand has these paras too.  
    vTorso_td_filt << 0.0, 0.0, 0.0; 
}
bool gaitPlan::predTouchState(Robot_Data *robotdata){
    double e1, e2, c1, c2;

    lambda = std::sqrt(9.81/(hd+pOffset[2]));
    double eta_com = 0.05;
    robotdata->v_com_filt = (1.0 - eta_com)*robotdata->v_com_filt + eta_com*robotdata->v_com;

    e1 = std::exp(lambda*(robotdata->T + robotdata->Tc - robotdata->t));
    e2 = std::exp(-lambda*(robotdata->T + robotdata->Tc - robotdata->t));

    // c1 = 0.5*(robotdata->task_card_set[robotdata->com_task_id]->X_a(0,3) + pOffset[0] + 1.0/lambda*robotdata->q_dot_a(0));
    // c2 = 0.5*(robotdata->task_card_set[robotdata->com_task_id]->X_a(0,3) + pOffset[0] - 1.0/lambda*robotdata->q_dot_a(0));

    // pTorso_td(0) = c1*e1 + c2*e2 - pOffset[0] - robotdata->task_card_set[robotdata->com_task_id]->X_a(0,3) + robotdata->q_a(0);
    // vTorso_td(0) = lambda*(c1*e1 - c2*e2);

    // c1 = 0.5*(robotdata->q_a(1) + pOffset[1] + 1.0/lambda*robotdata->q_dot_a(1));
    // c2 = 0.5*(robotdata->q_a(1) + pOffset[1] - 1.0/lambda*robotdata->q_dot_a(1));

    // pTorso_td(1) = c1*e1 + c2*e2;
    // vTorso_td(1) = lambda*(c1*e1 - c2*e2);

    //-------- use constant COM offset----------//
    c1 = 0.5*(robotdata->q_a(0) + pOffset[0] + 1.0/lambda*robotdata->q_dot_a(0));
    c2 = 0.5*(robotdata->q_a(0) + pOffset[0] - 1.0/lambda*robotdata->q_dot_a(0));
    // c1 = 0.5*(robotdata->q_a(0) + pOffset[0] + 1.0/lambda*robotdata->v_com(0));
    // c2 = 0.5*(robotdata->q_a(0) + pOffset[0] - 1.0/lambda*robotdata->v_com(0));

    pTorso_td(0) = c1*e1 + c2*e2 - pOffset[0];
    vTorso_td(0) = lambda*(c1*e1 - c2*e2);

    c1 = 0.5*(robotdata->q_a(1) + pOffset[1] + 1.0/lambda*robotdata->q_dot_a(1));
    c2 = 0.5*(robotdata->q_a(1) + pOffset[1] - 1.0/lambda*robotdata->q_dot_a(1));
    // c1 = 0.5*(robotdata->q_a(1) + pOffset[1] + 1.0/lambda*robotdata->v_com(1));
    // c2 = 0.5*(robotdata->q_a(1) + pOffset[1] - 1.0/lambda*robotdata->v_com(1));

    pTorso_td(1) = c1*e1 + c2*e2;
    vTorso_td(1) = lambda*(c1*e1 - c2*e2);
    

    // filter
    // double ita = 0.024;
    // if(robotdata->t < 0.3*robotdata->T + robotdata->Tc){
    //     ita = 0.015;
    // }
    // else{
    //     ita = 0.08;
    // }
    // vTorso_td_filt(0) = ita*vTorso_td(0) + (1.0-ita)*vTorso_td_filt(0);
    // vTorso_td_filt(1) = ita*vTorso_td(1) + (1.0-ita)*vTorso_td_filt(1);
    // if(robotdata->t < 0.5*robotdata->dt){
    //     vTorso_td_filt(1) = vTorso_td(1);
    // }
    double ita = 0.024;
    if(robotdata->t < 0.3*robotdata->T + robotdata->Tc){
        ita = 0.08;
    }
    else{
        ita = 0.08;
    }
    vTorso_td_filt(0) = ita*vTorso_td(0) + (1.0-ita)*vTorso_td_filt(0);
    vTorso_td_filt(1) = ita*vTorso_td(1) + (1.0-ita)*vTorso_td_filt(1);

    robotdata->temp2.head(3) = vTorso_td_filt;
    robotdata->temp2.tail(3) = vTorso_td;
    vTorso_td = vTorso_td_filt;
    // vx_com mean
    int N = robotdata->T/robotdata->dt;
    double ite_vx = 1.0/N;
    robotdata->avg_vx = ite_vx*robotdata->v_com(0) + (1.0-ite_vx)*robotdata->avg_vx;

    robotdata->odometer_avg = ite_vx*robotdata->odometer + (1.0-ite_vx)*robotdata->odometer_avg;

    return true;
}

bool gaitPlan::walkPlan(Robot_Data *robotdata, Eigen::VectorXd &qCmd_, Eigen::VectorXd &qDotCmd_){
    
    // double vx = 0.0;
    // vx = robotdata->time*0.1;
    // vx = std::min(vx, 0.0);
    if(robotdata->time < 0.5*robotdata->dt){
        robotdata->odometer_d = robotdata->odometer;
    }
    // joystick
    if((fabs(robotdata->vCmd_joystick(0))>0.1)){
        if((fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0))>0.0009))
        {
            robotdata->vCmd(0) +=  0.0009*(robotdata->vCmd_joystick(0) - robotdata->vCmd(0))/fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0));
        }else{
            robotdata->vCmd(0) = robotdata->vCmd_joystick(0);
        }
    }else{
        if((fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0))>0.0015))
        {
            robotdata->vCmd(0) +=  0.0015*(robotdata->vCmd_joystick(0) - robotdata->vCmd(0))/fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0));
        }else{
            robotdata->vCmd(0) = robotdata->vCmd_joystick(0);
        }    
    }


    // robotdata->vCmd << vx, 0.18, -0.18;
    // vy_cmd adjust

    // if(fabs(robotdata->vCmd_joystick(1) - robotdata->vy)>0.00025)
    // {
    //     robotdata->vy +=  0.00025*(robotdata->vCmd_joystick(1) - robotdata->vy)/fabs(robotdata->vCmd_joystick(1) - robotdata->vy);
    // }else{
    //     robotdata->vy = robotdata->vCmd_joystick(1);
    // }
    robotdata->odometer_d(1) += robotdata->vCmd_offset_joystick(1);
    robotdata->vy = 0.*(robotdata->odometer_d(1) - robotdata->odometer_avg(1));  
    // 
    double vy_offset = 0.26;  
    if(robotdata->avg_vx>0.1)
    {
        vy_offset = std::max(0.1,0.26 - 0.08*(fabs(robotdata->avg_vx)-0.1));
    }
    if(robotdata->avg_vx<-0.1){
        vy_offset = std::max(0.1,0.26 + 0.08*(fabs(robotdata->avg_vx)-0.1));
    }

    // double vy_offset = 0.3;  
    // if(robotdata->avg_vx>0.1)
    // {
    //     vy_offset = std::max(0.1,0.3 - 0.15*(fabs(robotdata->avg_vx)-0.1));
    // }
    // if(robotdata->avg_vx<-0.1){
    //     vy_offset = std::max(0.1,0.3 + 0.1*(fabs(robotdata->avg_vx)-0.1));
    // }
    
    robotdata->vCmd(1) = vy_offset + robotdata->vy;
    robotdata->vCmd(2) = -vy_offset + robotdata->vy;


    gaitPlan::predTouchState(robotdata); 

    double d2, sigma1, sigma2, kp_star;
    d2 = robotdata->vCmd(1) + robotdata->vCmd(2);

    sigma1 = 1.8*lambda/std::tanh(0.5*robotdata->Td*lambda);
    // sigma2 = 1.02*lambda*std::tanh(0.5*robotdata->Td*lambda);
    sigma2 = 1.05*lambda*std::tanh(0.5*robotdata->Td*lambda);

    kp_star = 1.0/lambda/std::sinh(robotdata->Td*lambda);

    pFootStrike(0) =  vTorso_td(0)/sigma1 + 0.8*kp_star*(vTorso_td(0)-robotdata->vCmd(0)) + pOffset[0];

    if (robotdata->stance_index==1){
        pFootStrike(1) =  (vTorso_td(1) - d2)/sigma2 - 1.0*kp_star*(vTorso_td(1) - robotdata->vCmd(1)) + pOffset[1];
        // if (pFootStrike(1) + robotdata->q_a(1)<0.1)
        //     pFootStrike(1) = 0.1 - robotdata->q_a(1);
        if (pFootStrike(1) < 0.065)
            pFootStrike(1) = 0.065;
        if (pFootStrike(1) > 0.25)
            pFootStrike(1) = 0.25;
    }
    else{
        pFootStrike(1) =  (vTorso_td(1) - d2)/sigma2 - 1.0*kp_star*(vTorso_td(1) - robotdata->vCmd(2)) + pOffset[1];
        // if (pFootStrike(1) + robotdata->q_a(1)>-0.1)
        //     pFootStrike(1) = -0.1 - robotdata->q_a(1);
        if (pFootStrike(1) > -0.065)
            pFootStrike(1) = -0.065;
        if (pFootStrike(1) < -0.25)
            pFootStrike(1) = -0.25;
    }
    if(pFootStrike(0) - 0.0538 > 0.24){
        pFootStrike(0) = 0.24 + 0.0538;
    }
    else if(pFootStrike(0) - 0.0538 < -0.24){
        pFootStrike(0) = -0.24 + 0.0538;
    }

    pFootStrike(2) = 0.0;

    //foot strike location method 2
    robotdata->pFootb_tgt2(0) = robotdata->q_dot_a(0)/sigma1 + 0.8*kp_star*(robotdata->q_dot_a(0)-robotdata->vCmd(0)) + pOffset[0];
    if (robotdata->stance_index==1){
        robotdata->pFootb_tgt2(1) =  (robotdata->q_dot_a(1) - d2)/sigma2 - 1.0*kp_star*(robotdata->q_dot_a(1) - robotdata->vCmd(1)) + pOffset[1];
        if (robotdata->pFootb_tgt2(1) < 0.065)
            robotdata->pFootb_tgt2(1) = 0.065;
        if (robotdata->pFootb_tgt2(1) > 0.25)
            robotdata->pFootb_tgt2(1) = 0.25;
    }
    else{
        robotdata->pFootb_tgt2(1) =  (robotdata->q_dot_a(1) - d2)/sigma2 - 1.0*kp_star*(robotdata->q_dot_a(1) - robotdata->vCmd(2)) + pOffset[1];
        if (robotdata->pFootb_tgt2(1) > -0.065)
            robotdata->pFootb_tgt2(1) = -0.065;
        if (robotdata->pFootb_tgt2(1) < -0.25)
            robotdata->pFootb_tgt2(1) = -0.25;
    }

    //change period T
    // if(robotdata->s > 0.3 && robotdata->s<0.8 && ((robotdata->stance_index==1 && pFootStrike(1) + robotdata->q_a(1)<0.101) || (robotdata->stance_index==0 && pFootStrike(1) + robotdata->q_a(1)>-0.101) || fabs(pFootStrike(1)) > 0.249)){
    if(robotdata->s > 0.3 && robotdata->s<0.8 && (fabs(pFootStrike(0)-0.0538)> 0.239|| fabs(pFootStrike(1)) < 0.066 || fabs(pFootStrike(1)) > 0.249)){
        robotdata->T = robotdata->Td2;
        robotdata->t = robotdata->s*robotdata->T + robotdata->Tc;
        robotdata->t_switch = robotdata->time - robotdata->t;

        gaitPlan::predTouchState(robotdata);

        // sigma1 = 2.0*lambda/std::tanh(0.5*robotdata->T*lambda);
        // sigma2 = 1.0*lambda*std::tanh(0.5*robotdata->T*lambda);

        // kp_star = 1./lambda/std::sinh(robotdata->T*lambda);

        pFootStrike(0) =  vTorso_td(0)/sigma1 + 0.8*kp_star*(vTorso_td(0)-robotdata->vCmd(0)) + pOffset[0];

        if (robotdata->stance_index==1){
            pFootStrike(1) =  (vTorso_td(1) - d2)/sigma2 - 1.0*kp_star*(vTorso_td(1) - robotdata->vCmd(1)) + pOffset[1];
            if (pFootStrike(1)<0.065)
                pFootStrike(1) = 0.065;
            if (pFootStrike(1)>0.25)
                pFootStrike(1) = 0.25;
        }
        else{
            pFootStrike(1) =  (vTorso_td(1) - d2)/sigma2 - 1.0*kp_star*(vTorso_td(1) - robotdata->vCmd(2)) + pOffset[1];
            if (pFootStrike(1) > -0.065)
                pFootStrike(1) = -0.065;
            if (pFootStrike(1)<-0.25)
                pFootStrike(1) = -0.25;
        }
        if(pFootStrike(0) - 0.0538 > 0.24){
            pFootStrike(0) = 0.24 + 0.0538;
        }
        else if(pFootStrike(0) - 0.0538 < -0.24){
            pFootStrike(0) = -0.24 + 0.0538;
        } 
    }

    if(robotdata->prestand == true){
        if (robotdata->stance_index==1){
            pFootStrike(1) =  0.24 - robotdata->q_a(1);
        }
        else{
            pFootStrike(1) =  -0.24 - robotdata->q_a(1);
        }
        // pFootStrike(0) = - robotdata->q_a(0);
    }

    if(robotdata->stance_index==0){
        pFoot = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0,3,1,3).transpose();
        vFoot = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(1,3,1,3).transpose();
    }
    else{
        pFoot = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0,3,1,3).transpose();
        vFoot = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(1,3,1,3).transpose();
    }
    if(robotdata->t < 0.5*robotdata->dt){
        robotdata->qCmd_td = qCmd;
        robotdata->qDotCmd_td = qDotCmd;

        robotdata->pTorso_td = ( - robotdata->pFootb_tgt);
        robotdata->vTorso_td = ( - robotdata->vFootb_tgt);

        robotdata->pFootb_td = (- robotdata->pTorso_tgt);
        robotdata->vFootb_td = (- robotdata->vTorso_tgt);

        robotdata->pFoot_td = pFoot;
        robotdata->vFoot_td = vFoot;

        robotdata->rFoot_td = robotdata->rTorso_tgt;
        robotdata->rTorso_td = robotdata->rFoot_tgt;

        robotdata->pArm_td = robotdata->pArm_tgt;
        robotdata->vArm_td = robotdata->vArm_tgt;
    }
 
    robotdata->pFootb_end(0) = pFootStrike(0);  
    robotdata->pFootb_end(1) = pFootStrike(1);
    robotdata->pFootb_end(2) = pFootStrike(2)-robotdata->task_card_set[robotdata->body_task_id]->X_a(0,5);
    
    robotdata->temp.head(3) = pFootStrike;

    robotdata->vFootb_end(0) = -vTorso_td(0);
    robotdata->vFootb_end(1) = 0.0;//-vTorso_td(1);
    robotdata->vFootb_end(2) = vZEnd;//-robotdata->task_card_set[robotdata->body_task_id]->X_a(1,5);

    robotdata->temp.block(3,0,3,1) = robotdata->vFootb_end;
// ------------------------
    
    // if(robotdata->stance_index == 0)
    // {
    //     robotdata->pFootb_end(1) = -0.08;  
    // }else{
    //     robotdata->pFootb_end(1) = 0.08;  
    // }
    // robotdata->pFootb_end(0) = 0.08;
    // // robotdata->pFootb_end(1) = 0;
    // robotdata->pFootb_end(2) = -robotdata->task_card_set[robotdata->body_task_id]->X_a(0,5);
    // robotdata->vFootb_end(0) = 0;
    // robotdata->vFootb_end(1) = 0;
    // robotdata->vFootb_end(2) = vZEnd;

    // // robotdata->pTorso_end(2) = -xStand(2);
    // // robotdata->vTorso_end(2) = 0.0;
// -----------------------------
    robotdata->pTorso_end(2) = hd;
    robotdata->vTorso_end(2) = 0.0;

    robotdata->rTorso = rotX(robotdata->q_a(3))*rotY(robotdata->q_a(4))*rotZ(robotdata->q_a(5));
    robotdata->rTorso_d = Eigen::Matrix3d::Identity();

    double sc, sL, sR, sfc, sfL, sfR;
    if(robotdata->t < robotdata->Tc - 0.5*robotdata->dt){
        sc = robotdata->t/robotdata->Tc;
    }
    else{
        sc = 1.0;
    }
    if(robotdata->stance_index==0){
        sL = sc;
        sR = 1.0 - sc;
    }
    else{
        sR = sc;
        sL = 1.0 -sR;
    }
    robotdata->sL = sL;
    robotdata->sR = sR;
    robotdata->sc = sc;

    if(robotdata->t < robotdata->t_ftd - 0.5*robotdata->dt){
        sfc = 0.0;
    }
    else if(robotdata->t < robotdata->Tc + robotdata->t_ftd - 0.5*robotdata->dt){
        sfc = (robotdata->t - robotdata->t_ftd)/robotdata->Tc;
    }
    else{
        sfc = 1.0;
    }
    if(robotdata->stance_index==0){
        sfL = sfc;
        sfR = 1.0 - sfc;
    }
    else{
        sfR = sfc;
        sfL = 1.0 -sfR;
    }

    robotdata->sfL = sfL;
    robotdata->sfR = sfR;
    robotdata->sfc = sfc;
    gaitPlan::swingPlan(robotdata);
    gaitPlan::torsoPlan(robotdata);
    gaitPlan::cart2Joint(robotdata);

    qCmd_ = qCmd;
    qDotCmd_ = qDotCmd;

    if(robotdata->t < robotdata->Tc - 0.5*robotdata->dt){
        qCmd_ = (1.0 - sc)*robotdata->qCmd_td + sc*qCmd;
        qDotCmd_ = (1.0 - sc)*robotdata->qDotCmd_td + sc*robotdata->q_dot_a.tail(12);
    }
    else if(robotdata->t < 2*robotdata->Tc - 0.5*robotdata->dt){
        double sc2 = (robotdata->t - robotdata->Tc)/robotdata->Tc;
        qDotCmd_ = (1.0 - sc2)*robotdata->q_dot_a.tail(12) + sc2*qDotCmd;
    }


    //Weights
    Eigen::VectorXd weight_FootForce = Eigen::VectorXd::Zero(12);
    weight_FootForce << 0.1, 0.1, 0.1, 0.1, 0.1, 0.001, 0.1, 0.1, 0.1, 0.1, 0.1, 0.001;
    for(int i=0; i<12; ++i){
        robotdata->WF1(i,i) = weight_FootForce(i);
    }

    robotdata->WF2 = 0.001*Eigen::MatrixXd::Identity(12,12);
    robotdata->WF2(5,5) = 0.0001;
    robotdata->WF2(11,11) = 0.0001;

    robotdata->task_card_set[robotdata->body_task_id]->weight << 100., 100., 100., 0., 0., 100.;
    robotdata->task_card_set[robotdata->left_foot_id]->weight = (sL * 1000. +  sR * 100.)* Eigen::VectorXd::Ones(6);
    robotdata->task_card_set[robotdata->right_foot_id]->weight = (sR * 1000. +  sL * 100.)* Eigen::VectorXd::Ones(6);

    //Bounds
    Eigen::VectorXd torSw = Eigen::VectorXd::Zero(12);
    for(int i=0; i<6; i++){
        torSw(i) = jointP(i)*(qCmd(i) - robotdata->q_a(6+i)) + jointD(i)*(qDotCmd(i) - robotdata->q_dot_a(6+i));
        torSw(i+6) = jointP(i)*(qCmd(i+6) - robotdata->q_a(12+i)) + jointD(i)*(qDotCmd(i+6) - robotdata->q_dot_a(12+i));
    }
    robotdata->tau_ub << 82.5, 82.5, 90.0, 90.0, 32.0, 32.0, 82.5, 82.5, 90.0, 90.0, 32.0, 32.0;
    robotdata->tau_lb = -robotdata->tau_ub;

    // robotdata->tau_ub.tail(6) =  (1.0 - sL)*robotdata->tau_ub.tail(6) + sL*torSw.tail(6);
    // robotdata->tau_lb.tail(6) =  (1.0 - sL)*robotdata->tau_lb.tail(6) + sL*torSw.tail(6);
    // robotdata->tau_ub.head(6) =  (1.0 - sR)*robotdata->tau_ub.head(6) + sR*torSw.head(6);
    // robotdata->tau_lb.head(6) =  (1.0 - sR)*robotdata->tau_lb.head(6) + sR*torSw.head(6);
    // if(robotdata->step < 1){
    //     robotdata->tau_ub.head(6) =  torSw.head(6);
    //     robotdata->tau_lb.head(6) =  torSw.head(6);
    // }

    double my_inf = 1000.0;
    robotdata->GRF_ub << my_inf, my_inf, my_inf, my_inf, my_inf, sL*1.5*robotdata->MG, my_inf, my_inf, my_inf, my_inf, my_inf, sR*1.5*robotdata->MG;
    // if(robotdata->step < 1){
    //     if(robotdata->stance_index==1){
    //         robotdata->GRF_ub << my_inf, my_inf, my_inf, my_inf, my_inf, 0.0, my_inf, my_inf, my_inf, my_inf, my_inf, 1.5*robotdata->MG;
    //     }
    //     else{
    //         robotdata->GRF_ub << my_inf, my_inf, my_inf, my_inf, my_inf, 1.5*robotdata->MG, my_inf, my_inf, my_inf, my_inf, my_inf, 0.0;
    //     }
    // }
    robotdata->GRF_lb = -robotdata->GRF_ub;
    robotdata->GRF_lb(5) = 0.0; robotdata->GRF_lb(11) = 0.0;
    // ankle control
    double akRoll_tor = 0.0;
    double akPitch_tor = 0.0;
    if (robotdata->touch_index==4){
        akPitch_tor = 24.0*(robotdata->vCmd[0] - vTorso_td[0]);
        akRoll_tor = -18.0*(robotdata->vCmd[2] - vTorso_td[1]);
    }
    else if(robotdata->touch_index==2){
        akPitch_tor = 24.0*(robotdata->vCmd[0] - vTorso_td[0]);
        akRoll_tor = -18.0*(robotdata->vCmd[1] - vTorso_td[1]);
    }
    else{
        akPitch_tor = 0.0;
        akRoll_tor = 0.0;
    }

    akPitch_tor = std::min(7.0, std::max(-7.0, akPitch_tor));
    akRoll_tor = std::min(2.0, std::max(-2.0, akRoll_tor));

    if(robotdata->stance_index==0){
        robotdata->GRF_ub(0) = sL*akRoll_tor;
        robotdata->GRF_ub(1) = sL*akPitch_tor;
        robotdata->GRF_lb(0) = sL*akRoll_tor;
        robotdata->GRF_lb(1) = sL*akPitch_tor;
        robotdata->GRF_ub(6) = 0.0;
        robotdata->GRF_ub(7) = 0.0;
        robotdata->GRF_lb(6) = 0.0;
        robotdata->GRF_lb(7) = 0.0;
    }
    else{
        robotdata->GRF_ub(6) = sR*akRoll_tor;
        robotdata->GRF_ub(7) = sR*akPitch_tor;
        robotdata->GRF_lb(6) = sR*akRoll_tor;
        robotdata->GRF_lb(7) = sR*akPitch_tor;
        robotdata->GRF_ub(0) = 0.0;
        robotdata->GRF_ub(1) = 0.0;
        robotdata->GRF_lb(0) = 0.0;
        robotdata->GRF_lb(1) = 0.0;
    }
    
    
    

    //PD factor
    // double ratio = 0.9;
    // if(robotdata->step < 1){
    //     robotdata->q_factor << Eigen::VectorXd::Ones(6), (1.0-ratio*sfR)*Eigen::VectorXd::Ones(6);
    //     robotdata->q_dot_factor << Eigen::VectorXd::Ones(6), (1.0-ratio*sfR)*Eigen::VectorXd::Ones(6);
    //     robotdata->q_factor.segment(10,2) << (1.0-ratio*sR), (1.0-ratio*sR);
    //     robotdata->q_dot_factor.segment(10,2) << (1.0-ratio*sR), (1.0-ratio*sR);
    // }
    // robotdata->q_factor << (1.0-ratio*sfL)*Eigen::VectorXd::Ones(6), (1.0-ratio*sfR)*Eigen::VectorXd::Ones(6);
    // robotdata->q_dot_factor << (1.0-ratio*sfL)*Eigen::VectorXd::Ones(6), (1.0-ratio*sfR)*Eigen::VectorXd::Ones(6);

    // robotdata->q_factor.segment(4,2) <<  (1.0-ratio*sL),  (1.0-ratio*sL);
    // robotdata->q_dot_factor.segment(4,2) << (1.0-ratio*sL),  (1.0-ratio*sL);
    // robotdata->q_factor.segment(10,2) <<  (1.0-ratio*sR),  (1.0-ratio*sR);
    // robotdata->q_dot_factor.segment(10,2) << (1.0-ratio*sR),  (1.0-ratio*sR);

    double ratio = 0.9;

    robotdata->q_factor << (1.0-ratio*sfL)*Eigen::VectorXd::Ones(6), (1.0-ratio*sfR)*Eigen::VectorXd::Ones(6);
    robotdata->q_dot_factor << (1.0-ratio*sfL)*Eigen::VectorXd::Ones(6), (1.0-ratio*sfR)*Eigen::VectorXd::Ones(6);

    // robotdata->q_factor.segment(4,2) <<  (1.0-ratio*sL),  (1.0-ratio*sL);
    // robotdata->q_dot_factor.segment(4,2) << (1.0-ratio*sL),  (1.0-ratio*sL);
    // robotdata->q_factor.segment(10,2) <<  (1.0-ratio*sR),  (1.0-ratio*sR);
    // robotdata->q_dot_factor.segment(10,2) << (1.0-ratio*sR),  (1.0-ratio*sR);

    robotdata->q_factor.segment(4,2) <<  (1.0-sL),  (1.0-sL);
    robotdata->q_dot_factor.segment(4,2) << (1.0-sL),  (1.0-sL);
    robotdata->q_factor.segment(10,2) <<  (1.0-sR),  (1.0-sR);
    robotdata->q_dot_factor.segment(10,2) << (1.0-sR),  (1.0-sR);

    // if(robotdata->stance_index==0){
    //     robotdata->q_factor.segment(4,2) <<  0.0,  0.0;
    //     robotdata->q_dot_factor.segment(4,2) << 0.0, 0.0;
    //     robotdata->q_factor.segment(10,2) <<  (1.0-sR),  (1.0-sR);
    //     robotdata->q_dot_factor.segment(10,2) << (1.0-sR),  (1.0-sR);
    // }else{
    //     robotdata->q_factor.segment(10,2) <<  0.0,  0.0;
    //     robotdata->q_dot_factor.segment(10,2) << 0.0, 0.0;
    //     robotdata->q_factor.segment(4,2) <<  (1.0-sL),  (1.0-sL);
    //     robotdata->q_dot_factor.segment(4,2) << (1.0-sL),  (1.0-sL);
    // } 

    if(robotdata->step < 1){
        if(robotdata->stance_index==1){
            robotdata->q_factor << (1.0-ratio*sL)*Eigen::VectorXd::Ones(6), (1.0-ratio)*Eigen::VectorXd::Ones(6);
            robotdata->q_dot_factor << (1.0-ratio*sL)*Eigen::VectorXd::Ones(6), (1.0-ratio)*Eigen::VectorXd::Ones(6);
            robotdata->q_factor.segment(10,2) <<  0.0,  0.0;
            robotdata->q_dot_factor.segment(10,2) <<  0.0,  0.0;
            robotdata->q_factor.segment(4,2) <<  (1.0-ratio*sL),  (1.0-ratio*sL);
            robotdata->q_dot_factor.segment(4,2) << (1.0-ratio*sL),  (1.0-ratio*sL);
        }else{
            robotdata->q_factor << (1.0-ratio)*Eigen::VectorXd::Ones(6), (1.0-ratio*sR)*Eigen::VectorXd::Ones(6);
            robotdata->q_dot_factor << (1.0-ratio)*Eigen::VectorXd::Ones(6), (1.0-ratio*sR)*Eigen::VectorXd::Ones(6);
            robotdata->q_factor.segment(4,2) <<  0.0,  0.0;
            robotdata->q_dot_factor.segment(4,2) << 0.0,  0.0;
            robotdata->q_factor.segment(10,2) <<  (1.0-ratio*sR),  (1.0-ratio*sR);
            robotdata->q_dot_factor.segment(10,2) << (1.0-ratio*sR),  (1.0-ratio*sR);
        }
        
    }


    // if(robotdata->flag_ankle==1){
    //     if(robotdata->stance_index==1){
    //         robotdata->q_factor.segment(4,2) <<  0.0,  0.0;
    //         robotdata->q_dot_factor.segment(4,2) << 0.0,  0.0;
    //     }
    //     else{
    //         robotdata->q_factor.segment(10,2) <<  0.0,  0.0;
    //         robotdata->q_dot_factor.segment(10,2) << 0.0,  0.0;
    //     }
    // }
    
    // if(robotdata->stance_index==0){
    //     robotdata->q_factor.segment(4,2) << 0.1, 0.1;
    //     robotdata->q_dot_factor.segment(4,2) << 0.1, 0.1;
    // }
    // else{
    //     robotdata->q_factor.segment(10,2) << 0.1, 0.1;
    //     robotdata->q_dot_factor.segment(10,2) << 0.1, 0.1;
    // }

    return true;
}

bool gaitPlan::stepPlan(Robot_Data *robotdata, Eigen::VectorXd &qCmd_, Eigen::VectorXd &qDotCmd_){
    if(robotdata->stance_index==0){
        pFoot = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0,3,1,3).transpose();
        vFoot = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(1,3,1,3).transpose();
    }
    else{
        pFoot = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0,3,1,3).transpose();
        vFoot = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(1,3,1,3).transpose();
    }
    if(robotdata->t < 0.5*robotdata->dt){
        robotdata->qCmd_td = qCmd;
        robotdata->qDotCmd_td = qDotCmd;

        robotdata->pTorso_td = ( - robotdata->pFootb_tgt);
        robotdata->vTorso_td = ( - robotdata->vFootb_tgt);

        robotdata->pFootb_td = (- robotdata->pTorso_tgt);
        robotdata->vFootb_td = (- robotdata->vTorso_tgt);

        robotdata->pFoot_td = pFoot;
        robotdata->vFoot_td = vFoot;

        robotdata->rFoot_td = robotdata->rTorso_tgt;
        robotdata->rTorso_td = robotdata->rFoot_tgt;
    }
    if(robotdata->stance_index == 0)
    {
        robotdata->pFootb_end(1) = xStand(4);  
    }else{
        robotdata->pFootb_end(1) = xStand(1);  
    }
    robotdata->pFootb_end(0) = xStand(0);
    // robotdata->pFootb_end(1) = 0;
    robotdata->pFootb_end(2) = -robotdata->task_card_set[robotdata->body_task_id]->X_a(0,5);
    robotdata->vFootb_end(0) = 0;
    robotdata->vFootb_end(1) = 0;
    robotdata->vFootb_end(2) = vZEnd;

    robotdata->pTorso_end(2) = -xStand(2);
    robotdata->vTorso_end(2) = 0.0;

    robotdata->rTorso = Eigen::Matrix3d::Identity();
    robotdata->rTorso_d = Eigen::Matrix3d::Identity();

    double sc, sL, sR, sfc, sfL, sfR;
    if(robotdata->t < robotdata->Tc - 0.5*robotdata->dt){
        sc = robotdata->t/robotdata->Tc;
    }
    else{
        sc = 1.0;
    }
    if(robotdata->stance_index==0){
        sL = sc;
        sR = 1.0 - sc;
    }
    else{
        sR = sc;
        sL = 1.0 -sR;
    }
    robotdata->sL = sL;
    robotdata->sR = sR;
    robotdata->sc = sc;

    if(robotdata->t < robotdata->t_ftd - 0.5*robotdata->dt){
        sfc = 0.0;
    }
    else if(robotdata->t < robotdata->Tc + robotdata->t_ftd - 0.5*robotdata->dt){
        sfc = (robotdata->t - robotdata->t_ftd)/robotdata->Tc;
    }
    else{
        sfc = 1.0;
    }
    if(robotdata->stance_index==0){
        sfL = sfc;
        sfR = 1.0 - sfc;
    }
    else{
        sfR = sfc;
        sfL = 1.0 -sfR;
    }

    robotdata->sfL = sfL;
    robotdata->sfR = sfR;
    robotdata->sfc = sfc;
    gaitPlan::swingPlan(robotdata);
    gaitPlan::torsoPlan(robotdata);
    gaitPlan::cart2Joint(robotdata);

    qCmd_ = qCmd;
    qDotCmd_ = qDotCmd;

    if(robotdata->t < robotdata->Tc - 0.5*robotdata->dt){
        qCmd_ = (1.0 - sc)*robotdata->qCmd_td + sc*qCmd;
        qDotCmd_ = (1.0 - sc)*robotdata->qDotCmd_td + sc*robotdata->q_dot_a.tail(12);
    }
    else if(robotdata->t < 2*robotdata->Tc - 0.5*robotdata->dt){
        double sc2 = (robotdata->t - robotdata->Tc)/robotdata->Tc;
        qDotCmd_ = (1.0 - sc2)*robotdata->q_dot_a.tail(12) + sc2*qDotCmd;
    }


    //Weights
    Eigen::VectorXd weight_FootForce = Eigen::VectorXd::Zero(12);
    weight_FootForce << 4., 4., 4., 0.1, 0.1, 0.001, 4., 4., 4., 0.1, 0.1, 0.001;
    for(int i=0; i<12; ++i){
        robotdata->WF1(i,i) = weight_FootForce(i);
    }

    robotdata->WF2 = 0.001*Eigen::MatrixXd::Identity(12,12);

    // robotdata->task_card_set[robotdata->body_task_id]->weight << 100., 100., 100., 0., 0., 100.;
    robotdata->task_card_set[robotdata->body_task_id]->weight << 1., 1., 1., 0., 0., 100.;
    robotdata->task_card_set[robotdata->left_foot_id]->weight = (sL * 1000. +  sR * 100.)* Eigen::VectorXd::Ones(12);
    robotdata->task_card_set[robotdata->right_foot_id]->weight = (sR * 1000. +  sL * 100.)* Eigen::VectorXd::Ones(12);

    //Bounds
    Eigen::VectorXd torSw = Eigen::VectorXd::Zero(12);
    for(int i=0; i<6; i++){
        torSw(i) = jointP(i)*(qCmd(i) - robotdata->q_a(6+i)) + jointD(i)*(qDotCmd(i) - robotdata->q_dot_a(6+i));
        torSw(i+6) = jointP(i)*(qCmd(i+6) - robotdata->q_a(12+i)) + jointD(i)*(qDotCmd(i+6) - robotdata->q_dot_a(12+i));
    }
    robotdata->tau_ub << 82.5, 82.5, 100.0, 100.0, 32.0, 32.0, 82.5, 82.5, 100.0, 100.0, 32.0, 32.0;
    robotdata->tau_lb = -robotdata->tau_ub;
    robotdata->tau_ub.tail(6) =  (1.0 - sL)*robotdata->tau_ub.tail(6) + sL*torSw.tail(6);
    robotdata->tau_lb.tail(6) =  (1.0 - sL)*robotdata->tau_lb.tail(6) + sL*torSw.tail(6);
    robotdata->tau_ub.head(6) =  (1.0 - sR)*robotdata->tau_ub.head(6) + sR*torSw.head(6);
    robotdata->tau_lb.head(6) =  (1.0 - sR)*robotdata->tau_lb.head(6) + sR*torSw.head(6);
    if(robotdata->step < 1){
        robotdata->tau_ub.head(6) =  torSw.head(6);
        robotdata->tau_lb.head(6) =  torSw.head(6);
    }

    double my_inf = 1000.0;
    robotdata->GRF_ub << my_inf, my_inf, my_inf, my_inf, my_inf, sL*1.5*robotdata->MG, my_inf, my_inf, my_inf, my_inf, my_inf, sR*1.5*robotdata->MG;
    if(robotdata->step < 1){
        robotdata->GRF_ub << my_inf, my_inf, my_inf, my_inf, my_inf, 0.0, my_inf, my_inf, my_inf, my_inf, my_inf, 1.5*robotdata->MG;
    }
    robotdata->GRF_lb = -robotdata->GRF_ub;
    robotdata->GRF_lb(5) = 0.0; robotdata->GRF_lb(11) = 0.0;

    //PD factor
    double ratio = 0.9;
    if(robotdata->step < 1){
        robotdata->q_factor << Eigen::VectorXd::Ones(6), (1.0-ratio*sfR)*Eigen::VectorXd::Ones(6);
        robotdata->q_dot_factor << Eigen::VectorXd::Ones(6), (1.0-ratio*sfR)*Eigen::VectorXd::Ones(6);
    }
    robotdata->q_factor << (1.0-ratio*sfL)*Eigen::VectorXd::Ones(6), (1.0-ratio*sfR)*Eigen::VectorXd::Ones(6);
    robotdata->q_dot_factor << (1.0-ratio*sfL)*Eigen::VectorXd::Ones(6), (1.0-ratio*sfR)*Eigen::VectorXd::Ones(6);

    return true;
}

bool gaitPlan::swingPlan(Robot_Data *robotdata){

    if(robotdata->t < robotdata->Tc - 0.5*robotdata->dt){
        robotdata->pFootb_tgt = pFoot - robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0,3,1,3).transpose();
        robotdata->vFootb_tgt = Eigen::VectorXd::Zero(3);//(vFoot - robotdata->task_card_set[robotdata->body_id]->X_a.block(1,3,1,3).transpose());
        robotdata->pFootb_ini = robotdata->pFootb_tgt;
        robotdata->vFootb_ini = robotdata->vFootb_tgt;
        robotdata->rFoot_tgt = robotdata->rTorso;
    }
    else{
        for(int i = 0; i < 2; i++){
            //Thirdpoly(robotdata->pFootb_ini(i), robotdata->vFootb_ini(i), robotdata->pFootb_end(i), robotdata->vFootb_end(i), robotdata->T, robotdata->t - robotdata->Tc + robotdata->dt, robotdata->pFootb_tgt(i) , robotdata->vFootb_tgt(i));
            Thirdpoly(robotdata->pFootb_tgt(i), robotdata->vFootb_tgt(i), robotdata->pFootb_end(i), robotdata->vFootb_end(i), robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pFootb_tgt(i) , robotdata->vFootb_tgt(i));
        }
        double xDDotTrans;
        // TriPointsQuintic(robotdata->T, robotdata->t - robotdata->Tc + robotdata->dt, robotdata->pFootb_ini(2), robotdata->vFootb_ini(2), zMid+robotdata->pFootb_end(2), 0., robotdata->pFootb_end(2), robotdata->vFootb_end(2), robotdata->pFootb_tgt(2), robotdata->vFootb_tgt(2), xDDotTrans);
        //foot z replan
        double tMid_ = 0.5*robotdata->T - 0.5*robotdata->Tc + 0.5*robotdata->t;
        double zMid_, vMid_;
        TriPointsQuintic(robotdata->T, tMid_, robotdata->pFootb_ini(2), robotdata->vFootb_ini(2), zMid+robotdata->pFootb_end(2), 0., robotdata->pFootb_end(2), robotdata->vFootb_end(2), zMid_, vMid_, xDDotTrans);
        TriPointsQuintic(robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pFootb_tgt(2), robotdata->vFootb_tgt(2), zMid_, vMid_, robotdata->pFootb_end(2), robotdata->vFootb_end(2), robotdata->pFootb_tgt(2), robotdata->vFootb_tgt(2), xDDotTrans);

        robotdata->rFoot_tgt = robotdata->rTorso;
    }
     
    //ankle pose plan
    if(robotdata->t < 0.5*robotdata->dt){
        if(robotdata->vCmd(0)>-0.01){
            if(robotdata->stance_index==0){
                robotdata->rFoot_d = basicfunction::RotZ(-M_PI/22.)*basicfunction::RotY(M_PI/60.);
            }else{
                robotdata->rFoot_d = basicfunction::RotZ(M_PI/22.)*basicfunction::RotY(M_PI/60.);
            }
        }else{
            robotdata->rFoot_d = basicfunction::RotY(M_PI/60.);
        }
    }
    // if(robotdata->vCmd(0)>-0.01){
    //     if(robotdata->stance_index==0){
    //     robotdata->rFoot_d = basicfunction::RotZ(-M_PI/22.)*basicfunction::RotY(M_PI/60.);
    //     }else{
    //         robotdata->rFoot_d = basicfunction::RotZ(M_PI/22.)*basicfunction::RotY(M_PI/60.);
    //     }
    // }else{
    //     robotdata->rFoot_d = basicfunction::RotY(M_PI/60.);
    // }
     
    if(robotdata->stance_index==0){
        basicfunction::Euler_XYZToMatrix(robotdata->rFoot_l, robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0,0,1,3).transpose());
        if(robotdata->t < 2.0*robotdata->Tc){
            basicfunction::Euler_XYZToMatrix(robotdata->rFoot_r, robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0,0,1,3).transpose());
            robotdata->rFoot_rtd = robotdata->rFoot_r;
        }
        else{
            Eigen::Vector3d omiga_d, acc_d;
            quaternionInterp(robotdata->rFoot_rtd, robotdata->rFoot_d, 0.8*robotdata->T - 2.0*robotdata->Tc, robotdata->t - 2.0*robotdata->Tc + robotdata->dt,
                                                robotdata->rFoot_r, omiga_d, acc_d);
            // robotdata->rFoot_r = Eigen::Matrix3d::Identity();
        }
    }
    else{
        basicfunction::Euler_XYZToMatrix(robotdata->rFoot_r, robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0,0,1,3).transpose());
        if(robotdata->t < 2.0*robotdata->Tc){
            basicfunction::Euler_XYZToMatrix(robotdata->rFoot_l, robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0,0,1,3).transpose());
            robotdata->rFoot_ltd = robotdata->rFoot_l;
        }
        else{
            Eigen::Vector3d omiga_d, acc_d;
            quaternionInterp(robotdata->rFoot_ltd, robotdata->rFoot_d, 0.8*robotdata->T - 2.0*robotdata->Tc, robotdata->t - 2.0*robotdata->Tc + robotdata->dt,
                                                robotdata->rFoot_l, omiga_d, acc_d);
            // robotdata->rFoot_l = Eigen::Matrix3d::Identity();
        }     
    }
    
    //arm swing plan
    if(robotdata->t < 0.8*robotdata->T + robotdata->Tc){
        if(robotdata->stance_index==1){
            Thirdpoly(robotdata->pArm_tgt(0), robotdata->vArm_tgt(0), -(robotdata->pFootb_end(0)-0.0)/0.8 + 0.42, 0.0, 0.8*robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(0) , robotdata->vArm_tgt(0));
            Thirdpoly(robotdata->pArm_tgt(1), robotdata->vArm_tgt(1), (robotdata->pFootb_end(0)-0.0)/0.8 + 0.42, 0.0, 0.8*robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(1) , robotdata->vArm_tgt(1));
        }else{
                Thirdpoly(robotdata->pArm_tgt(0), robotdata->vArm_tgt(0), (robotdata->pFootb_end(0)-0.0)/0.8 + 0.42, 0.0, 0.8*robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(0) , robotdata->vArm_tgt(0));
                Thirdpoly(robotdata->pArm_tgt(1), robotdata->vArm_tgt(1), -(robotdata->pFootb_end(0)-0.0)/0.8 + 0.42, 0.0, 0.8*robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(1) , robotdata->vArm_tgt(1));
        }
        robotdata->pArm_tgt(0) = std::min(std::max(robotdata->pArm_tgt(0),0.0),1.0);
        robotdata->pArm_tgt(1) = std::min(std::max(robotdata->pArm_tgt(1),0.0),1.0);
    }
    
    // if(robotdata->stance_index==1){
    //         Thirdpoly(robotdata->pArm_tgt(0), robotdata->vArm_tgt(0), -(robotdata->pFootb_end(0)-0.0)/0.8 + 0.5, 0.0, robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(0) , robotdata->vArm_tgt(0));
    //         Thirdpoly(robotdata->pArm_tgt(1), robotdata->vArm_tgt(1), (robotdata->pFootb_end(0)-0.0)/0.8 + 0.5, 0.0, robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(1) , robotdata->vArm_tgt(1));
    // }else{
    //         Thirdpoly(robotdata->pArm_tgt(0), robotdata->vArm_tgt(0), (robotdata->pFootb_end(0)-0.0)/0.8 + 0.5, 0.0, robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(0) , robotdata->vArm_tgt(0));
    //         Thirdpoly(robotdata->pArm_tgt(1), robotdata->vArm_tgt(1), -(robotdata->pFootb_end(0)-0.0)/0.8 + 0.5, 0.0, robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(1) , robotdata->vArm_tgt(1));
    // }
    // robotdata->pArm_tgt(0) = std::min(std::max(robotdata->pArm_tgt(0),0.0),1.0);
    // robotdata->pArm_tgt(1) = std::min(std::max(robotdata->pArm_tgt(1),0.0),1.0);

    return true;
}

bool gaitPlan::torsoPlan(Robot_Data *robotdata){
    robotdata->rTorso_d = basicfunction::RotX(-0.15*robotdata->avg_vx*robotdata->vyaw);

    if(robotdata->t < robotdata->Tc - 0.5*robotdata->dt){
        robotdata->pTorso_tgt = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0,3,1,3).transpose();
        robotdata->pTorso_tgt(2) = robotdata->pTorso_td(2);
        robotdata->vTorso_tgt = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1,3,1,3).transpose();
        robotdata->vTorso_tgt(2) = 0.0;
        robotdata->pTorso_ini = robotdata->pTorso_tgt;
        robotdata->vTorso_ini = robotdata->vTorso_tgt;
        robotdata->rTorso_tgt = robotdata->rTorso_d;
    }
    else{
        robotdata->pTorso_tgt = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0,3,1,3).transpose();
        robotdata->vTorso_tgt = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1,3,1,3).transpose();
        Thirdpoly(robotdata->pTorso_ini(2), robotdata->vTorso_ini(2), robotdata->pTorso_end(2), robotdata->vTorso_end(2), robotdata->T, robotdata->t - robotdata->Tc + robotdata->dt, robotdata->pTorso_tgt(2) , robotdata->vTorso_tgt(2));
        //Thirdpoly(robotdata->pTorso_tgt(2), robotdata->vTorso_tgt(2), robotdata->pTorso_end(2), robotdata->vTorso_end(2), robotdata->T, robotdata->t - robotdata->Tc + robotdata->dt, robotdata->pTorso_tgt(2) , robotdata->vTorso_tgt(2));
    
        robotdata->rTorso_tgt = robotdata->rTorso_d;
    }

    return true;
}

bool gaitPlan::cart2Joint(Robot_Data *robotdata){
    Eigen::Vector3d PosL, PosR;
    Eigen::Vector3d RPY(0.0, 0.0, 0.0);
    Eigen::VectorXd qIKL = Eigen::VectorXd::Zero(6), qIKR = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd xIKL_dot = Eigen::VectorXd::Zero(6), xIKR_dot = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd qIKL_dot = Eigen::VectorXd::Zero(6), qIKR_dot = Eigen::VectorXd::Zero(6);
    if(robotdata->stance_index==0){
        PosL = robotdata->rTorso_tgt.transpose()*(-robotdata->pTorso_tgt) - offSetL;
        PosR = robotdata->rFoot_tgt.transpose()*(robotdata->pFootb_tgt) - offSetR;
        oneLegIK2(PosL, robotdata->rTorso_tgt.transpose()*robotdata->rFoot_l, qIKL);
        oneLegIK2(PosR, robotdata->rFoot_tgt.transpose()*robotdata->rFoot_r, qIKR);
    }
    else{
        PosL = robotdata->rFoot_tgt.transpose()*(robotdata->pFootb_tgt) - offSetL;
        PosR = robotdata->rTorso_tgt.transpose()*(-robotdata->pTorso_tgt) - offSetR;
        oneLegIK2(PosL, robotdata->rFoot_tgt.transpose()*robotdata->rFoot_l, qIKL);
        oneLegIK2(PosR, robotdata->rTorso_tgt.transpose()*robotdata->rFoot_r, qIKR);
    }
    qCmd_pre = qCmd;
    
    qCmd.head(6) = qIKL;
    qCmd.tail(6) = qIKR;
    // todo velocity ik
    Eigen::VectorXd qCmd_full = Eigen::VectorXd::Zero(18);
    qCmd_full.tail(12) = qCmd;
    Eigen::MatrixXd J_6D;
    J_6D.setZero(6,robotdata->ndof);
    if(robotdata->stance_index==0){
        // left foot
        qCmd_full.segment(3,3) = matrixtoeulerxyz_(robotdata->rTorso_tgt);
        RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model),qCmd_full,
                                            robotdata->task_card_set[robotdata->left_foot_id]->joint_id,robotdata->task_card_set[robotdata->left_foot_id]->T_offset.block(0,3,3,1),
                                            J_6D,false);
        xIKL_dot.setZero();
        xIKL_dot(2) = -robotdata->vyaw;
        xIKL_dot.tail(3) = -robotdata->vTorso_tgt;
        qIKL_dot = J_6D.block(0,6,6,6).completeOrthogonalDecomposition().pseudoInverse()*(xIKL_dot);
        // right foot
        qCmd_full.segment(3,3) = matrixtoeulerxyz_(robotdata->rFoot_tgt);
        RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model),qCmd_full,
                                            robotdata->task_card_set[robotdata->right_foot_id]->joint_id,robotdata->task_card_set[robotdata->right_foot_id]->T_offset.block(0,3,3,1),
                                            J_6D,false);
        xIKR_dot.setZero();
        xIKR_dot.tail(3) = robotdata->vFootb_tgt;
        qIKR_dot = J_6D.block(0,12,6,6).completeOrthogonalDecomposition().pseudoInverse()*(xIKR_dot);
    }else{
        // left foot
        qCmd_full.segment(3,3) = matrixtoeulerxyz_(robotdata->rFoot_tgt);
        RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model),qCmd_full,
                                            robotdata->task_card_set[robotdata->left_foot_id]->joint_id,robotdata->task_card_set[robotdata->left_foot_id]->T_offset.block(0,3,3,1),
                                            J_6D,false);
        xIKL_dot.setZero();
        xIKL_dot.tail(3) = robotdata->vFootb_tgt;
        qIKL_dot = J_6D.block(0,6,6,6).completeOrthogonalDecomposition().pseudoInverse()*(xIKL_dot);
        // right foot
        qCmd_full.segment(3,3) = matrixtoeulerxyz_(-robotdata->rTorso_tgt);
        RigidBodyDynamics::CalcPointJacobian6D(*(robotdata->robot_model),qCmd_full,
                                            robotdata->task_card_set[robotdata->right_foot_id]->joint_id,robotdata->task_card_set[robotdata->right_foot_id]->T_offset.block(0,3,3,1),
                                            J_6D,false);
        xIKR_dot.setZero();
        xIKR_dot(2) = -robotdata->vyaw;
        xIKR_dot.tail(3) = -robotdata->vTorso_tgt;
        qIKR_dot = J_6D.block(0,12,6,6).completeOrthogonalDecomposition().pseudoInverse()*(xIKR_dot);
    }
    robotdata->qdotcmd_temp.head(6) = qIKL_dot;
    robotdata->qdotcmd_temp.tail(6) = qIKR_dot;

    qDotCmd.head(6) = qIKL_dot;
    qDotCmd.tail(6) = qIKR_dot;
    // if(!firstFlag)
    //     qDotCmd = (qCmd - qCmd_pre)/robotdata->dt;
    // firstFlag = 0;
    return true;
}

Eigen::Vector3d gaitPlan::matrixtoeulerxyz_(Eigen::Matrix3d R)
{
    Eigen::Vector3d euler;
    euler.setZero();
    // y (-pi/2 pi/2)
    euler(1) = asin(R(0,2));
    // z [-pi pi]
    double sinz = -R(0,1)/cos(euler(1));
    double cosz = R(0,0)/cos(euler(1));
    euler(2) = atan2(sinz,cosz);
    // x [-pi pi]
    double sinx = -R(1,2)/cos(euler(1));
    double cosx = R(2,2)/cos(euler(1));
    euler(0) = atan2(sinx,cosx);
    return euler;
}

void gaitPlan::quaternionInterp(Eigen::Matrix3d R_start,
                                                Eigen::Matrix3d R_end, 
                                                double totaltime, double currenttime,
                                                Eigen::Matrix3d& R_d, Eigen::Vector3d& omiga_d, Eigen::Vector3d& acc_d)
                                                {
                                                  RigidBodyDynamics::Math::Quaternion  Q_start = Q_start.fromMatrix(R_start).conjugate();
                                                  RigidBodyDynamics::Math::Quaternion  Q_end = Q_end.fromMatrix(R_end).conjugate();       
                                                  RigidBodyDynamics::Math::Quaternion  Q_d;
                                                   
                                                   RigidBodyDynamics::Math::Quaternion deltQ = Q_start.conjugate()*Q_end;
                                                  
                                                   Eigen::Vector3d p =  (deltQ.block<3,1>(0,0));
                                                   if (p.norm() != 0)
                                                   {
                                                      p = p/p.norm();
                                                   }
                                                   
                                                   double delttheta = 2*acos(deltQ(3));

                                                   Eigen::VectorXd p0 = Eigen::VectorXd::Zero(1);//p0 = p0 p0_dot p0_ddot p1_dot p1_ddot

                                                   Eigen::VectorXd p1 = Eigen::VectorXd::Zero(1);
                                                   p1[0] = delttheta;

                                                   Eigen::VectorXd pd = Eigen::VectorXd::Zero(1);
                                                   Eigen::VectorXd pd_dot = Eigen::VectorXd::Zero(1);
                                                   Eigen::VectorXd pd_ddot = Eigen::VectorXd::Zero(1);

                                                   FifthPoly(p0,p0,p0,p1,p0,p0,totaltime,currenttime,pd,pd_dot,pd_ddot);
                                                   deltQ.block<3,1>(0,0) = p*sin(pd[0]/2.0);
                                                   deltQ[3] = cos(pd[0]/2.0);
                                                   Q_d = Q_start*deltQ ;
                                                   //Q_d = Q_d/sqrt(Q_d.dot(Q_d));
                                                   
                                                   R_d = QuanteiniontoMatrix(Q_d);
                                                   omiga_d = R_start * p*pd_dot[0];
                                                   acc_d = R_start * p*pd_ddot[0];

                                                   //std::cout<<"R_d:"<<std::endl<<R_d<<std::endl;
                                                   //std::cout<<"Q_end:"<<std::endl<<Q_end<<std::endl;
                                                   //std::cout<<"omiga_d:"<<std::endl<<omiga_d.transpose()<<std::endl;
                                                   //std::cout<<"acc_d:"<<std::endl<<acc_d.transpose()<<std::endl;
}

Eigen::Matrix3d gaitPlan::QuanteiniontoMatrix(RigidBodyDynamics::Math::Quaternion Q) {
      double x = Q[0];
      double y = Q[1];
      double z =Q[2];
      double w = Q[3];
      Eigen::Matrix3d R;
      R<<    1 - 2*y*y - 2*z*z,
             2*x*y - 2*w*z,
             2*x*z + 2*w*y,

             2*x*y + 2*w*z,
             1 - 2*x*x - 2*z*z,
             2*y*z - 2*w*x,

             2*x*z - 2*w*y,
             2*y*z + 2*w*x,
             1 - 2*x*x - 2*y*y;
      return R;
    }

void gaitPlan::FifthPoly(Eigen::VectorXd p0, Eigen::VectorXd p0_dot, Eigen::VectorXd p0_dotdot,     // start point states 
                     Eigen::VectorXd p1, Eigen::VectorXd p1_dot, Eigen::VectorXd p1_dotdot,     // end point states 
                     double totalTime,       // total permating time 
                     double currenttime,     //current time,from 0 to total time 
                     Eigen::VectorXd& pd, Eigen::VectorXd& pd_dot, Eigen::VectorXd& pd_dotdot )
                     {
                         double t = currenttime;
                         double time = totalTime;
                         if(t < totalTime)
                         {
                            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6,6);
                            A << 1,         0,             0,          0,         0,            0,
                                0,         1,             0,          0,         0,            0,
                                0,         0,           1/2,          0,         0,            0,
                                -10/pow(time,3), -6/pow(time,2),   -3/(2*time),  10/pow(time,3), -4/pow(time,2),   1/(2*time),
                                15/pow(time,4),  8/pow(time,3),  3/(2*pow(time,2)), -15/pow(time,4),  7/pow(time,3),    -1/pow(time,2),
                                -6/pow(time,5), -3/pow(time,4), -1/(2*pow(time,3)),   6/pow(time,5), -3/pow(time,4), 1/(2*pow(time,3));
                            Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(6,1);
                            Eigen::MatrixXd a = Eigen::MatrixXd::Zero(6,1);
                            for(int i = 0; i < p0.size(); i++)
                            {
                                x0 << p0(i), p0_dot(i), p0_dotdot(i), p1(i), p1_dot(i), p1_dotdot(i);
                                a = A*x0;
                                pd(i) = a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t + a(5) * t * t * t * t * t;
                                pd_dot(i) = a(1) + 2 * a(2) * t + 3 * a(3) * t * t + 4 * a(4) * t * t * t + 5 * a(5) * t * t * t * t;
                                pd_dotdot(i) = 2 * a(2) + 6 * a(3) * t + 12 * a(4) * t * t + 20 * a(5) * t * t * t;
                            }
                         }else{
                             pd = p1;
                             pd_dot = p1_dot;
                             pd_dotdot = p1_dotdot;
                         }
                         
                     }
