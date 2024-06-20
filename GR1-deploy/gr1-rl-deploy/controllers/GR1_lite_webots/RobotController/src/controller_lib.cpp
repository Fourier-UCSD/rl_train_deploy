#include "../include/controller_lib.h"
// #include "controller_lib.h"
#include<iostream>
#include "../include/basicfunction.h"
// #include "basicfunction.h"
#include "rbdl/rbdl.h"
// #define DEBUG
Controller_Lib::Controller_Lib()
{

}
Controller_Lib::~Controller_Lib()
{

}

void Controller_Lib::controller_run()
{
    switch(controller_type)
    {
    case PositionController:
    {
        positioncontroller();
        break;
    }
    case PID:
    {
        pid();
        break;
    }
    case PID_force:
    {
        pid_force();
        break;
    }
    case PID_position:
    {
        pid_position();
        break;
    }
    case PID_orient:
    {
        pid_orient();
        break;
    }
    case Admittance_force_feeedback:
    {
        admittance_force_feedback();
        break;
    }
    case Impedance_force_feedback:
    {
        impedance_force_feedback();
        break;
    }
    case Admittance:
    {
        admittance();
        break;
    }
    case Impedance:
    {
        impedance();
        break;
    }
    case None:
    {
        none();
        break;
    }
    case Userdefined:
    {
        user_defined();
        break;
    }
    default:
    {
        std::cout<<"no task controller can be used!"<<std::endl;
        break;
    }
    }
}

void Controller_Lib::positioncontroller()
{
    output_data = input_data_d;
    // X_dot_c
    output_data.row(1) = input_data_d.row(1) + para(0)*(input_data_d.row(0) - input_data_a.row(0));
    // X_ddot_c
    output_data.row(2) = input_data_d.row(2) + para(2)*(input_data_d.row(1) - input_data_a.row(1)) + para(0)*(input_data_d.row(0) - input_data_a.row(0));
}

void Controller_Lib::pid_force()
{
  
    // F
    // alter_v.row(3) +=  para(1) * (input_data_d.row(3) - input_data_a.row(3));
    // output_data.row(3) = input_data_d.row(3) + para(0)*(input_data_d.row(3) - input_data_a.row(3));
}

void Controller_Lib::pid_position()
{

}

void Controller_Lib::pid()
{
    output_data = input_data_d;
    // X_dot_c
    output_data.row(1)(0) = input_data_d.row(1)(0) + para(3)*(input_data_d.row(0)(0) - input_data_a.row(0)(0));
    output_data.row(1)(1) = input_data_d.row(1)(1) + para(4)*(input_data_d.row(0)(1) - input_data_a.row(0)(1));
    output_data.row(1)(2) = input_data_d.row(1)(2) + para(5)*(input_data_d.row(0)(2) - input_data_a.row(0)(2));
    // X_ddot_c
    output_data.row(2)(0) = input_data_d.row(2)(0) + para(9)*(input_data_d.row(1)(0) - input_data_a.row(1)(0)) + para(3)*(input_data_d.row(0)(0) - input_data_a.row(0)(0));
    output_data.row(2)(1) = input_data_d.row(2)(1) + para(10)*(input_data_d.row(1)(1) - input_data_a.row(1)(1)) + para(4)*(input_data_d.row(0)(1) - input_data_a.row(0)(1));
    output_data.row(2)(2) = input_data_d.row(2)(2) + para(11)*(input_data_d.row(1)(2) - input_data_a.row(1)(2)) + para(5)*(input_data_d.row(0)(2) - input_data_a.row(0)(2));
    // F
    output_data.row(3) = input_data_d.row(3);// + para(0)*(input_data_d.row(3) - input_data_a.row(3));
    #ifdef DEBUG
    std::cout << "output_data: "  <<std::endl<< output_data <<std::endl;
    #endif
}

void Controller_Lib::pid_orient()
{
    output_data = input_data_d;
    int size_ = input_data_d.cols();
    // std::cout<<"size_: "<<size_<<std::endl;
    // std::cout <<  "input_data_d: " <<std::endl<< input_data_d <<std::endl;
    // std::cout <<  "input_data_a: " <<std::endl<< input_data_a <<std::endl;
    if(size_==6)
    {
        // std::cout <<  "input_data_d:hehe " <<std::endl;
        

        // orientation error(euler - xyz)
        Eigen::Vector3d euler_a = input_data_a.row(0).block(0,0,1,3).transpose();
        Eigen::Vector3d euler_d = input_data_d.row(0).block(0,0,1,3).transpose();
        Eigen::Matrix3d  R_a = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d  R_d = Eigen::AngleAxisd(euler_d[0], Eigen::Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(euler_d[1], Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(euler_d[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // Eigen::Vector3d euler_err = (R_a.transpose() *R_d).eulerAngles(0,1,2);
        // Eigen::MatrixXd pose_err = (input_data_d.row(0) - input_data_a.row(0));
        // pose_err.block(0,0,1,3) = (basicfunction::VelProjectionMatrix_EulerXYZ(euler_a) * euler_err).transpose();

        // orientation error
        RigidBodyDynamics::Math::Quaternion Quat_d = RigidBodyDynamics::Math::Quaternion::fromMatrix(R_d).conjugate();
        RigidBodyDynamics::Math::Quaternion Quat_a = RigidBodyDynamics::Math::Quaternion::fromMatrix(R_a).conjugate();
        Eigen::Vector3d Quat_err  =R_a*(Quat_a.conjugate()*Quat_d).block<3,1>(0,0);

        Eigen::MatrixXd pose_err = (input_data_d.row(0) - input_data_a.row(0));
        pose_err.block(0,0,1,3) = Quat_err.transpose();
        // X_c
        // output_data.row(0) = 100.0*pose_err;
        // output_data.row(0)(0) =  para(0)*pose_err(0);
        // output_data.row(0)(1) =  para(1)*pose_err(1);
        // output_data.row(0)(2) =  para(2)*pose_err(2);
        // output_data.row(0)(3) =  para(3)*pose_err(3);
        // output_data.row(0)(4) =  para(4)*pose_err(4);
        // output_data.row(0)(5) =  para(5)*pose_err(5);
        output_data.row(0)(0) =  pose_err(0);
        output_data.row(0)(1) =  pose_err(1);
        output_data.row(0)(2) =  pose_err(2);
        output_data.row(0)(3) =  pose_err(3);
        output_data.row(0)(4) =  pose_err(4);
        output_data.row(0)(5) =  pose_err(5);
        // std::cout<<"pose_err: "<<std::endl<<pose_err<<std::endl;
        // X_dot_c
        output_data.row(1)(0) = input_data_d.row(1)(0);// + para(0)*pose_err(0) ;//+ para(6)*(input_data_d.row(1)(0) - input_data_a.row(1)(0));
        output_data.row(1)(1) = input_data_d.row(1)(1);// + para(1)*pose_err(1) ;//+ para(7)*(input_data_d.row(1)(1) - input_data_a.row(1)(1));
        output_data.row(1)(2) = input_data_d.row(1)(2);// + para(2)*pose_err(2) ;//+ para(8)*(input_data_d.row(1)(2) - input_data_a.row(1)(2));
        output_data.row(1)(3) = input_data_d.row(1)(3);// + para(3)*pose_err(3) ;//+ para(9)*(input_data_d.row(1)(3) - input_data_a.row(1)(3));
        output_data.row(1)(4) = input_data_d.row(1)(4);// + para(4)*pose_err(4) ;//+ para(10)*(input_data_d.row(1)(4) - input_data_a.row(1)(4));
        output_data.row(1)(5) = input_data_d.row(1)(5);// + para(5)*pose_err(5) ;//+ para(11)*(input_data_d.row(1)(5) - input_data_a.row(1)(5));
        // X_ddot_c
        output_data.row(2)(0) = input_data_d.row(2)(0) + para(6)*(input_data_d.row(1)(0) - input_data_a.row(1)(0)) + para(0)*pose_err(0);
        output_data.row(2)(1) = input_data_d.row(2)(1) + para(7)*(input_data_d.row(1)(1) - input_data_a.row(1)(1)) + para(1)*pose_err(1);
        output_data.row(2)(2) = input_data_d.row(2)(2) + para(8)*(input_data_d.row(1)(2) - input_data_a.row(1)(2)) + para(2)*pose_err(2);
        output_data.row(2)(3) = input_data_d.row(2)(3) + para(9)*(input_data_d.row(1)(3) - input_data_a.row(1)(3)) + para(3)*pose_err(3);
        output_data.row(2)(4) = input_data_d.row(2)(4) + para(10)*(input_data_d.row(1)(4) - input_data_a.row(1)(4)) + para(4)*pose_err(4);
        output_data.row(2)(5) = input_data_d.row(2)(5) + para(11)*(input_data_d.row(1)(5) - input_data_a.row(1)(5)) + para(5)*pose_err(5);
        // std::cout<<"para: "<<para(9)<<para(10)<<para(11)<<para(3)<<para(4)<<para(5)<<std::endl;
    }else if(size_ == 3){
        // std::cout <<  "input_data_d: " <<std::endl<< input_data_d <<std::endl;
        // std::cout <<  "input_data_a: " <<std::endl<< input_data_a <<std::endl;

        // orientation error(euler - xyz)
        Eigen::Vector3d euler_a = input_data_a.row(0).block(0,0,1,3).transpose();
        Eigen::Vector3d euler_d = input_data_d.row(0).block(0,0,1,3).transpose();
        Eigen::Matrix3d  R_a = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d  R_d = Eigen::AngleAxisd(euler_d[0], Eigen::Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(euler_d[1], Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(euler_d[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // Eigen::Vector3d euler_err = (R_a.transpose() *R_d).eulerAngles(0,1,2);
        // Eigen::MatrixXd pose_err = (input_data_d.row(0) - input_data_a.row(0));
        // pose_err.block(0,0,1,3) = (basicfunction::VelProjectionMatrix_EulerXYZ(euler_a) * euler_err).transpose();

        // orientation error
        RigidBodyDynamics::Math::Quaternion Quat_d = RigidBodyDynamics::Math::Quaternion::fromMatrix(R_d).conjugate();
        RigidBodyDynamics::Math::Quaternion Quat_a = RigidBodyDynamics::Math::Quaternion::fromMatrix(R_a).conjugate();
        Eigen::Vector3d Quat_err  =R_a*(Quat_a.conjugate()*Quat_d).block<3,1>(0,0);

        Eigen::MatrixXd pose_err = (input_data_d.row(0) - input_data_a.row(0));
        pose_err.block(0,0,1,3) = Quat_err.transpose();
        // X_c
        // output_data.row(0) = 100.0*pose_err;
        // output_data.row(0)(0) = para(0)*pose_err(0);
        // output_data.row(0)(1) = para(1)*pose_err(1);
        // output_data.row(0)(2) = para(2)*pose_err(2);
        output_data.row(0)(0) = pose_err(0);
        output_data.row(0)(1) = pose_err(1);
        output_data.row(0)(2) = pose_err(2);
        // std::cout<<"pose_err: "<<std::endl<<pose_err<<std::endl;
        // X_dot_c
        output_data.row(1)(0) = input_data_d.row(1)(0);// + para(0)*pose_err(0) ;//+ para(6)*(input_data_d.row(1)(0) - input_data_a.row(1)(0));
        output_data.row(1)(1) = input_data_d.row(1)(1);// + para(1)*pose_err(1) ;//+ para(7)*(input_data_d.row(1)(1) - input_data_a.row(1)(1));
        output_data.row(1)(2) = input_data_d.row(1)(2);// + para(2)*pose_err(2) ;//+ para(8)*(input_data_d.row(1)(2) - input_data_a.row(1)(2));
//        output_data.row(1)(3) = input_data_d.row(1)(3) ;//+ para(3)*pose_err(3);
//        output_data.row(1)(4) = input_data_d.row(1)(4) ;//+ para(4)*pose_err(4);
//        output_data.row(1)(5) = input_data_d.row(1)(5) ;//+ para(5)*pose_err(5);
        // X_ddot_c
        output_data.row(2)(0) = input_data_d.row(2)(0) + para(6)*(input_data_d.row(1)(0) - input_data_a.row(1)(0)) + para(0)*pose_err(0);
        output_data.row(2)(1) = input_data_d.row(2)(1) + para(7)*(input_data_d.row(1)(1) - input_data_a.row(1)(1)) + para(1)*pose_err(1);
        output_data.row(2)(2) = input_data_d.row(2)(2) + para(8)*(input_data_d.row(1)(2) - input_data_a.row(1)(2)) + para(2)*pose_err(2);
//        output_data.row(2)(3) = input_data_d.row(2)(3) + para(9)*(input_data_d.row(1)(3) - input_data_a.row(1)(3)) + para(3)*pose_err(3);
//        output_data.row(2)(4) = input_data_d.row(2)(4) + para(10)*(input_data_d.row(1)(4) - input_data_a.row(1)(4)) + para(4)*pose_err(4);
//        output_data.row(2)(5) = input_data_d.row(2)(5) + para(11)*(input_data_d.row(1)(5) - input_data_a.row(1)(5)) + para(5)*pose_err(5);
    }else if(size_ == 4){
        // for three dims orientation and  one dim line
        // std::cout <<  "input_data_d: " <<std::endl<< input_data_d <<std::endl;
        // std::cout <<  "input_data_a: " <<std::endl<< input_data_a <<std::endl;

        // orientation error(euler - xyz)
        Eigen::Vector3d euler_a = input_data_a.row(0).block(0,0,1,3).transpose();
        Eigen::Vector3d euler_d = input_data_d.row(0).block(0,0,1,3).transpose();
        Eigen::Matrix3d  R_a = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d  R_d = Eigen::AngleAxisd(euler_d[0], Eigen::Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(euler_d[1], Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(euler_d[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // Eigen::Vector3d euler_err = (R_a.transpose() *R_d).eulerAngles(0,1,2);
        // Eigen::MatrixXd pose_err = (input_data_d.row(0) - input_data_a.row(0));
        // pose_err.block(0,0,1,3) = (basicfunction::VelProjectionMatrix_EulerXYZ(euler_a) * euler_err).transpose();

        // orientation error
        RigidBodyDynamics::Math::Quaternion Quat_d = RigidBodyDynamics::Math::Quaternion::fromMatrix(R_d).conjugate();
        RigidBodyDynamics::Math::Quaternion Quat_a = RigidBodyDynamics::Math::Quaternion::fromMatrix(R_a).conjugate();
        Eigen::Vector3d Quat_err  =R_a*(Quat_a.conjugate()*Quat_d).block<3,1>(0,0);

        Eigen::MatrixXd pose_err = (input_data_d.row(0) - input_data_a.row(0));
        pose_err.block(0,0,1,3) = Quat_err.transpose();
        // X_c
        // output_data.row(0) = 100.0*pose_err;
        // output_data.row(0)(0) = para(0)*pose_err(0);
        // output_data.row(0)(1) = para(1)*pose_err(1);
        // output_data.row(0)(2) = para(2)*pose_err(2);
        output_data.row(0)(0) = pose_err(0);
        output_data.row(0)(1) = pose_err(1);
        output_data.row(0)(2) = pose_err(2);
        output_data.row(0)(3) = pose_err(3);
        // std::cout<<"pose_err: "<<std::endl<<pose_err<<std::endl;
        // X_dot_c
        output_data.row(1)(0) = input_data_d.row(1)(0);// + para(0)*pose_err(0) ;//+ para(6)*(input_data_d.row(1)(0) - input_data_a.row(1)(0));
        output_data.row(1)(1) = input_data_d.row(1)(1);// + para(1)*pose_err(1) ;//+ para(7)*(input_data_d.row(1)(1) - input_data_a.row(1)(1));
        output_data.row(1)(2) = input_data_d.row(1)(2);// + para(2)*pose_err(2) ;//+ para(8)*(input_data_d.row(1)(2) - input_data_a.row(1)(2));
        output_data.row(1)(3) = input_data_d.row(1)(3) ;//+ para(3)*pose_err(3);
//        output_data.row(1)(4) = input_data_d.row(1)(4) ;//+ para(4)*pose_err(4);
//        output_data.row(1)(5) = input_data_d.row(1)(5) ;//+ para(5)*pose_err(5);
        // X_ddot_c
        output_data.row(2)(0) = input_data_d.row(2)(0) + para(6)*(input_data_d.row(1)(0) - input_data_a.row(1)(0)) + para(0)*pose_err(0);
        output_data.row(2)(1) = input_data_d.row(2)(1) + para(7)*(input_data_d.row(1)(1) - input_data_a.row(1)(1)) + para(1)*pose_err(1);
        output_data.row(2)(2) = input_data_d.row(2)(2) + para(8)*(input_data_d.row(1)(2) - input_data_a.row(1)(2)) + para(2)*pose_err(2);
        output_data.row(2)(3) = input_data_d.row(2)(3) + para(11)*(input_data_d.row(1)(3) - input_data_a.row(1)(3)) + para(5)*pose_err(3);
//        output_data.row(2)(4) = input_data_d.row(2)(4) + para(10)*(input_data_d.row(1)(4) - input_data_a.row(1)(4)) + para(4)*pose_err(4);
//        output_data.row(2)(5) = input_data_d.row(2)(5) + para(11)*(input_data_d.row(1)(5) - input_data_a.row(1)(5)) + para(5)*pose_err(5);
    }else{
        output_data = input_data_d;
    }

    // F
//    output_data.row(3) = input_data_d.row(3);// + para(0)*(input_data_d.row(3) - input_data_a.row(3));
    
    // #ifdef DEBUG
    // std::cout << "R_a: "  <<std::endl<< R_a <<std::endl;
    // std::cout <<  "R_d: " <<std::endl<< R_d <<std::endl;
    // std::cout <<  "pose_err: " <<std::endl<< pose_err <<std::endl;
    // std::cout <<  "output_data: " <<std::endl<< output_data <<std::endl;
    // #endif
}

void Controller_Lib::none()
{
    output_data = input_data_d;
}

void Controller_Lib::admittance()
{
    output_data = input_data_d;
    
     int num = dim;
     double steptime = dt;
     // init deltx and delt x dot
     if(flag == false)
     {
        alter_v.row(0) =  input_data_a.row(0) - input_data_d.row(0);
        alter_v.row(1) =  input_data_a.row(1) - input_data_d.row(1);
        flag = true;
     }
     Eigen::MatrixXd K;
     Eigen::MatrixXd M;
     Eigen::MatrixXd B;
     K.setZero(num,num);
     M.setZero(num,num);
     B.setZero(num,num);
     for(int i = 0;i<num;i++)
     {
         K(i,i) = para(0);
         M(i,i) = para(1);
         B(i,i) = para(2);
     }
    // define state space
     Eigen::MatrixXd    A_c_left;
     Eigen::MatrixXd    B_c_left;
     Eigen::MatrixXd    C_c_left;

     Eigen::MatrixXd    X_c_left;
     Eigen::MatrixXd    U_c_left;
     Eigen::MatrixXd    Y_c_left;
     A_c_left.setZero(2*num,2*num);
     B_c_left.setZero(2*num,num);
     C_c_left.setZero(num,2*num);
     X_c_left.setZero(2*num,1);
     U_c_left.setZero(num,1);
     Y_c_left.setZero(num,1);
     // M inverse
     Eigen::MatrixXd    Minv;
     Minv.setZero(num,num);
     for (int i=0; i<num; i++)
     {
        Minv(i,i) = 1.0/M(i,i);
     }
      //std::cout<<"K: "<<K<<std::endl;
      //std::cout<<"B: "<<B<<std::endl;
      //std::cout<<"M: "<<M<<std::endl;
     // state space init
     A_c_left.block(0,num,num,num).setIdentity();
     A_c_left.block(num,0,num,num) = -Minv*K;
     A_c_left.block(num,num,num,num) = -Minv*B;

     B_c_left.block(num,0,num,num) = Minv;

     C_c_left.block(0,num,num,num).setIdentity();
  //std::cout<<"A_c_left: "<<A_c_left<<std::endl;
  //std::cout<<"B_c_left: "<<B_c_left<<std::endl;

  // calculate the command velocity
  X_c_left.block(0,0,num,1) = alter_v.row(0).transpose();
  X_c_left.block(num,0,num,1) = alter_v.row(1).transpose();
  U_c_left = input_data_a.row(3).transpose();
  Eigen::MatrixXd I;
  I.setIdentity(2*num,2*num);
  X_c_left = (I+A_c_left * steptime) * X_c_left + B_c_left * steptime * U_c_left;
  //Y_c_left.noalias() = C_c_left * X_c_left;
  alter_v.row(0) = X_c_left.block(0,0,num,1).transpose();
  alter_v.row(1) = X_c_left.block(num,0,num,1).transpose();
  output_data.row(0) = input_data_d.row(0) + X_c_left.block(0,0,num,1).transpose();
  output_data.row(1) = input_data_d.row(1) + X_c_left.block(num,0,num,1).transpose();
  output_data.row(2) = A_c_left * X_c_left + B_c_left * U_c_left;
}


void Controller_Lib::admittance_force_feedback()
{

}

void Controller_Lib::impedance()
{

}

void Controller_Lib::impedance_force_feedback()
{

}

void Controller_Lib::setinput_data(Eigen::MatrixXd input_a, Eigen::MatrixXd input_d, int input_dim, double input_dt)
{
    input_data_a = input_a;
    input_data_d = input_d;
    dim = input_dim;
    dt = input_dt;

    #ifdef DEBUG
    std::cout<<"input_data_a: " <<std::endl<<input_data_a<<std::endl;
    std::cout<<"input_data_d: " <<std::endl<<input_data_d<<std::endl;
    std::cout<<"dim: "<<dim<<std::endl;
    std::cout<<"dt: "<<dt<<std::endl;
    #endif
}

void Controller_Lib::setcontrol_para(Eigen::VectorXd control_para)
{
    para = control_para;
}

void Controller_Lib::getoutput_data(Eigen::MatrixXd &output)
{
    output = output_data;
    #ifdef DEBUG
    std::cout<<"output_data: " <<std::endl<<output_data<<std::endl;
    #endif
}

void Controller_Lib::user_defined()
{

}
