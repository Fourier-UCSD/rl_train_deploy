#ifndef CONTROLLER_LIB_H
#define CONTROLLER_LIB_H
#include <Eigen/Dense>
/**
 * @brief The basic_controller enum: some basic controllers
 */
enum basic_controller {PID_position=1,
                       PID_force,
                       PID_orient,
                       PositionController,
                       PID,
                       Admittance,
                       Impedance,
                       Admittance_force_feeedback,
                       Impedance_force_feedback,
                       Userdefined,
                       None};
/**
 * @brief The Controller_Lib class: the basic controller lib
 */
class Controller_Lib
{
public:
    // construct function
    Controller_Lib();
    virtual ~Controller_Lib();
    // basic controller
    virtual void user_defined();
    /**
     * @brief positioncontroller
     * para: k_P , k_D;
     * inputdata_a: x_a; x_dot_a; x_ddot_a; F_a
     * inputdate_d: x_d; x_dot_d; x_ddot_d; F_d
     * alter_v(): none
     * outputdata: x; x_dot; x_ddot; F
     */
    void positioncontroller();
    /**
     * @brief PID
     * para: k_P , k_I , k_D;
     * inputdata_a: x_a; x_dot_a; x_ddot_a; F_a
     * inputdate_d: x_d; x_dot_d; x_ddot_d; F_d
     * alter_v():
     * outputdata: x; x_dot; x_ddot; F
     *
     */
    void pid();
    /**
     * @brief admittance
     * para: K , M , B;
     * inputdata_a: x_a; x_dot_a; x_ddot_a; F_a
     * inputdate_d: x_d; x_dot_d; x_ddot_d; F_d
     * alter_v():  (x_a - x_d) *dim;
     *            (x_dot_a - x_dot_d) *dim;
     * outputdata: x; x_dot; x_ddot; F
     */
    void admittance();

    void impedance();
    void admittance_force_feedback();
    void impedance_force_feedback();
    void pid_position();
    void pid_force();
    void pid_orient();

    void none();
    //void mpc();
    // run controller
    void controller_run();

    // input and output
    void setinput_data(Eigen::MatrixXd input_a, Eigen::MatrixXd input_d,int input_dim, double input_dt);
    void setcontrol_para(Eigen::VectorXd control_para);
    void getoutput_data(Eigen::MatrixXd & output);
public:
    // controller type
    enum basic_controller controller_type;
    // dim
    int dim;
    // input data
    Eigen::MatrixXd input_data_a;
    Eigen::MatrixXd input_data_d;
    // output data
    Eigen::MatrixXd output_data;
    // controller parameters such as kp ki kd,
    Eigen::Matrix<double, Eigen::Dynamic,1> para;
    // Alternate variable
    Eigen::MatrixXd alter_v;
    // init mark
    bool flag;
    // cycle time
    double dt;
};

#endif // CONTROLLER_LIB_H
