#ifndef SENSOR_H
#define SENSOR_H
#include<Eigen/Dense>
/**
 * @brief The sensor_type enum
 * FT_sensor: 6 dims: tau_x tau_y tau_z f_x f_y f_z
 * Imu_sensor: 6*3 dims, multi-sensors' final results: X X_dot X_ddot
 */
enum sensor_type {FT_sensor=1, Imu_sensor, Joint_external_torque_observer};
/**
 * @brief The Sensor class
 */
class Sensor
{
public:
    // construct function
    Sensor();
    ~Sensor();
    // sensor ID: from 1 to ...
    int ID;
    // sensor type
    enum sensor_type type;
    // sensor data
    Eigen::Matrix<double, Eigen::Dynamic,1> _data;
    // locate in link num
    uint link_num;
    // offset
    Eigen::Matrix4d T_offset;

};

#endif // SENSOR_H
