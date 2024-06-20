#pragma once

#include <string>
namespace FSA_CONNECT {
/**
 * @brief this namespace for config of actuator, include pid config and control config.
 * both of these struct has default config in every actuator drive.
 */
namespace FSAConfig {

/**
 * @brief this struct include position kp, velocity kp, ki, current kp,ki and limit.
 */
struct FSAPIDParams {
    double control_position_kp;
    double control_velocity_kp;
    double control_velocity_ki;
    double control_current_kp;
    double control_current_ki;
    double control_position_output_max;
    double control_position_output_min;
    double control_velocity_output_max;
    double control_velocity_output_min;
    double control_current_output_max;
    double control_current_output_min;
};

/**
 * @brief this struct you do not need config.
 */
struct FSAControlConfig {
    int actuator_type;
    int actuator_reduction_ratio;
    int motor_index;
    double motor_vbus;
    int motor_direction;
    int motor_pole_pairs;
    double motor_max_speed;
    int encoder_direction;
    double encoder_resolution;
    double encoder_phase_offset;
};

// struct FSARootConfig {
//     std::string serial_number;
//     std::string connect_mode;
//     std::string name;
//     std::string model;
//     // double gear_ratio;
//     int vbus_voltage;
//     int motor_temp_m1;
//     int inverter_temp_m1;
//     std::vector<std::string> hw_version;
//     std::vector<std::string> fw_version;
// };

} // namespace FSAConfig
} // namespace FSA_CONNECT