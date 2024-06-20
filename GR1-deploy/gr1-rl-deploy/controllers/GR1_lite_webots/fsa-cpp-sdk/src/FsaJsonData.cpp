#include "FsaJsonData.h"
#include "FsaStatus.h"
namespace FSA_CONNECT {
namespace JsonData {
ordered_json get_root_config_json = {{"method", "GET"}, {"reqTarget", "/"}, {"property", ""}};

ordered_json get_state_json = {{"method", "GET"}, {"reqTarget", "/state"}, {"property", ""}};

ordered_json enable_json = {{"method", "SET"}, {"reqTarget", "/control_word"}, {"property", ""}, {"control_word", 1}};

ordered_json disable_json = {{"method", "SET"}, {"reqTarget", "/control_word"}, {"property", ""}, {"control_word", 1}};

ordered_json set_control_mode_json = {
    {"method", "SET"},
    {"reqTarget", "/config"},
    {"property", ""},
    {"actuator_type", Status::FSAActuatorType::TYPE_60_ACB_120},
    {"actuator_reduction_ratio", 1},
    {"motor_index", 1},
    {"motor_vbus", 1},
    {"motor_direction", 1},
    {"motor_pole_pairs", 1},
    {"motor_max_speed", 1},
    {"encoder_direction", 1},
    {"encoder_resolution", 1},
    {"encoder_phase_offset", 0}};

ordered_json get_control_mode_json = {{"method", "GET"}, {"reqTarget", "/config"}, {"property", ""}};

ordered_json set_pid_params_json = {
    {"method", "SET"},
    {"reqTarget", "/pid_param_imm"},
    {"property", ""},
    {"control_position_kp_imm", 0.0},
    {"control_velocity_kp_imm", 0.0},
    {"control_velocity_ki_imm", 0.0},
    {"control_current_kp_imm", 0.0},
    {"control_current_ki_imm", 0.0}};
ordered_json set_operation_mode_json = {{"method", "SET"}, {"reqTarget", "/mode_of_operation"}, {"mode_of_operation", Status::POSITION_CONTROL}};

ordered_json get_pid_params_json = {{"method", "GET"}, {"reqTarget", "/pid_param_imm"}, {"property", ""}};

ordered_json set_pos_json = {{"method", "SET"}, {"reqTarget", "/position_control"}, {"reply_enable", true}, {"position", 0}, {"velocity_ff", 0}, {"current_ff", 0}};

ordered_json set_vel_json = {{"method", "SET"}, {"reqTarget", "/velocity_control"}, {"reply_enable", true}, {"velocity", 0}, {"current_ff", 0}};

ordered_json set_cur_json = {{"method", "SET"}, {"reqTarget", "/current_control"}, {"reply_enable", true}, {"current", 0}};

ordered_json get_pvc_json = {{"method", "GET"}, {"reqTarget", "/measured"}, {"position", true}, {"velocity", true}, {"current", true}};

} // namespace JsonData
} // namespace FSA_CONNECT