

#pragma once

namespace FSA_CONNECT {
/**
 * @brief this namespace for enum of motor status, include control mode, control word, actuator type and so on
 */
namespace Status {

enum FSAFlagState {
    CLEAR = 0,
    SET = 1,
};

/**
 * @enum  FSAModeOfOperation
 * @brief which control mode you want use
 * @details dont use current control if not necessarys
 * @note TRAPEZOIDAL_CONTROL can not use now
 * @attention  only need choose once before realtime control,if you want change control mode, must choose this mode before
 */

enum FSAModeOfOperation {
    NONE = 0,
    POSITION_CONTROL = 1,
    VELOCITY_CONTROL = 3,
    CURRENT_CLOSE_LOOP_CONTROL = 4,
    TRAPEZOIDAL_CONTROL = 5,
};

// enum class FSAInputMode {
//     INPUT_MODE_INACTIVE = 0,
//     INPUT_MODE_PASSTHROUGH = 1,
//     INPUT_MODE_VEL_RAMP = 2,
//     INPUT_MODE_POS_FILTER = 3,
//     INPUT_MODE_TRAP_TRAJ = 5,
//     INPUT_MODE_TORQUE_RAMP = 6,
// };

enum FSAControlWord {
    CALIBRATE_ADC = 1,
    CALIBRATE_MOTOR = 2,
    OPEN_LOOP_CONTORL = 3,
    // CURRENT_CLOSE_LOOP_CONTROL = 4,
    SERVO_OFF = 0X06,
    SERVO_ON = 0X0F,
    MOTION_ABS = 0X103F,
    CLEAR_FAULT = 0X86,

};

enum InternalControlMode {
    mode_none = 0,
    mode_ext_position = 1,
    mode_ext_veloticy = 2,
    mode_ext_current = 3,
    mode_vel_ramp = 4,
    mode_trapezoidal_move = 5,
};

/**
 * @enum  FSAActuatorType
 * @brief all type of our motor
 * @details you don not need choose this because we havev default config
 * @note noting
 * @attention  noting
 */

enum FSAActuatorType {
    TYPE_DEFAULT = 0x0001,
    TYPE_36_ABC_1 = 0x010101,
    TYPE_36_ABC_30 = 0x010102,
    TYPE_36_ACB_1 = 0x010201,
    TYPE_36_ACB_30 = 0x010202,

    TYPE_60_ABC_1 = 0x020101,
    TYPE_60_ABC_120 = 0x020102,
    TYPE_60_ACB_1 = 0x020201,
    TYPE_60_ACB_120 = 0x020202,

    TYPE_80_ABC_1 = 0x030101,
    TYPE_80_ABC_7 = 0x030102,
    TYPE_80_ACB_1 = 0x030201,
    TYPE_80_ACB_7 = 0x030202,

    TYPE_130_ABC_1 = 0x040101,
    TYPE_130_ABC_7 = 0x040102,
    TYPE_130_ACB_1 = 0x040201,
    TYPE_130_ACB_7 = 0x040202,
};

enum FSAMotorDirection {
    ABC = 1,
    ACB = -1,
};
} // namespace Status
} // namespace FSA_CONNECT