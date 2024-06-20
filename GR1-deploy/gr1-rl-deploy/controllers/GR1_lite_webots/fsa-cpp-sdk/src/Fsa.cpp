#include "Fsa.h"
#include <fstream>

void FSA_CONNECT::FSA::init(const string &ip) {
    ip_ = ip;
    ctrl_udp_socket = std::make_shared<Transmit::UDPSocket>(ip, 2333);
    root_udp_socket = std::make_shared<Transmit::UDPSocket>(ip, 2334);
    // pt_udp_socket = std::make_shared<Transmit::UDPSocket>(ip, 10000);
};

int FSA_CONNECT::FSA::GetRootConfig() {
    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::FSAConfig;

    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (get_root_config) {

        case 0: //
            begin = std::chrono::steady_clock::now();
            std::cout << get_root_config_json.dump() << std::endl;
            ret = ctrl_udp_socket->SendData(get_root_config_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << " GET ROOT CONFIG SEND ERROR! ERROR CODE: " << ret << std::endl;
                return ret;
            }
            // data send succeed
            get_root_config = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_nrt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", GET ROOT CONFIG RECEIVE ERROR! ERROR CODE: " << ret << std::endl;
                get_root_config = 0;
                return ret;
            }

            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);

                receive_state = recv_data_json.at("status");

                if (receive_state.compare("OK")) {
                    get_root_config = 0;
                    std::cout << "MOTOR: " << ip_ << ", GET ROOT CONFIG FAILED" << std::endl;
                    return GET_ROOT_FAILED;
                }

                get_root_config = 0;
                std::cout << "MOTOR: " << ip_ << ", GET ROOT CONFIG SUCCESS! " << std::endl;
                std::cout << recv_data_str << std::endl;
                return SUCCESS;
            }

            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                get_root_config = 0;
                std::cout << "MOTOR: " << ip_ << ", GET ROOT CONFIG TIMEOUT" << std::endl;

                return FSA_CONNECT::ResultCode::TIMEOUT;
            }
            break;
        default:
            get_root_config = 0;
            break;
        };
    }

    return NOT_EXECUTE;
}

int FSA_CONNECT::FSA::GetState() {
    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::FSAConfig;

    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (get_state) {

        case 0: //
            begin = std::chrono::steady_clock::now();
            std::cout << get_state_json.dump() << std::endl;
            ret = root_udp_socket->SendData(get_state_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", GET STATE SEND ERROR! ERROR CODE: " << ret << std::endl;
                return ret;
            }
            // data send succeed
            get_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = root_udp_socket->ReceiveData_nrt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", GET STATE RECEIVE ERROR! ERROR CODE: " << ret << std::endl;
                get_state = 0;
                return ret;
            }

            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);

                receive_state = recv_data_json.at("status");
                // std::cout << "json: " << receive_state << std::endl;
                if (receive_state.compare("OK")) {
                    get_state = 0;
                    return GET_ROOT_FAILED;
                }

                get_state = 0;
                std::cout << "MOTOR: " << ip_ << ", GET ROOT CONFIG SUCCESS! " << std::endl;

                std::cout << recv_data_str << std::endl;
                return SUCCESS;
            }

            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                get_state = 0;
                return FSA_CONNECT::ResultCode::TIMEOUT;
            }
            break;
        default:
            get_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
}

int FSA_CONNECT::FSA::Enable() {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (poweron_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            enable_json["control_word"] = FSAControlWord::SERVO_ON;
            ret = ctrl_udp_socket->SendData(enable_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;
                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            poweron_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_nrt(recv_data_str);

            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;
                poweron_state = 0;
                return ret;
            }

            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    poweron_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", ENABLE FAILED! " << std::endl;

                    return ENABLE_FAILED;
                }
                is_enabled = true;
                poweron_state = 0;
                std::cout << "MOTOR: " << ip_ << ", ENABLE SUCCESS! " << std::endl;
                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                poweron_state = 0;
                std::cout << "MOTOR: " << ip_ << ", ENABLE TIMEOUT! " << std::endl;
                return TIMEOUT;
            }
            break;

        default:
            poweron_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::Disable() {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (poweroff_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            disable_json["control_word"] = FSAControlWord::SERVO_OFF;
            ret = ctrl_udp_socket->SendData(disable_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            poweroff_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_nrt(recv_data_str);

            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;
                poweroff_state = 0;
                return ret;
            }

            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    poweroff_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", DISABLE FAILED! " << std::endl;

                    return DISABLE_FAILED;
                }
                poweroff_state = 0;
                std::cout << "MOTOR: " << ip_ << ", DISABLE SUCCESS! " << std::endl;
                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                poweroff_state = 0;
                std::cout << "MOTOR: " << ip_ << ", DISABLE TIMEOUT! " << std::endl;

                return TIMEOUT;
            }
            break;

        default:
            poweroff_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::SetControlConfig(const FSAConfig::FSAControlConfig &config) {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (set_ctrlcfg_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            set_control_mode_json["actuator_type"] = config.actuator_type;
            set_control_mode_json["actuator_reduction_ratio"] = config.actuator_reduction_ratio;
            set_control_mode_json["motor_index"] = config.motor_index;
            set_control_mode_json["motor_vbus"] = config.motor_vbus;
            set_control_mode_json["motor_direction"] = config.motor_direction;
            set_control_mode_json["motor_pole_pairs"] = config.motor_pole_pairs;
            set_control_mode_json["motor_max_speed"] = config.motor_max_speed;
            set_control_mode_json["encoder_direction"] = config.encoder_direction;
            set_control_mode_json["encoder_resolution"] = config.encoder_resolution;
            set_control_mode_json["encoder_phase_offset"] = config.encoder_phase_offset;

            ret = ctrl_udp_socket->SendData(set_control_mode_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;
                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            set_ctrlcfg_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_nrt(recv_data_str);

            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;

                set_ctrlcfg_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    set_ctrlcfg_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", SET CONTROL CONFIG FAILED! " << std::endl;

                    return DISABLE_FAILED;
                }
                set_ctrlcfg_state = 0;
                std::cout << "MOTOR: " << ip_ << ",  SET CONTROL CONFIG SUCCESS! " << std::endl;

                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                set_ctrlcfg_state = 0;
                std::cout << "MOTOR: " << ip_ << ", SET CONTROL CONFIG TIMEOUT! " << std::endl;

                return TIMEOUT;
            }
            break;

        default:
            set_ctrlcfg_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::GetControlConfig() {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (get_ctrlcfg_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();

            ret = ctrl_udp_socket->SendData(get_control_mode_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            get_ctrlcfg_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_nrt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;

                get_ctrlcfg_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    get_ctrlcfg_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", GET CONTROL CONFIG FAILED! " << std::endl;

                    return DISABLE_FAILED;
                }
                get_ctrlcfg_state = 0;
                std::cout << "MOTOR: " << ip_ << ",  GET CONTROL CONFIG SUCCESS! " << std::endl;

                std::cout << recv_data_str << std::endl;
                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                get_ctrlcfg_state = 0;
                std::cout << "MOTOR: " << ip_ << ", GET CONTROL CONFIG TIMEOUT! " << std::endl;

                return TIMEOUT;
            }
            break;

        default:
            get_ctrlcfg_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::SetPIDParams(FSAConfig::FSAPIDParams &pidparams) {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (set_pid_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            set_pid_params_json["control_position_kp_imm"] = pidparams.control_position_kp;
            set_pid_params_json["control_velocity_kp_imm"] = pidparams.control_velocity_kp;
            set_pid_params_json["control_velocity_ki_imm"] = pidparams.control_velocity_ki;
            set_pid_params_json["control_current_kp_imm"] = pidparams.control_current_kp;
            set_pid_params_json["control_current_ki_imm"] = pidparams.control_current_ki;
            // set_pid_params_json["control_position_output_max"] = pidparams.control_position_output_max;
            // set_pid_params_json["control_position_output_min"] = pidparams.control_position_output_min;
            // set_pid_params_json["control_velocity_output_max"] = pidparams.control_velocity_output_max;
            // set_pid_params_json["control_velocity_output_min"] = pidparams.control_velocity_output_min;
            // set_pid_params_json["control_current_output_max"] = pidparams.control_current_output_max;
            // set_pid_params_json["control_current_output_min"] = pidparams.control_current_output_min;

            ret = ctrl_udp_socket->SendData(set_pid_params_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            set_pid_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_nrt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;

                set_pid_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    set_pid_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", SET PID PARAMS FAILED! " << std::endl;

                    return DISABLE_FAILED;
                }
                set_pid_state = 0;
                std::cout << "MOTOR: " << ip_ << ",  SET PID PARAMS SUCCESS! " << std::endl;

                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                set_pid_state = 0;
                std::cout << "MOTOR: " << ip_ << ", SET PID PARAMS TIMEOUT! " << std::endl;

                return TIMEOUT;
            }
            break;

        default:
            set_pid_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::GetPIDParams(FSAConfig::FSAPIDParams &pidparams) {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (get_pid_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            ret = ctrl_udp_socket->SendData(get_pid_params_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            get_pid_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_nrt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;

                get_pid_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    get_pid_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", GET PID PARAMS FAILED! " << std::endl;

                    return DISABLE_FAILED;
                }
                get_pid_state = 0;
                pidparams.control_position_kp = recv_data_json.at("control_position_kp_imm");
                pidparams.control_velocity_kp = recv_data_json.at("control_velocity_kp_imm");
                pidparams.control_velocity_ki = recv_data_json.at("control_velocity_ki_imm");
                pidparams.control_current_kp = recv_data_json.at("control_current_kp_imm");
                pidparams.control_current_ki = recv_data_json.at("control_current_ki_imm");

                // pidparams.control_position_output_max = recv_data_json.at("control_position_output_max");
                // pidparams.control_position_output_min = recv_data_json.at("control_position_output_min");
                // pidparams.control_velocity_output_max = recv_data_json.at("control_velocity_output_max");
                // pidparams.control_velocity_output_min = recv_data_json.at("control_velocity_output_min");
                // pidparams.control_current_output_max = recv_data_json.at("control_current_output_max");
                // pidparams.control_current_output_min = recv_data_json.at("control_current_output_min");

                std::cout << "MOTOR: " << ip_ << ",  GET PID PARAMS SUCCESS! " << std::endl;

                std::cout << recv_data_str << std::endl;
                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                get_pid_state = 0;
                std::cout << "MOTOR: " << ip_ << ", SET PID PARAMS TIMEOUT! " << std::endl;

                return TIMEOUT;
            }
            break;

        default:
            get_pid_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::EnablePosControl() {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (control_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            set_operation_mode_json["mode_of_operation"] = Status::POSITION_CONTROL;
            ret = ctrl_udp_socket->SendData(set_operation_mode_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            control_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_nrt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;

                control_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    control_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", ENABLE POSITION CONTROL FAILED! " << std::endl;

                    return DISABLE_FAILED;
                }
                control_state = 0;
                std::cout << "MOTOR: " << ip_ << ",  ENABLE POSITION CONTROL SUCCESS! " << std::endl;

                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                control_state = 0;
                std::cout << "MOTOR: " << ip_ << ", ENABLE POSITION CONTROL TIMEOUT! " << std::endl;

                return TIMEOUT;
            }
            break;

        default:
            control_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::EnableVelControl() {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (control_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            set_operation_mode_json["mode_of_operation"] = Status::VELOCITY_CONTROL;
            ret = ctrl_udp_socket->SendData(set_operation_mode_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            control_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_nrt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;

                control_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    control_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", ENABLE VELOCITY CONTROL FAILED! " << std::endl;

                    return DISABLE_FAILED;
                }
                control_state = 0;
                std::cout << "MOTOR: " << ip_ << ",  ENABLE VELOCITY CONTROL SUCCESS! " << std::endl;

                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                control_state = 0;
                std::cout << "MOTOR: " << ip_ << ", ENABLE VELOCITY CONTROL TIMEOUT! " << std::endl;

                return TIMEOUT;
            }
            break;

        default:
            control_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::EnableCurControl() {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (control_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            set_operation_mode_json["mode_of_operation"] = Status::CURRENT_CLOSE_LOOP_CONTROL;
            ret = ctrl_udp_socket->SendData(set_operation_mode_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            control_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_nrt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;

                control_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    control_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", ENABLE CURRENT CONTROL FAILED! " << std::endl;

                    return DISABLE_FAILED;
                }
                control_state = 0;
                std::cout << "MOTOR: " << ip_ << ",  ENABLE CURRENT CONTROL SUCCESS! " << std::endl;

                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                control_state = 0;
                std::cout << "MOTOR: " << ip_ << ", ENABLE CURRENT CONTROL TIMEOUT! " << std::endl;

                return TIMEOUT;
            }
            break;

        default:
            control_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::SetPosition(const double &pos, const double &vel_ff, const double &cur_ff) {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (set_pos_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            set_pos_json["position"] = pos;
            set_pos_json["velocity_ff"] = vel_ff;
            set_pos_json["current_ff"] = cur_ff;

            ret = ctrl_udp_socket->SendData(set_pos_json.dump());

            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            set_pos_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_rt(recv_data_str);
            std::cout << recv_data_str << std::endl;
            if (ret < 0) {
                std::cout << recv_data_str << std::endl;
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;
                ;
                set_pos_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);

                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    set_pos_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", SET POSITION FAILED! " << std::endl;
                    ;

                    return DISABLE_FAILED;
                }
                set_pos_state = 0;
                // std::cout<<"MOTOR: "<<ip_<<",  SET POS SUCCESS! "<<ip_<<std::endl;;
                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                set_pos_state = 0;
                std::cout << "MOTOR: " << ip_ << ", SET POSITION TIMEOUT! " << std::endl;
                ;
                return TIMEOUT;
            }
            break;

        default:
            set_pos_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::SetVelocity(const double &vel, const double &cur_ff) {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (set_vel_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            set_vel_json["velocity"] = vel;
            set_vel_json["current_ff"] = cur_ff;

            ret = ctrl_udp_socket->SendData(set_vel_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            set_vel_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_rt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;
                ;
                set_vel_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    set_vel_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", SET VELOCITY FAILED! " << std::endl;
                    ;

                    return DISABLE_FAILED;
                }
                set_vel_state = 0;
                // std::cout<<"MOTOR: "<<ip_<<",  SET POS SUCCESS! "<<ip_<<std::endl;;
                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                set_vel_state = 0;
                std::cout << "MOTOR: " << ip_ << ", SET VELOCITY TIMEOUT! " << std::endl;
                ;
                return TIMEOUT;
            }
            break;

        default:
            set_vel_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::SetCurrent(const double &cur) {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (set_cur_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();

            set_cur_json["current"] = cur;

            ret = ctrl_udp_socket->SendData(set_cur_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            set_cur_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_rt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;
                ;
                set_cur_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    set_cur_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", SET CURRENT FAILED! " << std::endl;
                    ;

                    return DISABLE_FAILED;
                }
                set_cur_state = 0;
                // std::cout<<"MOTOR: "<<ip_<<",  SET POS SUCCESS! "<<ip_<<std::endl;;
                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                set_cur_state = 0;
                std::cout << "MOTOR: " << ip_ << ", SET CURRENT TIMEOUT! " << std::endl;
                ;
                return TIMEOUT;
            }
            break;

        default:
            set_cur_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};

int FSA_CONNECT::FSA::GetPVC(double &pos, double &vel, double &cur) {

    using namespace FSA_CONNECT::JsonData;
    using namespace FSA_CONNECT::ResultCode;
    using namespace FSA_CONNECT::Status;
    int ret;
    std::string recv_data_str;
    json recv_data_json;
    std::string receive_state;
    while (1) {
        switch (get_pvc_state) {
        case 0: // enable
            begin = std::chrono::steady_clock::now();
            ret = ctrl_udp_socket->SendData(get_pvc_json.dump());
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET SEND FAILED! ERROR CODE: " << ret << std::endl;

                return ret;
            }
            // data send succeed
            // clock_gettime(CLOCK_MONOTONIC,&start_udp_socket_time);
            get_pvc_state = 1;
            break;

        case 1: // wait for feedback
            // receive error
            ret = ctrl_udp_socket->ReceiveData_rt(recv_data_str);
            if (ret < 0) {
                std::cout << "MOTOR: " << ip_ << ", UDP SOCKET RECEIVE FAILED! ERROR CODE: " << ret << std::endl;

                get_pvc_state = 0;
                return ret;
            }
            // receive something
            if (!recv_data_str.empty()) {
                recv_data_json = json::parse(recv_data_str);
                receive_state = recv_data_json.at("status");
                if (receive_state.compare("OK")) {
                    get_pvc_state = 0;
                    std::cout << "MOTOR: " << ip_ << ", SET CURRENT FAILED! " << std::endl;

                    return DISABLE_FAILED;
                }
                get_pvc_state = 0;
                pos = recv_data_json.at("position");
                vel = recv_data_json.at("velocity");
                cur = recv_data_json.at("current");
                // std::cout<<"MOTOR: "<<ip_<<",  SET POS SUCCESS! "<<ip_<<std::endl;;
                return SUCCESS;
            }

            // clock_gettime(CLOCK_MONOTONIC,&now_time);
            end = std::chrono::steady_clock::now();
            // time out
            int_ms = chrono::duration_cast<chrono::milliseconds>(end - begin);
            if (int_ms.count() > 3000) {
                get_pvc_state = 0;
                std::cout << "MOTOR: " << ip_ << ", SET CURRENT TIMEOUT! " << std::endl;

                return TIMEOUT;
            }
            break;

        default:
            get_pvc_state = 0;
            break;
        };
    }

    return NOT_EXECUTE;
};
