#pragma once

namespace FSA_CONNECT {
/**
 * @brief this namespace for enum of motor result, include result and error, every function will return result code, please find code here
 */
namespace ResultCode {

enum FSAFunctionResult {
    SUCCESS = 0,
    FAIL = -1,
    RUNNING = 1,
    PREPARE = 2,
    EXECUTE = 3,
    NOT_EXECUTE = 4,
    TIMEOUT = 5,
};

enum FSASocketResult {
    SOCKET_SEND_FAILED = -10001,
    SOCKET_SEND_SIZE_WRONG = -10002,
    SOCKET_RECEIVE_FAILED = -10003,
    SOCKET_RECEIVE_SIZE_WRONG = -10004,

};

enum FSAPowerResult {
    ENABLE_FAILED = -20001,
    DISABLE_FAILED = -20002,

};

enum FSAConfigResult {
    GET_ROOT_SUCCESS = 30001,
    GET_ROOT_FAILED = -30001,
};

} // namespace ResultCode
} // namespace FSA_CONNECT
