#include "Eigen/Dense"
#include "Fsa.h"
#include "ThreadPool.h"
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace FSA_CONNECT {
/**
 * @brief all command in this class ,only need ip of motor
 */
class FSAMutiMotor {

  public:
    // ructor
    FSAMutiMotor(int num) : pool(num){};
    void init(std::vector<string> &ip);
    // FSA &operator=( FSA &otherAIOS);
    /**
     * @fn GetRootConfig
     * @brief get motor attribute ,which saved in motor drives, you can get some root config such as hardware version
     * @details recommend use python sdk
     * @return return result code, find in "FsaResult.h"
     */
    void GetRootConfig();

    /**
     * @fn GetState
     * @brief get motor state
     * @details sate = 0 when normal conditions
     * @return return result code, find in "FsaResult.h"
     */
    void GetState();
    /**
     * @fn Enable
     * @brief servo on
     * @details must enable before control motor
     * @return return result code, find in "FsaResult.h"
     */
    void Enable();
    /**
     * @fn Disable
     * @brief servo off
     * @details use this func best when end program
     * @return return result code, find in "FsaResult.h"
     */
    void Disable();
    // int SetConfig( FSAConfig::FSAMotionConfig &motion_controller_config);

    /**
     * @fn SetControlConfig
     * @brief set motor config
     * @details motor drive has default config, do not use if not necessary
     * @return return result code, find in "FsaResult.h"
     */
    void SetControlConfig(std::vector<FSAConfig::FSAControlConfig> &config);
    /**
     * @fn SetPIDParams
     * @brief set PID paraments
     * @details motor drive has default pid ,if you want use your pid, must use this func every time, because motor deive can not remember your pid
     * @return return result code, find in "FsaResult.h"
     */
    void SetPIDParams(std::vector<FSAConfig::FSAPIDParams> &pidparams);
    /**
     * @fn GetPIDParams
     * @brief read pid params now
     * @details recommend use python sdk
     * @return return result code, find in "FsaResult.h"
     */
    void GetPIDParams(std::vector<FSAConfig::FSAPIDParams> &pidparams);
    /**
     * @fn GetControlConfig
     * @brief read control config now
     * @details recommend use python sdk
     * @return return result code, find in "FsaResult.h"
     */
    void GetControlConfig();
    /**
     * @fn SetPosition
     * @brief send ideal position order
     * @details use in realtime loop
     * @return return result code, find in "FsaResult.h"
     */
    void SetPosition(Eigen::VectorXd &pos, Eigen::VectorXd &vel_ff, Eigen::VectorXd &cur_ff);
    /**
     * @fn SetVelocity
     * @brief send ideal velocity order
     * @details use in realtime loop
     * @return return result code, find in "FsaResult.h"
     */
    void SetVelocity(Eigen::VectorXd &vel, Eigen::VectorXd &cur_ff);
    /**
     * @fn SetCurrent
     * @brief send ideal current order
     * @details use in realtime loop
     * @return return result code, find in "FsaResult.h"
     */
    void SetCurrent(Eigen::VectorXd &cur);
    /**
     * @fn GetPVC
     * @brief read actual position, velocity, current from motor drive
     * @details use in realtime loop
     * @return return result code, find in "FsaResult.h"
     */
    void GetPVC(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &cur);
    /**
     * @fn EnablePosControl
     * @brief set operation of mode : POSITION CONTROL
     * @details must enable this func before use "setpositon"
     * @return return result code, find in "FsaResult.h"
     */
    void EnablePosControl();
    /**
     * @fn EnableVelControl
     * @brief set operation of mode : VELOCITY CONTROL
     * @details must enable this func before use "setvelocity"
     * @return return result code, find in "FsaResult.h"
     */
    void EnableVelControl();
    /**
     * @fn EnableCurControl
     * @brief set operation of mode : CURRENT_CLOSE_LOOP_CONTROL
     * @details must enable this func before use "setcurrent"
     * @return return result code, find in "FsaResult.h"
     */
    void EnableCurControl();
    // std::shared_ptr<Transmit::UDPSocket> pt_udp_socket;
    /**
     * @fn UpdateReceiveData
     * @brief use to update thradpool
     * @details has be used by EnablePosControl
     * @return return result code, find in "FsaResult.h"
     */
    void UpdateReceiveData(int sfd_now, int i);

  private:
    int i_now;
    int sfd_now;
    std::vector<std::shared_ptr<FSA_CONNECT::FSA>> muti_motor;
    ThreadPool pool;
    Eigen::VectorXd pos_set;
    Eigen::VectorXd vel_set;
    Eigen::VectorXd cur_set;
    Eigen::VectorXd pos_get;
    Eigen::VectorXd vel_get;
    Eigen::VectorXd cur_get;
    int motornum;
    std::mutex dataMutex;
    bool enable_pos_control;
    bool enable_vel_control;
    bool enable_cur_control;
    std::string receive;
    json recv_data_json;
    int err_count;
    long long res1;
    long long res;
    // int socketFd_now;
    // std::string ip_now;
    // std::thread updatedata;
};

// void UpdateReceiveData(int socketFd, std::string ip);
} // namespace FSA_CONNECT