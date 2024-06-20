#pragma once
#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace FSA_CONNECT {
/**
 * @brief this namespace for transmit, but only has udp socket now
 */
namespace Transmit {
/**
 * @brief udp socket receive and send, use udp port 2333 and 2334
 * 2333 for control, 2334 for root config
 */
class UDPSocket {
  public:
    UDPSocket(std::string ip, uint16_t remote_port);
    ~UDPSocket();
    int SendData(const std::string &send_data);
    int SendData(const unsigned char *send_data, uint16_t data_length);
    int SendData(const char *send_data, uint16_t data_length);
    int ReceiveData_rt(std::string &receivedata);
    int ReceiveData_nrt(std::string &receivedata);
    int ReceiveData(unsigned char *rec_ptr);
    int sfd;
    char *ip_ = new char[16];
    struct sockaddr_in ser_addr_;
};
} // namespace Transmit
} // namespace FSA_CONNECT
