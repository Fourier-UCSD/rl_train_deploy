#include "FsaSocket.h"
#include "FsaResult.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>

#define MAX_SEND_SIZE 1024
#define MAX_RECV_SIZE 1024

using namespace FSA_CONNECT::Transmit;

UDPSocket::UDPSocket(std::string ip, uint16_t remote_port) {

    // char* ip_local;
    // std::cout<<"[UDPSocket] construct start"<<std::endl;
    // std::cout<< ip <<"size:" << ip.size() << std::endl;
    strcpy(ip_, ip.c_str());
    // std::cout<<"[UDPSocket] strcpy"<<std::endl;
    ser_addr_.sin_family = AF_INET;
    ser_addr_.sin_addr.s_addr = inet_addr(ip_);
    ser_addr_.sin_port = htons(remote_port);
    sfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct timeval timeout;
    timeout.tv_sec = 1 / 1000;
    timeout.tv_usec = (1 % 1000) * 1000;
    setsockopt(sfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // connect(sd, )
}
UDPSocket::~UDPSocket() { close(sfd); }

int UDPSocket::SendData(const std::string &send_data) {
    char send_data_[MAX_SEND_SIZE] = {'\0'};
    int ret;
    int send_data_length = send_data.size();
    std::strcpy(send_data_, send_data.c_str());

    ret = sendto(sfd, send_data_, send_data_length, 0, (struct sockaddr *)&ser_addr_, sizeof(ser_addr_));
    if (ret < 0) {
        return FSA_CONNECT::ResultCode::SOCKET_SEND_FAILED;
    }
    if (ret != send_data.size()) {
        return FSA_CONNECT::ResultCode::SOCKET_SEND_SIZE_WRONG;
    }
    return FSA_CONNECT::ResultCode::SUCCESS;
}

int UDPSocket::SendData(const unsigned char *send_data, uint16_t data_length) {
    int ret;
    // std::cout << "data_length:" << data_length << std::endl;
    ret = sendto(sfd, send_data, data_length, 0, (struct sockaddr *)&ser_addr_, sizeof(ser_addr_));
    if (ret < 0) {
        return FSA_CONNECT::ResultCode::SOCKET_SEND_FAILED;
    }
    if (ret != data_length) {
        return FSA_CONNECT::ResultCode::SOCKET_SEND_SIZE_WRONG;
    }

    return FSA_CONNECT::ResultCode::SUCCESS;
}

int UDPSocket::SendData(const char *send_data, uint16_t data_length) {
    int ret;
    // std::cout << "data_length:" << data_length << std::endl;
    ret = sendto(sfd, send_data, data_length, 0, (struct sockaddr *)&ser_addr_, sizeof(ser_addr_));
    if (ret < 0) {
        return FSA_CONNECT::ResultCode::SOCKET_SEND_FAILED;
    }
    if (ret != data_length) {
        return FSA_CONNECT::ResultCode::SOCKET_SEND_SIZE_WRONG;
    }
    return FSA_CONNECT::ResultCode::SUCCESS;
}

int UDPSocket::ReceiveData_nrt(std::string &receivedata) {
    char recv_data_[MAX_RECV_SIZE] = {'\0'};
    int ret;
    socklen_t addrlen = sizeof(ser_addr_);
    ret = recvfrom(sfd, recv_data_, sizeof(recv_data_), 0, (struct sockaddr *)&ser_addr_, &addrlen);
    // receive nothing
    if (ret < 0) {
        receivedata = "";
        return FSA_CONNECT::ResultCode::SOCKET_RECEIVE_FAILED;
    }
    receivedata = std::string(recv_data_);
    return FSA_CONNECT::ResultCode::SUCCESS;
}

int UDPSocket::ReceiveData_rt(std::string &receivedata) {
    char recv_data_[MAX_RECV_SIZE] = {'\0'};
    int ret;
    socklen_t addrlen = sizeof(ser_addr_);
    ret = recvfrom(sfd, recv_data_, sizeof(recv_data_), MSG_DONTWAIT, (struct sockaddr *)&ser_addr_, &addrlen);

    // receive nothing
    if (ret <= 0) {
        receivedata = "";
        return FSA_CONNECT::ResultCode::SOCKET_RECEIVE_FAILED;
    }
    receivedata = std::string(recv_data_);
    return FSA_CONNECT::ResultCode::SUCCESS;
}

int UDPSocket::ReceiveData(unsigned char *rec_ptr) {
    unsigned char recv_data[17] = {'\0'};
    int ret;
    socklen_t addrlen = sizeof(ser_addr_);
    ret = recvfrom(sfd, recv_data, sizeof(recv_data), MSG_DONTWAIT, (struct sockaddr *)&ser_addr_, &addrlen);

    // error or receive nothing
    if (ret <= 0) {
        return FSA_CONNECT::ResultCode::SOCKET_RECEIVE_FAILED;
    }

    // receive something
    else {
        if (rec_ptr != nullptr) {
            // std::cout << "recv_data_size:" << ret << std::endl;
            memcpy(rec_ptr, recv_data, ret);
            return 1;
        }
    }
    return FSA_CONNECT::ResultCode::SUCCESS;
}
