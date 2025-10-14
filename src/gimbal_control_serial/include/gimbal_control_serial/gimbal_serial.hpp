#pragma once
#include <serial/serial.h>
#include <string>
#pragma once
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include "gimbal_control_serial/gimbal_protocol.hpp"

class GimbalSerial
{
public:
    GimbalSerial(const std::string& port, int baudrate);
    ~GimbalSerial();

    bool open();
    void close();
    bool isOpen() const;

    bool sendPacket(const gimbal_protocol::Gcu2GbcPkt_t& pkt);
    bool readPacket(gimbal_protocol::Gbc2GcuPkt_t& pkt);

private:
    void readThread();
    void handleReceivedData(const uint8_t* data, size_t len);

private:
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::string port_;
    int baudrate_;
    std::thread read_thread_;
    std::mutex mutex_;
    bool running_ = false;

    std::vector<uint8_t> recv_buffer_;
};
