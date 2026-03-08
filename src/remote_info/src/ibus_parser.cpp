#include "remote_info/ibus_parser.h"
#include <ros/ros.h>  // 用于调试打印

namespace remote_info
{

IbusParser::IbusParser()
    : state_(WAIT_LENGTH)
    , index_(0)
    , checksum_calc_(0)
{
}

void IbusParser::reset()
{
    state_ = WAIT_LENGTH;
    index_ = 0;
    checksum_calc_ = 0;
}

bool IbusParser::parseByte(uint8_t byte, std::vector<uint16_t>& out_channels)
{
    switch (state_)
    {
    case WAIT_LENGTH:
        if (byte == 0x20)  // 帧头长度字节
        {
            buffer_[0] = byte;
            index_ = 1;
            checksum_calc_ = byte;
            state_ = WAIT_COMMAND;
        }
        break;

    case WAIT_COMMAND:
        if (byte == 0x40)  // 命令码
        {
            buffer_[1] = byte;
            index_ = 2;
            checksum_calc_ += byte;
            state_ = WAIT_DATA;
        }
        else
        {
            reset();  // 命令码错误，重新同步
        }
        break;

    case WAIT_DATA:
        buffer_[index_] = byte;
        checksum_calc_ += byte;
        index_++;
        
        if (index_ == 32)  // 已接收完32字节
        {
            state_ = WAIT_CHECKSUM;
        }
        break;

    case WAIT_CHECKSUM:
    {
        // 校验和 = 0xFFFF - (前30字节和)
        uint16_t expected_checksum = 0xFFFF - (checksum_calc_ - buffer_[30] - buffer_[31]);
        uint16_t received_checksum = (buffer_[30] | (buffer_[31] << 8));
        
        if (expected_checksum == received_checksum)
        {
            // 校验通过，解析14个通道（小端序）
            out_channels.clear();
            for (int i = 0; i < 14; i++)
            {
                uint16_t ch = buffer_[2 + i*2] | (buffer_[2 + i*2 + 1] << 8);
                out_channels.push_back(ch);
            }
            reset();
            return true;
        }
        else
        {
            ROS_DEBUG_THROTTLE(1.0, "IBUS checksum mismatch: calc=%04x, recv=%04x", 
                               expected_checksum, received_checksum);
            reset();
            return false;
        }
        break;
    }

    default:
        reset();
        break;
    }
    return false;
}

} // namespace remote_info