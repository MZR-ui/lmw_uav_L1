#include "msp_interface/msp_parser.h"
#include "msp_interface/msp_protocol.h"   // 需要 CRC 函数

namespace msp_interface
{

MspV2Parser::MspV2Parser()
    : state_(WAIT_HEADER1)
    , flag_(0)
    , cmd_(0)
    , payload_size_(0)
    , crc_calc_(0)
    , crc_received_(0)
{
}

void MspV2Parser::reset()
{
    state_ = WAIT_HEADER1;
    flag_ = 0;
    cmd_ = 0;
    payload_size_ = 0;
    payload_.clear();
    crc_calc_ = 0;
    crc_received_ = 0;
}

bool MspV2Parser::parseByte(uint8_t byte, std::vector<uint8_t>& out_packet)
{
    switch (state_)
    {
    case WAIT_HEADER1:
        if (byte == '$')
            state_ = WAIT_HEADER2;
        break;

    case WAIT_HEADER2:
        if (byte == 'X')
        {
            state_ = WAIT_FLAG;
            crc_calc_ = 0;
        }
        else
        {
            reset();
        }
        break;

    case WAIT_FLAG:
        flag_ = byte;
        crc_calc_ = crc16_ccitt(crc_calc_, byte);
        state_ = WAIT_CMD1;
        break;

    case WAIT_CMD1:
        cmd_ = byte;
        crc_calc_ = crc16_ccitt(crc_calc_, byte);
        state_ = WAIT_CMD2;
        break;

    case WAIT_CMD2:
        cmd_ |= (static_cast<uint16_t>(byte) << 8);
        crc_calc_ = crc16_ccitt(crc_calc_, byte);
        state_ = WAIT_SIZE1;
        break;

    case WAIT_SIZE1:
        payload_size_ = byte;
        crc_calc_ = crc16_ccitt(crc_calc_, byte);
        state_ = WAIT_SIZE2;
        break;

    case WAIT_SIZE2:
        payload_size_ |= (static_cast<uint16_t>(byte) << 8);
        crc_calc_ = crc16_ccitt(crc_calc_, byte);
        if (payload_size_ == 0)
        {
            state_ = WAIT_CRC1;
        }
        else
        {
            payload_.clear();
            payload_.reserve(payload_size_);
            state_ = WAIT_PAYLOAD;
        }
        break;

    case WAIT_PAYLOAD:
        payload_.push_back(byte);
        crc_calc_ = crc16_ccitt(crc_calc_, byte);
        if (payload_.size() == payload_size_)
            state_ = WAIT_CRC1;
        break;

    case WAIT_CRC1:
        crc_received_ = byte;          // CRC 低字节
        state_ = WAIT_CRC2;
        break;

    case WAIT_CRC2:
        crc_received_ |= (static_cast<uint16_t>(byte) << 8); // CRC 高字节
        if (crc_calc_ == crc_received_)
        {
            out_packet = std::move(payload_);
            reset();
            return true;
        }
        else
        {
            reset();
            return false;
        }
        break;

    default:
        reset();
        break;
    }
    return false;
}

} // namespace msp_interface