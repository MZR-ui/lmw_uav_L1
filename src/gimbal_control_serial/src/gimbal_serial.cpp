#include "gimbal_control_serial/gimbal_serial.hpp"
#include <iostream>
#include <iomanip>

using namespace gimbal_protocol;

GimbalSerial::GimbalSerial(const std::string& port, int baudrate)
    : serial_(io_), port_(port), baudrate_(baudrate)
{
}

GimbalSerial::~GimbalSerial()
{
    close();
}

bool GimbalSerial::open()
{
    try
    {
        serial_.open(port_);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        running_ = true;
        read_thread_ = std::thread(&GimbalSerial::readThread, this);

        ROS_INFO("Serial port %s opened at %d baud", port_.c_str(), baudrate_);
        return true;
    }
    catch (std::exception& e)
    {
        ROS_ERROR("Failed to open serial port %s : %s", port_.c_str(), e.what());
        return false;
    }
}

// 请把下面函数替换你现有的 readPacket 实现
bool GimbalSerial::readPacket(Gbc2GcuPkt_t& pkt)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 必须至少有帧头 + 最小长度
    size_t pkt_size = sizeof(Gbc2GcuPkt_t);
    if (recv_buffer_.size() < 2) return false;

    // 从缓冲区查找帧头 0xB5 0x9A
    size_t i = 0;
    while (i + 1 < recv_buffer_.size()) {
        if (recv_buffer_[i] == 0xB5 && recv_buffer_[i+1] == 0x9A) {
            // 如果从该位置到末尾的字节不足以构成完整包，等待更多字节
            if (recv_buffer_.size() - i < pkt_size) {
                return false;
            }

            // 有完整长度，复制到 pkt
            memcpy(&pkt, &recv_buffer_[i], pkt_size);

            // 计算并校验 CRC（包尾最后 2 字节）
            uint16_t crc_calc = gimbal_protocol::CalculateCrc16(reinterpret_cast<uint8_t*>(&pkt), pkt_size - 2);
            uint16_t crc_recv = static_cast<uint16_t>(pkt.crc[0]) | (static_cast<uint16_t>(pkt.crc[1]) << 8);

            if (crc_calc == crc_recv) {
                // 成功解析：移除缓冲区中到这个包结束的字节
                recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + i + pkt_size);
                return true;
            } else {
                // CRC 错误：跳过这个头（即丢掉这个 0xB5 ）并继续尝试下一个可能的头
                ROS_WARN("GimbalSerial: CRC mismatch in received packet, skipping this header");
                i += 1; // move forward one byte (skip current 0xB5)
                continue;
            }
        } else {
            ++i;
        }
    }

    // 如果走到这里，说明没找到完整的帧头+完整包
    // 若缓冲区过大但没有有效头，可以清理前面的垃圾以免无限增长
    const size_t max_buffer_allowed = 4096;
    if (recv_buffer_.size() > max_buffer_allowed) {
        // 清理掉无用前缀，保留尾部以免损失可能的头
        recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.end() - 1024);
        ROS_WARN("GimbalSerial: recv_buffer_ too large, trimming.");
    }

    return false;
}

void GimbalSerial::close()
{
    running_ = false;
    if (serial_.is_open())
        serial_.cancel(), serial_.close();
    if (read_thread_.joinable())
        read_thread_.join();
}


bool GimbalSerial::isOpen() const
{
    return serial_.is_open();
}

bool GimbalSerial::sendPacket(const Gcu2GbcPkt_t& pkt)
{
    if (!isOpen()) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
    size_t len = sizeof(pkt);
    boost::asio::write(serial_, boost::asio::buffer(data, len));
    return true;
}

void GimbalSerial::readThread()
{
    uint8_t buf[256];
    while (running_)
    {
        try
        {
            size_t n = serial_.read_some(boost::asio::buffer(buf));
            handleReceivedData(buf, n);
        }
        catch (std::exception& e)
        {
            ROS_WARN("Serial read error: %s", e.what());
        }
    }
}

void GimbalSerial::handleReceivedData(const uint8_t* data, size_t len)
{
    recv_buffer_.insert(recv_buffer_.end(), data, data + len);

    // 查找帧头 0xB5 0x9A
    while (recv_buffer_.size() >= sizeof(Gbc2GcuPkt_t))
    {
        //auto it = std::search(recv_buffer_.begin(), recv_buffer_.end(), {0xB5, 0x9A});
        const uint8_t header[] = {0xB5, 0x9A};
        auto it = std::search(recv_buffer_.begin(), recv_buffer_.end(),
                              std::begin(header), std::end(header));
        if (it == recv_buffer_.end()) { recv_buffer_.clear(); break; }

        size_t idx = std::distance(recv_buffer_.begin(), it);
        if (recv_buffer_.size() - idx < sizeof(Gbc2GcuPkt_t)) break;

        Gbc2GcuPkt_t pkt;
        memcpy(&pkt, &recv_buffer_[idx], sizeof(pkt));

        // CRC 校验
        uint16_t crc = gimbal_protocol::CalculateCrc16((uint8_t*)&pkt, sizeof(pkt) - 2);
        uint16_t crc_recv = pkt.crc[0] | (pkt.crc[1] << 8);

        if (crc == crc_recv)
        {
            ROS_INFO("Recv Gimbal: roll=%.2f, pitch=%.2f, yaw=%.2f",
                     pkt.cam_angle[0] * 0.01, pkt.cam_angle[1] * 0.01, pkt.cam_angle[2] * 0.01);
        }
        else
        {
            ROS_WARN("CRC mismatch");
        }

        recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + idx + sizeof(Gbc2GcuPkt_t));
    }
}

bool GimbalSerial::sendAndWaitReply(const Gcu2GbcPkt_t& tx_pkt,
                          Gbc2GcuPkt_t& rx_pkt,
                          int timeout_ms)
{
    if (!sendPacket(tx_pkt)) {
        ROS_WARN("GimbalSerial: failed to send packet");
        return false;
    }

    auto start = std::chrono::steady_clock::now();
    std::vector<uint8_t> recv_buf;
    recv_buf.reserve(256);

    while (true) {
        // ---------- 超时判断 ----------
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms) {
            ROS_WARN("GimbalSerial: response timeout");
            return false;
        }

        // ---------- 检查可读字节 ----------
        int bytes_available = 0;
        int fd = serial_.lowest_layer().native_handle();
        if (ioctl(fd, FIONREAD, &bytes_available) < 0) {
            ROS_ERROR("GimbalSerial: ioctl(FIONREAD) failed");
            return false;
        }

        if (bytes_available > 0) {
            std::vector<uint8_t> tmp(bytes_available);
            boost::system::error_code ec;

            size_t r = serial_.read_some(boost::asio::buffer(tmp), ec);
            if (ec) {
                ROS_ERROR("GimbalSerial: read_some error: %s", ec.message().c_str());
                return false;
            }
            recv_buf.insert(recv_buf.end(), tmp.begin(), tmp.begin() + r);
        }

        // ---------- 查找帧头并校验 ----------
        for (size_t i = 0; i + 1 < recv_buf.size(); ++i) {
            if (recv_buf[i] == 0xB5 && recv_buf[i + 1] == 0x9A) {
                size_t remain = recv_buf.size() - i;
                if (remain >= sizeof(Gbc2GcuPkt_t)) {
                    memcpy(&rx_pkt, &recv_buf[i], sizeof(Gbc2GcuPkt_t));

                    uint16_t crc_calc = CalculateCrc16(reinterpret_cast<uint8_t*>(&rx_pkt),
                                                       sizeof(Gbc2GcuPkt_t) - 2);
                    uint16_t crc_recv = rx_pkt.crc[0] | (rx_pkt.crc[1] << 8);

                    // --- 打印接收到的数据 ---
                    std::ostringstream oss;
                    oss << "Recv data: ";
                    for (size_t j = 0; j < sizeof(Gbc2GcuPkt_t); ++j) {
                        oss << std::hex << std::setw(2) << std::setfill('0')
                            << static_cast<int>(reinterpret_cast<uint8_t*>(&rx_pkt)[j]) << " ";
                    }
                    ROS_INFO("%s", oss.str().c_str());

                    // --- 打印 CRC ---
                    ROS_INFO("CRC calc=0x%04X, CRC recv=0x%04X", crc_calc, crc_recv);
                    
                    if (crc_calc == crc_recv) {
                        return true;  // ? 成功
                    } else {
                        ROS_WARN("CRC mismatch (calc=0x%04X, recv=0x%04X)", crc_calc, crc_recv);
                        return false;
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}





