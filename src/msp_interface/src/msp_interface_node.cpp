#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

// 自定义消息
#include "msp_interface/MspSensor.h"
#include "msp_interface/MspChannel.h"

// 串口驱动和协议解析头文件
#include "msp_interface/serial_driver.h"
#include "msp_interface/msp_protocol.h"
#include "msp_interface/msp_parser.h"

namespace msp_interface
{

class MspInterfaceNode
{
public:
    MspInterfaceNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh)
        , nh_priv_(nh_priv)
        , running_(true)
    {
        // 1. 读取 ROS 参数
        nh_priv_.param<std::string>("port", port_, "/dev/ttyUSB0");
        nh_priv_.param<int>("baud", baud_, 115200);
        nh_priv_.param<double>("loop_rate", loop_rate_, 1000.0);
        nh_priv_.param<double>("sensor_rate", sensor_rate_, 50.0);      // 传感器请求频率 (Hz)
        nh_priv_.param<bool>("simulate_channels", simulate_channels_, false); // 是否模拟通道数据
        nh_priv_.param<double>("channel_rate", channel_rate_, 50.0);     // 模拟通道发送频率 (Hz)

        if (loop_rate_ > 0 && sensor_rate_ > 0) {
            sensor_interval_ = static_cast<int>(loop_rate_ / sensor_rate_);
            if (sensor_interval_ < 1) sensor_interval_ = 1;
        } else {
            sensor_interval_ = 1;
        }
        if (loop_rate_ > 0 && channel_rate_ > 0 && simulate_channels_) {
            channel_interval_ = static_cast<int>(loop_rate_ / channel_rate_);
            if (channel_interval_ < 1) channel_interval_ = 1;
        } else {
            channel_interval_ = 0;  // 不发送
        }

        ROS_INFO("Loop rate: %.1f Hz, sensor interval: %d, channel interval: %d",
                 loop_rate_, sensor_interval_, channel_interval_);

        // 2. 初始化发布者和订阅者
        sensor_pub_ = nh_.advertise<msp_interface::MspSensor>("msp_sensor", 10);
        channel_sub_ = nh_.subscribe("msp_channel", 10, &MspInterfaceNode::channelCallback, this);

        // 3. 创建串口驱动对象并打开串口
        serial_.reset(new SerialDriver());
        if (!serial_->open(port_, baud_))
        {
            ROS_FATAL("Failed to open serial port %s, please check permissions and device existence", port_.c_str());
            ros::shutdown();
            return;
        }
        ROS_INFO("Serial port %s opened (baudrate %d)", port_.c_str(), baud_);

        // 4. 启动异步读取线程，将收到的字节流交给解析器
        serial_->startAsyncRead(std::bind(&MspInterfaceNode::onSerialData, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));

        // 5. 创建主循环定时器
        double loop_period = 1.0 / loop_rate_;
        loop_timer_ = nh_.createTimer(ros::Duration(loop_period),
                                      &MspInterfaceNode::loopCallback, this);
        ROS_INFO("Main loop timer started at %.1f Hz", loop_rate_);
        ROS_INFO("MSP interface node started successfully");

    }

    ~MspInterfaceNode()
    {
        running_ = false;
        if (serial_)
            serial_->close();
    }

private:
    
    // void handleAttitude(const std::vector<uint8_t>& payload);

    /**
     * @brief 串口数据接收回调（在串口驱动线程中执行）
     */
    void onSerialData(const uint8_t* data, size_t len)
    {
        //ROS_INFO_THROTTLE(5.0, "Receive data：");
        //printf("Receive data: ptr=%p, len=%zu\n", data, len);
        // 拼接前16字节的十六进制字符串（防止单条日志过长）

        // std::stringstream ss;
        // ss << "Receive data: len=" << len << ", data (first 16 bytes): ";
        // for (size_t i = 0; i < len && i < 16; ++i) {
        //     ss << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
        // }
        // if (len > 16) ss << "...";
        // ROS_INFO("%s", ss.str().c_str());

        std::lock_guard<std::mutex> lock(parser_mutex_);
        for (size_t i = 0; i < len; ++i)
        {
            std::vector<uint8_t> packet;
            if (parser_.parseByte(data[i], packet))
            {
                uint16_t cmd = parser_.getLastCommand();  // 获取命令 ID
                handleSensorPacket(cmd, packet);
            }
        }
    }

    /**
     * @brief 处理解析出的完整传感器数据包
     */

     void handleSensorPacket(uint16_t cmd, const std::vector<uint8_t>& payload) {
        switch(cmd) {
            /*Receive info*/
            case MSP_ATTITUDE: handleAttitude(payload); break;
            case MSP2_RC: break;
            // case MSP_ALTITUDE: handleAltitude(payload); break;
            /*Receive Ack*/
            default: ROS_WARN_THROTTLE(1.0, "Unknown command: %d", cmd); break;
        }
    }

    void handleAttitude(const std::vector<uint8_t>& payload)
    {
        if (payload.size() != 6)  // 三个 int16_t = 6 字节
        {
            ROS_WARN_THROTTLE(1.2, "Invalid attitude packet size: %zu bytes (expected 6)", payload.size());
            return;
        }

        // 解析三个 int16_t (小端序)
        int16_t roll_raw  = static_cast<int16_t>(payload[0] | (payload[1] << 8));
        int16_t pitch_raw = static_cast<int16_t>(payload[2] | (payload[3] << 8));
        int16_t yaw_raw   = static_cast<int16_t>(payload[4] | (payload[5] << 8));

        // 转换为度 (原始单位为 0.1 度)
        float roll  = roll_raw  / 10.0f;
        float pitch = pitch_raw / 10.0f;
        float yaw   = yaw_raw   / 1.0f;

        // 临时填充到 MspSensor 消息（注意语义不符）
        msp_interface::MspSensor msg;
        msg.header.stamp = ros::Time::now();
        msg.orientation_x = roll;
        msg.orientation_y = pitch;
        msg.orientation_z = yaw;
        msg.orientation_w = 1.0;  // 占位

        // 其余字段置零（可选）
        msg.gyro_x = msg.gyro_y = msg.gyro_z = 0;
        msg.acc_x = msg.acc_y = msg.acc_z = 0;
        msg.altitude = 0;
        sensor_pub_.publish(msg);
        ROS_INFO_THROTTLE(1, "Published attitude: roll=%.1f pitch=%.1f yaw=%.1f", roll, pitch, yaw);
    }

    void loopCallback(const ros::TimerEvent&)
    {
        counter_++;  // 全局计数器递增

        // 发送传感器请求（取模控制）
        if (sensor_interval_ > 0 && (counter_ % sensor_interval_ == 0))
        {
            sendRequest(MSP_ATTITUDE); 
        }

        // 发送模拟通道数据（如果启用）
        if (simulate_channels_ && channel_interval_ > 0 && (counter_ % channel_interval_ == 0))
        {
            std::vector<uint16_t> channels(10, 1500);  // 模拟 10 个通道中位值
            sendChannels(channels);
        }
    }

    /**
     * @brief 定时器回调，向飞控发送传感器数据请求
     */
    void sendRequest(uint16_t cmd)
    {
        std::vector<uint8_t> request = packMspV2Request(cmd, nullptr, 0);
        if (!request.empty())
        {
            std::lock_guard<std::mutex> lock(serial_write_mutex_);
            if (!serial_->write(request.data(), request.size()))
            {
                ROS_ERROR_THROTTLE(1.0, "Failed to send request (cmd=%u)", cmd);
            }
        }
    }

    /**
     * @brief 订阅话题 /msp_channel 的回调，将控制数据发送给飞控
     */
    void channelCallback(const msp_interface::MspChannel::ConstPtr& msg)
    {
        // ROS_INFO_THROTTLE(1.0, "Channel callback triggered (rate %.1f Hz)", channel_rate_);
        // ROS_INFO("Channel callback triggered (rate %.1f Hz)", channel_rate_);
        sendChannels(msg->channels);
    }

    /**
     * @brief 将通道数据打包并通过串口发送给飞控
     * @param channels 通道值数组
     */
    void sendChannels(const std::vector<uint16_t>& channels)
    {
        std::vector<uint8_t> payload;
        for (uint16_t ch : channels)
        {
            payload.push_back(ch & 0xFF);
            payload.push_back((ch >> 8) & 0xFF);
        }

        uint16_t cmd = MSP2_RC;  // 请确保宏定义正确
        std::vector<uint8_t> request = packMspV2Request(cmd, payload.data(), payload.size());

        if (!request.empty())
        {
            std::lock_guard<std::mutex> lock(serial_write_mutex_);
            if (!serial_->write(request.data(), request.size()))
            {
                ROS_ERROR("Failed to send channel data");
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    std::string port_;
    int baud_;
    double loop_rate_;
    double sensor_rate_;                // 传感器请求频率 (Hz)
    bool simulate_channels_;            // 是否模拟通道数据
    double channel_rate_;               // 模拟通道发送频率 (Hz)

    int sensor_interval_;    // 传感器发送间隔（tick 数）
    int channel_interval_;   // 模拟通道发送间隔
    int counter_;            // 全局 tick 计数器

    std::unique_ptr<SerialDriver> serial_;
    std::atomic<bool> running_;

    MspV2Parser parser_;
    std::mutex parser_mutex_;
    std::mutex serial_write_mutex_;    // 保护串口写入（多线程安全）

    ros::Publisher sensor_pub_;
    ros::Subscriber channel_sub_;
    ros::Timer request_timer_;
    ros::Timer channel_sim_timer_;      // 模拟通道定时器

    ros::Timer loop_timer_;             // 主循环定时器
};

} // namespace msp_interface

int main(int argc, char** argv)
{
    ros::init(argc, argv, "msp_interface_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    msp_interface::MspInterfaceNode node(nh, nh_priv);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}