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

/**
 * @brief MSP V2 接口主节点类
 * 
 * 负责：
 * - 通过串口与飞控通信（MSP V2 协议）
 * - 异步接收并解析飞控发来的传感器数据（IMU、高度等），发布到 /msp_sensor 话题
 * - 订阅 /msp_channel 话题，将控制通道数据发送给飞控
 */
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

        // 2. 初始化发布者和订阅者
        sensor_pub_ = nh_.advertise<msp_interface::MspSensor>("msp_sensor", 10);
        channel_sub_ = nh_.subscribe("msp_channel", 10, &MspInterfaceNode::channelCallback, this);

        // 3. 创建串口驱动对象并打开串口
        serial_.reset(new SerialDriver());
        if (!serial_->open(port_, baud_))
        {
            ROS_FATAL("无法打开串口 %s，请检查权限和设备是否存在", port_.c_str());
            ros::shutdown();
            return;
        }
        ROS_INFO("串口 %s 已打开，波特率 %d", port_.c_str(), baud_);

        // 4. 启动异步读取线程，将收到的字节流交给解析器
        serial_->startAsyncRead(std::bind(&MspInterfaceNode::onSerialData, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));

        // 5. 创建定时器，定期向飞控请求传感器数据（例如 50Hz）
        //    MSP 协议中，飞控通常不会主动推送数据，需要主机发送请求命令
        request_timer_ = nh_.createTimer(ros::Duration(0.02),   // 50ms = 20Hz
                                         &MspInterfaceNode::requestTimerCallback, this);

        ROS_INFO("MSP 接口节点已启动，请求频率 20Hz");
    }

    ~MspInterfaceNode()
    {
        running_ = false;
        if (serial_)
            serial_->close();
        if (read_thread_.joinable())
            read_thread_.join();
    }

private:
    /**
     * @brief 串口数据接收回调（在串口驱动线程中执行）
     * @param data 字节数组指针
     * @param len  数据长度
     */
    void onSerialData(const uint8_t* data, size_t len)
    {
        std::lock_guard<std::mutex> lock(parser_mutex_);
        for (size_t i = 0; i < len; ++i)
        {
            std::vector<uint8_t> packet;
            if (parser_.parseByte(data[i], packet))
            {
                // 成功解析出一个完整 MSP V2 数据包
                // 注意：此处假设飞控返回的是传感器数据（命令 ID 由您根据实际固件定义）
                // 您需要根据实际命令 ID 判断，并正确解析 packet 中的数据
                handleSensorPacket(packet);
            }
        }
    }

    /**
     * @brief 处理解析出的完整传感器数据包
     * @param payload  MSP V2 数据包中的有效载荷（不包含帧头、命令、长度和 CRC）
     */
    void handleSensorPacket(const std::vector<uint8_t>& payload)
    {
        // 根据您的飞控固件（如 INAV）解析 payload 中的数据
        // 以下是一个示例，假设 IMU 数据按顺序排列：gyro(3*float), acc(3*float), quat(4*float), altitude(float)
        // 实际数据格式和命令 ID 请参考飞控源码（如 msp_protocol_v2.h）
        if (payload.size() < 4*8)   // 至少需要 8 个 float = 32 字节
        {
            ROS_WARN_THROTTLE(1.0, "接收到的传感器数据长度不足：%zu 字节", payload.size());
            return;
        }

        msp_interface::MspSensor msg;
        msg.header.stamp = ros::Time::now();

        // 将字节数组转换为 float 数组（假设飞控使用小端序）
        const float* data = reinterpret_cast<const float*>(payload.data());
        msg.gyro_x = data[0];
        msg.gyro_y = data[1];
        msg.gyro_z = data[2];
        msg.acc_x  = data[3];
        msg.acc_y  = data[4];
        msg.acc_z  = data[5];
        msg.orientation_x = data[6];
        msg.orientation_y = data[7];
        msg.orientation_z = data[8];
        msg.orientation_w = data[9];
        msg.altitude = data[10];   // 假设第 11 个 float 是高度

        sensor_pub_.publish(msg);
        ROS_DEBUG("发布传感器数据：gyro(%.2f,%.2f,%.2f) alt=%.2f",
                  msg.gyro_x, msg.gyro_y, msg.gyro_z, msg.altitude);
    }

    /**
     * @brief 定时器回调，向飞控发送传感器数据请求
     */
    void requestTimerCallback(const ros::TimerEvent&)
    {
        // 需要根据您的飞控固件确定正确的请求命令 ID
        // 例如 INAV 中请求 IMU 数据的命令可能是 MSP2_SENSOR_IMU = 0x1F00
        // 请在 msp_protocol.h 中定义这些常量
        uint16_t cmd = MSP2_SENSOR_IMU;   // 需要您根据实际固件定义
        std::vector<uint8_t> request = packMspV2Request(cmd, nullptr, 0);
        if (!request.empty())
        {
            if (!serial_->write(request.data(), request.size()))
            {
                ROS_ERROR_THROTTLE(1.0, "发送传感器请求失败");
            }
        }
    }

    /**
     * @brief 订阅话题 /msp_channel 的回调，将控制数据发送给飞控
     * @param msg 控制通道消息
     */
    void channelCallback(const msp_interface::MspChannel::ConstPtr& msg)
    {
        // 将 ROS 消息转换为 MSP V2 负载
        // 通常每个通道占 2 字节（uint16_t），按小端序排列
        std::vector<uint8_t> payload;
        for (uint16_t ch : msg->channels)
        {
            payload.push_back(ch & 0xFF);
            payload.push_back((ch >> 8) & 0xFF);
        }

        // 打包成 MSP V2 请求帧，命令 ID 通常为 MSP2_RC
        uint16_t cmd = MSP2_RC;   // 需要您根据实际固件定义
        std::vector<uint8_t> request = packMspV2Request(cmd, payload.data(), payload.size());

        if (!request.empty())
        {
            if (!serial_->write(request.data(), request.size()))
            {
                ROS_ERROR("发送控制通道数据失败");
            }
        }
    }

private:
    ros::NodeHandle nh_;               // 公共节点句柄
    ros::NodeHandle nh_priv_;           // 私有节点句柄
    std::string port_;                  // 串口设备
    int baud_;                          // 波特率
    std::unique_ptr<SerialDriver> serial_;   // 串口驱动对象
    std::thread read_thread_;            // 读取线程（由 SerialDriver 内部管理，此处不需要直接使用）
    std::atomic<bool> running_;          // 运行标志

    // MSP 协议解析器
    MspV2Parser parser_;
    std::mutex parser_mutex_;            // 保护 parser_ 的互斥锁（因在多个线程中调用）

    ros::Publisher sensor_pub_;          // 传感器数据发布者
    ros::Subscriber channel_sub_;        // 控制通道订阅者
    ros::Timer request_timer_;           // 传感器请求定时器
};

} // namespace msp_interface

int main(int argc, char** argv)
{
    ros::init(argc, argv, "msp_interface_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    msp_interface::MspInterfaceNode node(nh, nh_priv);

    // 多线程 spinner 允许回调函数（如定时器、订阅回调）在 separate 线程中执行
    ros::AsyncSpinner spinner(2);   // 使用 2 个线程
    spinner.start();
    ros::waitForShutdown();

    return 0;
}