#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include "gimbal_control_serial/gimbal_serial.hpp"
#include "gimbal_control_serial/gimbal_protocol.hpp"
#include "gimbal_control_serial/GimbalCmd.h"
#include <vector>
#include <mutex>

using namespace gimbal_protocol;

class GimbalControlNode
{
public:
    GimbalControlNode(ros::NodeHandle& nh)
        : nh_(nh), serial_(port_name_, baud_rate_)
    {
        // 1. 读取参数
        nh_.param<std::string>("port_name", port_name_, "/dev/ttyUSB0");
        nh_.param<int>("baud_rate", baud_rate_, 115200);
        nh_.param<double>("publish_rate", publish_rate_, 50.0);

        // 2. 初始化串口
        //serial_ = std::make_shared<GimbalSerial>(port_name_, baud_rate_);
        if (!serial_.open()) {
            ROS_ERROR("? Failed to open serial port: %s", port_name_.c_str());
            ros::shutdown();
        } else {
            ROS_INFO("? Serial port %s opened at %d baud", port_name_.c_str(), baud_rate_);
        }

        // 3. 初始化话题
        cmd_sub_ = nh_.subscribe("/gimbal/cmd", 10, &GimbalControlNode::cmdCallback, this);
        raw_tx_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("/gimbal/raw_tx", 10);

        // 4. 启动定时器
        timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), &GimbalControlNode::timerCallback, this);
        ROS_INFO("Gimbal control node initialized, sending at %.1f Hz", publish_rate_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher raw_tx_pub_;
    ros::Timer timer_;
    GimbalSerial serial_;
    //std::shared_ptr<GimbalSerial> serial_;   // ? 改成智能指针以延后初始化

    std::string port_name_;
    int baud_rate_;
    double publish_rate_;

    std::mutex mutex_;
    gimbal_control_serial::GimbalCmd last_cmd_; // 存储最新接收到的话题数据

    // 话题回调
    void cmdCallback(const gimbal_control_serial::GimbalCmd::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_cmd_ = *msg;
    }

    // 定时器回调
    void timerCallback(const ros::TimerEvent&)
    {
        Gcu2GbcPkt_t pkt = {0};

        // 填充协议数据
        pkt.sync[0] = 0xA9;
        pkt.sync[1] = 0x5B;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // 这里简单映射 roll/pitch/yaw 到 uav.angle
            pkt.uav.valid = 1;
            pkt.uav.angle[0] = static_cast<int16_t>(last_cmd_.roll * 100);  // deg -> 0.01deg
            pkt.uav.angle[1] = static_cast<int16_t>(last_cmd_.pitch * 100);
            pkt.uav.angle[2] = static_cast<int16_t>(last_cmd_.yaw * 100);

            // 工作模式/控制模式映射到 gbc[0] 可扩展
            for(int i=0;i<3;i++) {
                pkt.gbc[i].go_zero = 0;
                pkt.gbc[i].wk_mode = 1;   // 跟随模式
                pkt.gbc[i].op_type = 0;   // 角度控制
                pkt.gbc[i].op_value = pkt.uav.angle[i];
            }
        }
        pkt.uav.angle[0] = 0;
        pkt.uav.angle[1] = 0;
        pkt.uav.angle[2] = 0;
        pkt.cmd.value=2;
        // 计算CRC
        uint16_t crc = CalculateCrc16(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt)-2);
        pkt.crc[0] = crc & 0xFF;
        pkt.crc[1] = (crc >> 8) & 0xFF;

        // 发送串口
        if (!serial_.sendPacket(pkt)) {
            ROS_WARN("?? Failed to send packet");
        }

        // 发布原始数据包
        std_msgs::UInt8MultiArray raw_msg;
        raw_msg.data.resize(sizeof(pkt));
        memcpy(raw_msg.data.data(), &pkt, sizeof(pkt));
        raw_tx_pub_.publish(raw_msg);
        
        // ?修改：增加接收功能
        Gbc2GcuPkt_t recv_pkt;
        if (serial_.readPacket(recv_pkt))  // 这个函数你需要在 gimbal_serial.cpp 里实现
        {
            // 打印接收到的数据
            ROS_INFO("Recv: roll=%.2f pitch=%.2f yaw=%.2f",
                     recv_pkt.cam_angle[0] * 0.01,
                     recv_pkt.cam_angle[1] * 0.01,
                     recv_pkt.cam_angle[2] * 0.01);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gimbal_control_node");
    ros::NodeHandle nh("~");

    GimbalControlNode node(nh);

    ros::spin();
    return 0;
}
