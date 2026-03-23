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
        : nh_(nh)
    {
        // 1. 锟斤拷取锟斤拷锟斤拷
        nh_.param<std::string>("gim_port_name", port_name_, "/dev/ttyUSB1");
        nh_.param<int>("gim_baud_rate", baud_rate_, 115200);
        nh_.param<double>("gim_publish_rate", publish_rate_, 50.0);

        // 2. 锟斤拷始锟斤拷锟斤拷锟斤拷
        //serial_ = std::make_shared<GimbalSerial>(port_name_, baud_rate_);
        serial_.reset(new GimbalSerial(port_name_, baud_rate_));
        if (!serial_->open()) {
            ROS_ERROR("? Failed to open serial port: %s", port_name_.c_str());
            ros::shutdown();
        } else {
            ROS_INFO("? Serial port %s opened at %d baud", port_name_.c_str(), baud_rate_);
        }

        // 3. 锟斤拷始锟斤拷锟斤拷锟斤拷
        cmd_sub_ = nh_.subscribe("/gimbal/cmd", 10, &GimbalControlNode::cmdCallback, this);
        raw_tx_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("/gimbal/raw_tx", 10);

        // 4. 锟斤拷锟斤拷锟斤拷时锟斤拷
        timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), &GimbalControlNode::timerCallback, this);
        ROS_INFO("Gimbal control node initialized, sending at %.1f Hz", publish_rate_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher raw_tx_pub_;
    ros::Timer timer_;
    std::unique_ptr<GimbalSerial> serial_;   // 锟接猴拷锟绞硷拷锟�
    //std::shared_ptr<GimbalSerial> serial_;   // ? 锟侥筹拷锟斤拷锟斤拷指锟斤拷锟斤拷锟接猴拷锟绞硷拷锟�

    std::string port_name_;
    int baud_rate_;
    double publish_rate_;

    std::mutex mutex_;
    gimbal_control_serial::GimbalCmd last_cmd_; // 锟芥储锟斤拷锟铰斤拷锟秸碉拷锟侥伙拷锟斤拷锟斤拷锟斤拷

    // 锟斤拷锟斤拷氐锟�
    void cmdCallback(const gimbal_control_serial::GimbalCmd::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_cmd_ = *msg;
    }

    // 锟斤拷时锟斤拷锟截碉拷
    void timerCallback(const ros::TimerEvent&)
    {
        Gcu2GbcPkt_t pkt = {0};
        memset(&pkt, 0, sizeof(pkt)); // 锟斤拷证锟斤拷锟斤拷
        
        // 锟斤拷锟叫拷锟斤拷锟斤拷锟�
        pkt.sync[0] = 0xA9;
        pkt.sync[1] = 0x5B;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // 锟斤拷锟斤拷锟接筹拷锟� roll/pitch/yaw 锟斤拷 uav.angle
            pkt.uav.valid = 0;
            pkt.uav.angle[0] = static_cast<int16_t>(last_cmd_.roll * 100);  // deg -> 0.01deg
            pkt.uav.angle[1] = static_cast<int16_t>(last_cmd_.pitch * 100);
            pkt.uav.angle[2] = static_cast<int16_t>(last_cmd_.yaw * 100);

            // 锟斤拷锟斤拷模式/锟斤拷锟斤拷模式映锟戒到 gbc[0] 锟斤拷锟斤拷展
            for(int i=0;i<3;i++) {
                pkt.gbc[i].go_zero = 0;
                pkt.gbc[i].wk_mode = 0;   // 锟斤拷锟斤拷模式
                pkt.gbc[i].op_type = 0;   // 锟角度匡拷锟斤拷
                pkt.gbc[i].op_value = pkt.uav.angle[i];
            }
            //pkt.aux.fl_sens=4;
            //pkt.gbc[0].wk_mode=1;
            //pkt.gbc[1].op_type=2;
            //pkt.gbc[1].wk_mode=1;
            //pkt.gbc[2].op_type=2;
        }
        pkt.uav.angle[0] = 0;
        pkt.uav.angle[1] = 0;
        pkt.uav.angle[2] = 0;
        pkt.cmd.value=4;
        // 锟斤拷锟斤拷CRC
        uint16_t crc = CalculateCrc16(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt)-2);
        pkt.crc[1] = crc & 0xFF;
        pkt.crc[0] = (crc >> 8) & 0xFF;


        Gbc2GcuPkt_t recv_pkt;
        bool ok = serial_->sendAndWaitReply(pkt, recv_pkt, 5);  // 5ms 锟斤拷时

        if (ok) {
            // ROS_INFO("?? Recv OK: cam_angle = %.2f %.2f %.2f",
            //          recv_pkt.cam_angle[0] * 0.01,
            //          recv_pkt.cam_angle[1] * 0.01,
            //          recv_pkt.cam_angle[2] * 0.01);
        } else {
            ROS_WARN("?? No response from gimbal");
        }

        std_msgs::UInt8MultiArray raw_msg;
        raw_msg.data.resize(sizeof(pkt));
        memcpy(raw_msg.data.data(), &pkt, sizeof(pkt));
        raw_tx_pub_.publish(raw_msg);

        // 锟斤拷锟酵达拷锟斤拷
        //if (!serial_->sendPacket(pkt)) {
        //    ROS_WARN("?? Failed to send packet");
        //}

        // 锟斤拷锟斤拷原始锟斤拷锟捷帮拷
        //std_msgs::UInt8MultiArray raw_msg;
        //raw_msg.data.resize(sizeof(pkt));
        //memcpy(raw_msg.data.data(), &pkt, sizeof(pkt));
        //raw_tx_pub_.publish(raw_msg);
        
        // ?锟睫改ｏ拷锟斤拷锟接斤拷锟秸癸拷锟斤拷
        //Gbc2GcuPkt_t recv_pkt;
        //if (serial_.readPacket(recv_pkt))  // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟揭拷锟� gimbal_serial.cpp 锟斤拷实锟斤拷
        //{
            // 锟斤拷印锟斤拷锟秸碉拷锟斤拷锟斤拷锟斤拷
            //ROS_INFO("Recv: roll=%.2f pitch=%.2f yaw=%.2f",
                     //recv_pkt.cam_angle[0] * 0.01,
                     //recv_pkt.cam_angle[1] * 0.01,
                     //recv_pkt.cam_angle[2] * 0.01);
        //}
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
