#include <ros/ros.h>
#include <mutex>
#include <vector>
#include "msp_interface/MspChannel.h"
#include "remote_info/Remote.h"
#include "subtask/ControlData.h"
#include "gimbal_control_serial/GimbalCmd.h" 

namespace msp_interface
{

class ModeControllerNode
{
public:
    ModeControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh), nh_priv_(nh_priv), use_control_data_(false), use_remote_direct_(false)
    {
        // 读取参数
        nh_priv_.param<double>("publish_rate", publish_rate_, 50.0);
        nh_priv_.param<int>("max_channels", max_channels_, 16);
        nh_priv_.param<double>("remote_timeout", remote_timeout_, 0.1);   // 遥控器超时 (秒)
        nh_priv_.param<double>("control_timeout", control_timeout_, 0.1); // 控制数据超时 (秒)
        nh_priv_.param<double>("gimbal_angle_range", gimbal_angle_range_, 45.0);

        // 发布者和订阅者队列大小设为1，降低延迟
        channel_pub_ = nh_.advertise<msp_interface::MspChannel>("msp_channel", 1);
        remote_sub_ = nh_.subscribe("remote_order", 1, &ModeControllerNode::remoteCallback, this);
        control_sub_ = nh_.subscribe("/control_data", 1, &ModeControllerNode::controlCallback, this);
        gimbal_pub_ = nh_.advertise<gimbal_control_serial::GimbalCmd>("/gimbal/cmd", 1);

        // 定时器
        double period = 1.0 / publish_rate_;
        timer_ = nh_.createTimer(ros::Duration(period), &ModeControllerNode::timerCallback, this);

        // 初始化最后接收时间（设为0表示从未收到）
        last_remote_time_ = ros::Time(0);
        last_control_time_ = ros::Time(0);

        ROS_INFO("ModeControllerNode started, publishing at %.1f Hz, max_channels=%d", publish_rate_, max_channels_);
        ROS_INFO("Timeouts: remote=%.2fs, control=%.2fs", remote_timeout_, control_timeout_);
        ROS_INFO("Gimbal angle range: +/- %.1f deg", gimbal_angle_range_);
    }

private:
    void remoteCallback(const remote_info::Remote::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_remote_channels_ = msg->channels;
        last_remote_time_ = ros::Time::now();   // 更新最后接收时间

        // 根据遥控器通道8（索引7）的值更新强制遥控器模式标志
        if (last_remote_channels_.size() > 7) {
            use_remote_direct_ = (last_remote_channels_[7] < 1350);
            if (use_remote_direct_) {
                use_control_data_ = false;  // 强制遥控器模式时，清除控制数据标志
            }
        } else {
            use_remote_direct_ = false;
        }
        ROS_DEBUG_THROTTLE(1.0, "Received remote data with %zu channels, use_remote_direct=%d",
                           last_remote_channels_.size(), use_remote_direct_);
    }

    void controlCallback(const subtask::ControlData::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 仅在非强制遥控器模式下才接受控制数据
        if (!use_remote_direct_) {
            last_control_data_ = *msg;
            last_control_time_ = ros::Time::now();   // 更新最后接收时间
            use_control_data_ = true;
            ROS_DEBUG_THROTTLE(1.0, "Received control data");
        } else {
            ROS_DEBUG_THROTTLE(1.0, "Ignored control data (remote direct mode active)");
        }
    }

    void timerCallback(const ros::TimerEvent&)
    {
        msp_interface::MspChannel cmd_msg;
        ros::Time now = ros::Time::now();

        {
            std::lock_guard<std::mutex> lock(mutex_);

            // 检查数据源有效性
            bool remote_valid = (last_remote_time_ != ros::Time(0)) && 
                                (now - last_remote_time_ < ros::Duration(remote_timeout_));
            bool control_valid = (last_control_time_ != ros::Time(0)) && 
                                 (now - last_control_time_ < ros::Duration(control_timeout_));

            // 如果遥控器超时，强制遥控器模式标志应被忽略（数据已失效）
            // 注意：use_remote_direct_ 仅在遥控器有效时才有意义
            if (!remote_valid) {
                use_remote_direct_ = false;   // 超时后不再强制遥控器
            }
            
            // ----- 发布飞控通道 -----
            // 决策逻辑：优先级 强制遥控器(有效) > 控制数据(有效) > 普通遥控器(有效) > 默认安全值
            if (remote_valid && use_remote_direct_) {
                // 强制遥控器模式：直接使用遥控器数据
                size_t channels_to_copy = std::min(last_remote_channels_.size(), static_cast<size_t>(max_channels_));
                cmd_msg.channels.assign(max_channels_, 1500);
                for (size_t i = 0; i < channels_to_copy; ++i) {
                    cmd_msg.channels[i] = last_remote_channels_[i];
                }
                ROS_DEBUG_THROTTLE(1.0, "Using remote direct mode (remote valid)");
            }
            else if (control_valid) {
                // 控制数据模式：仅填充前4个通道
                cmd_msg.channels.assign(max_channels_, 1500);
                cmd_msg.channels[0] = static_cast<uint16_t>(last_control_data_.roll);
                cmd_msg.channels[1] = static_cast<uint16_t>(last_control_data_.pitch);
                cmd_msg.channels[2] = static_cast<uint16_t>(last_control_data_.throttle);
                cmd_msg.channels[3] = static_cast<uint16_t>(last_control_data_.yaw);
                ROS_DEBUG_THROTTLE(1.0, "Using control data (control valid)");
            }
            else if (remote_valid) {
                // 普通遥控器模式（遥控器有效但非强制）
                size_t channels_to_copy = std::min(last_remote_channels_.size(), static_cast<size_t>(max_channels_));
                cmd_msg.channels.assign(max_channels_, 1500);
                for (size_t i = 0; i < channels_to_copy; ++i) {
                    cmd_msg.channels[i] = last_remote_channels_[i];
                }
                ROS_DEBUG_THROTTLE(1.0, "Using remote (default)");
            }
            else {
                // 无任何有效数据，发布默认安全值
                cmd_msg.channels.assign(max_channels_, 1500);
                cmd_msg.channels[2] = 1000;  // 油门最低
                ROS_DEBUG_THROTTLE(1.0, "Using default safe values (no valid data)");
            }

            // ----- 发布云台指令（仅在强制遥控器模式下） -----
            if (remote_valid && use_remote_direct_ && last_remote_channels_.size() >= 10) {
                // 通道9（索引8）→ roll, 通道10（索引9）→ yaw
                uint16_t ch9 = last_remote_channels_[8];
                uint16_t ch10 = last_remote_channels_[9];

                // 将通道值 1000~2000 映射到 ±gimbal_angle_range_ 角度
                float roll = (ch9 - 1500.0f) / 500.0f * gimbal_angle_range_;
                float yaw   = (ch10 - 1500.0f) / 500.0f * gimbal_angle_range_;

                gimbal_control_serial::GimbalCmd gimbal_msg;
                gimbal_msg.roll  = roll;
                gimbal_msg.pitch = 0.0f;
                gimbal_msg.yaw   = yaw;
                gimbal_msg.mode  = 0;   // 模式0（根据需要可配置）

                gimbal_pub_.publish(gimbal_msg);
                ROS_DEBUG_THROTTLE(1.0, "Published gimbal cmd: roll=%.1f, yaw=%.1f", roll, yaw);
            }
        }

        channel_pub_.publish(cmd_msg);
        ROS_DEBUG_THROTTLE(1.0, "Published channel data (%zu channels)", cmd_msg.channels.size());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    double publish_rate_;
    int max_channels_;
    double remote_timeout_;    // 遥控器超时时间
    double control_timeout_;   // 控制数据超时时间
    ros::Publisher channel_pub_;
    ros::Subscriber remote_sub_;
    ros::Subscriber control_sub_;
    ros::Timer timer_;

    std::vector<uint16_t> last_remote_channels_;
    subtask::ControlData last_control_data_;
    bool use_control_data_;
    bool use_remote_direct_;
    ros::Time last_remote_time_;   // 最后一次收到遥控器数据的时间
    ros::Time last_control_time_;  // 最后一次收到控制数据的时间
    std::mutex mutex_;

    ros::Publisher gimbal_pub_;                     // 云台指令发布者
    double gimbal_angle_range_;                     // 云台角度范围（度）
};

} // namespace msp_interface

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mode_manage_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    msp_interface::ModeControllerNode node(nh, nh_priv);

    ros::spin();
    return 0;
}