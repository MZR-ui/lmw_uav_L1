#include <ros/ros.h>
#include <mutex>
#include <vector>
#include "msp_interface/MspChannel.h"
#include "remote_info/Remote.h"  // 需要添加对 remote_info 消息的依赖

namespace msp_interface
{

class ModeControllerNode
{
public:
    ModeControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh), nh_priv_(nh_priv)
    {
        // 读取参数：发布频率，默认50Hz
        nh_priv_.param<double>("publish_rate", publish_rate_, 50.0);

        // 创建发布者（飞控通道）
        channel_pub_ = nh_.advertise<msp_interface::MspChannel>("msp_channel", 10);

        // 订阅遥控器原始数据话题（/remote_order
        remote_sub_ = nh_.subscribe("remote_order", 10, &ModeControllerNode::remoteCallback, this);

        // 创建定时器
        double period = 1.0 / publish_rate_;
        timer_ = nh_.createTimer(ros::Duration(period), &ModeControllerNode::timerCallback, this);

        ROS_INFO("ModeControllerNode started, publishing at %.1f Hz", publish_rate_);
    }

private:
    void remoteCallback(const remote_info::Remote::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_remote_channels_ = msg->channels;  // 保存遥控器通道值
        ROS_DEBUG_THROTTLE(1.0, "Received remote data with %zu channels", last_remote_channels_.size());
    }

    void timerCallback(const ros::TimerEvent&)
    {
        msp_interface::MspChannel cmd_msg;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (last_remote_channels_.empty()) {
                // 无遥控器数据时，发布默认安全值（8通道，油门最低）
                cmd_msg.channels = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500};
            } else {
                // 将遥控器通道映射到飞控通道（通常飞控只需前8个通道）
                cmd_msg.channels.resize(8, 1500);
                size_t copy_len = std::min(last_remote_channels_.size(), size_t(8));
                for (size_t i = 0; i < copy_len; ++i) {
                    cmd_msg.channels[i] = last_remote_channels_[i];
                }

                // 可选：根据遥控器通道5切换模式（例如 >1500 为自动模式）
                // 此处可根据需求修改某些通道值
            }
        }
        channel_pub_.publish(cmd_msg);
        ROS_DEBUG_THROTTLE(1.0, "Published channel data (%zu channels)", cmd_msg.channels.size());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    double publish_rate_;
    ros::Publisher channel_pub_;
    ros::Subscriber remote_sub_;
    ros::Timer timer_;

    std::vector<uint16_t> last_remote_channels_;  // 保存最近接收到的遥控器通道
    std::mutex mutex_;
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