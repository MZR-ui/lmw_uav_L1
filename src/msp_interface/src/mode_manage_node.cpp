#include <ros/ros.h>
#include "msp_interface/MspChannel.h"

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
        
        // 创建发布者
        channel_pub_ = nh_.advertise<msp_interface::MspChannel>("msp_channel", 10);
        
        // 创建定时器
        double period = 1.0 / publish_rate_;
        timer_ = nh_.createTimer(ros::Duration(period), &ModeControllerNode::timerCallback, this);
        
        ROS_INFO("ModeControllerNode started, publishing at %.1f Hz", publish_rate_);
    }

private:
    void timerCallback(const ros::TimerEvent&)
    {
        // 这里生成您想要的通道值，例如所有通道中位1500，或者根据需要修改
        // 可以设计为通过ROS参数或服务动态改变通道值，简单起见先固定
        msp_interface::MspChannel msg;
        // 假设我们想要发送8个通道，每个值1500
        msg.channels.resize(8, 1500);
        msg.channels[2]=1000;
        
        channel_pub_.publish(msg);
        ROS_DEBUG_THROTTLE(1.0, "Published channel data (all 1500)");
    }

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    double publish_rate_;
    ros::Publisher channel_pub_;
    ros::Timer timer_;
};

} // namespace msp_interface

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mode_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    msp_interface::ModeControllerNode node(nh, nh_priv);
    
    ros::spin();
    return 0;
}