#include <ros/ros.h>
#include <mutex>
#include <vector>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt32MultiArray.h>
#include <boost/filesystem.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include "msp_interface/MspChannel.h"
#include "msp_interface/EscTelem.h"
#include "msp_interface/Attitude.h"
#include "remote_info/Remote.h"
#include "subtask/ControlData.h"
#include "gimbal_control_serial/GimbalCmd.h"

namespace msp_interface
{

class ModeControllerNode
{
public:
    ModeControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh), nh_priv_(nh_priv), use_control_data_(false), use_remote_direct_(false),
          is_recording_(false), last_ch5_state_(false), auto_restart_on_timeout_(false)
    {
        nh_priv_.param<double>("publish_rate", publish_rate_, 100.0);
        nh_priv_.param<int>("max_channels", max_channels_, 16);
        nh_priv_.param<double>("remote_timeout", remote_timeout_, 0.1);
        nh_priv_.param<double>("control_timeout", control_timeout_, 0.1);
        nh_priv_.param<double>("gimbal_angle_range", gimbal_angle_range_, 45.0);

        nh_priv_.param<std::string>("bag_save_dir", bag_save_dir_, std::string(getenv("HOME")) + "/lmw_catkin_ws/uav_ws/data/rosbag");
        nh_priv_.param<int>("max_bag_files", max_bag_files_, 100);
        nh_priv_.param<double>("max_record_duration", max_record_duration_, 600.0);
        nh_priv_.param<int>("ch5_threshold", ch5_threshold_, 1350);
        nh_priv_.param<bool>("auto_restart_on_timeout", auto_restart_on_timeout_, true);

        channel_pub_ = nh_.advertise<msp_interface::MspChannel>("msp_channel", 1);
        remote_sub_ = nh_.subscribe("remote_order", 1, &ModeControllerNode::remoteCallback, this);
        control_sub_ = nh_.subscribe("/control_data", 1, &ModeControllerNode::controlCallback, this);
        gimbal_pub_ = nh_.advertise<gimbal_control_serial::GimbalCmd>("/gimbal/cmd", 1);

        imu_sub_ = nh_.subscribe("/msp/imu", 100, &ModeControllerNode::imuCallback, this);
        altitude_sub_ = nh_.subscribe("/msp/altitude", 100, &ModeControllerNode::altitudeCallback, this);
        gps_sub_ = nh_.subscribe("/msp/gps", 100, &ModeControllerNode::gpsCallback, this);
        motor_sub_ = nh_.subscribe("/msp/motor", 100, &ModeControllerNode::motorCallback, this);
        status_sub_ = nh_.subscribe("/msp/status", 100, &ModeControllerNode::statusCallback, this);
        battery_sub_ = nh_.subscribe("/msp/battery", 100, &ModeControllerNode::batteryCallback, this);
        esc_sub_ = nh_.subscribe("/msp/esc_telem", 100, &ModeControllerNode::escCallback, this);
        attitude_sub_ = nh_.subscribe("/msp/attitude", 100, &ModeControllerNode::attitudeCallback, this);
        msp_channel_sub_ = nh_.subscribe("/msp/channel", 100, &ModeControllerNode::mspChannelCallback, this);

        double period = 1.0 / publish_rate_;
        timer_ = nh_.createTimer(ros::Duration(period), &ModeControllerNode::timerCallback, this);

        last_remote_time_ = ros::Time(0);
        last_control_time_ = ros::Time(0);

        createBagDirectory();

        ROS_INFO("ModeControllerNode started, publishing at %.1f Hz, max_channels=%d", publish_rate_, max_channels_);
        ROS_INFO("Timeouts: remote=%.2fs, control=%.2fs", remote_timeout_, control_timeout_);
        ROS_INFO("Gimbal angle range: +/- %.1f deg", gimbal_angle_range_);
        ROS_INFO("Bag recording: dir=%s, max_files=%d, max_duration=%.1fs",
                 bag_save_dir_.c_str(), max_bag_files_, max_record_duration_);
    }

    ~ModeControllerNode()
    {
        stopRecording();
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (is_recording_ && bag_.isOpen()) {
            bag_.write("/msp/imu", ros::Time::now(), msg);
        }
    }

    void altitudeCallback(const geometry_msgs::Point::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (is_recording_ && bag_.isOpen()) {
            bag_.write("/msp/altitude", ros::Time::now(), msg);
        }
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (is_recording_ && bag_.isOpen()) {
            bag_.write("/msp/gps", ros::Time::now(), msg);
        }
    }

    void motorCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (is_recording_ && bag_.isOpen()) {
            bag_.write("/msp/motor", ros::Time::now(), msg);
        }
    }

    void statusCallback(const std_msgs::UInt32MultiArray::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (is_recording_ && bag_.isOpen()) {
            bag_.write("/msp/status", ros::Time::now(), msg);
        }
    }

    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (is_recording_ && bag_.isOpen()) {
            bag_.write("/msp/battery", ros::Time::now(), msg);
        }
    }

    void escCallback(const msp_interface::EscTelem::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (is_recording_ && bag_.isOpen()) {
            bag_.write("/msp/esc_telem", ros::Time::now(), msg);
        }
    }

    void attitudeCallback(const msp_interface::Attitude::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (is_recording_ && bag_.isOpen()) {
            bag_.write("/msp/attitude", ros::Time::now(), msg);
        }
    }

    void mspChannelCallback(const msp_interface::MspChannel::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (is_recording_ && bag_.isOpen()) {
            bag_.write("/msp/channel", ros::Time::now(), msg);
        }
    }

    void createBagDirectory()
    {
        try {
            boost::filesystem::path dir(bag_save_dir_);
            if (!boost::filesystem::exists(dir)) {
                boost::filesystem::create_directories(dir);
                ROS_INFO("Created bag directory: %s", bag_save_dir_.c_str());
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to create bag directory: %s", e.what());
        }
    }

    std::string generateBagFilename()
    {
        std::vector<std::string> existing_files;
        try {
            boost::filesystem::path dir(bag_save_dir_);
            if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir)) {
                for (auto& entry : boost::filesystem::directory_iterator(dir)) {
                    if (entry.path().extension() == ".bag") {
                        existing_files.push_back(entry.path().filename().string());
                    }
                }
            }
        } catch (const std::exception& e) {
            ROS_WARN("Failed to list bag files: %s", e.what());
        }

        if (existing_files.size() >= static_cast<size_t>(max_bag_files_)) {
            std::sort(existing_files.begin(), existing_files.end());
            size_t files_to_delete = existing_files.size() - max_bag_files_ + 1;
            for (size_t i = 0; i < files_to_delete; ++i) {
                std::string file_to_delete = bag_save_dir_ + "/" + existing_files[i];
                try {
                    boost::filesystem::remove(file_to_delete);
                    ROS_INFO("Deleted old bag file: %s", existing_files[i].c_str());
                } catch (const std::exception& e) {
                    ROS_WARN("Failed to delete old bag file: %s", e.what());
                }
            }
        }

        int next_index = existing_files.size() + 1;
        if (next_index > max_bag_files_) {
            next_index = 1;
        }

        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now;
        localtime_r(&time_t_now, &tm_now);

        std::ostringstream oss;
        oss << bag_save_dir_ << "/"
            << std::setfill('0') << std::setw(3) << next_index << "_"
            << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << ".bag";

        return oss.str();
    }

    void startRecording()
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (is_recording_) {
            return;
        }

        try {
            std::string filename = generateBagFilename();
            bag_.open(filename, rosbag::bagmode::Write);
            is_recording_ = true;
            record_start_time_ = ros::Time::now();
            ROS_INFO("Started recording to: %s", filename.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to start recording: %s", e.what());
            is_recording_ = false;
        }
    }

    void stopRecording()
    {
        std::lock_guard<std::mutex> lock(bag_mutex_);
        if (!is_recording_) {
            return;
        }

        try {
            if (bag_.isOpen()) {
                bag_.close();
            }
            is_recording_ = false;
            ROS_INFO("Stopped recording");
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to stop recording: %s", e.what());
        }
    }

    void checkRecordingDuration()
    {
        if (is_recording_) {
            ros::Duration elapsed = ros::Time::now() - record_start_time_;
            if (elapsed.toSec() >= max_record_duration_) {
                ROS_INFO("Recording duration limit reached (%.1fs), stopping...", max_record_duration_);
                stopRecording();

                // If CH5 is still HIGH, immediately start a new bag to keep recording continuously.
                if (auto_restart_on_timeout_ && last_ch5_state_) {
                    ROS_INFO("CH5 still HIGH, starting a new bag for continuous recording");
                    startRecording();
                }
            }
        }
    }

    void remoteCallback(const remote_info::Remote::ConstPtr& msg)
    {
        bool should_start_recording = false;
        bool should_stop_recording = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_remote_channels_ = msg->channels;
            last_remote_time_ = ros::Time::now();

            if (last_remote_channels_.size() > 7) {
                // Channel 8 controls use_remote_direct mode (original functionality)
                use_remote_direct_ = (last_remote_channels_[7] < 1350);

                if (use_remote_direct_) {
                    use_control_data_ = false;
                }

                // Channel 5 controls recording: HIGH (>= threshold) = recording state
                // Rising edge  (LOW -> HIGH): start recording
                // Falling edge (HIGH -> LOW): stop recording
                bool current_ch5_state = (last_remote_channels_[4] >= ch5_threshold_);

                static bool first_callback = true;
                if (first_callback) {
                    last_ch5_state_ = current_ch5_state;
                    first_callback = false;
                    ROS_INFO("Initial CH5 state: %s (value=%d, threshold=%d)",
                             current_ch5_state ? "HIGH (>=threshold)" : "LOW (<threshold)",
                             last_remote_channels_[4], ch5_threshold_);
                } else {
                    if (current_ch5_state && !last_ch5_state_) {
                        should_start_recording = true;
                        ROS_INFO("CH5 rising edge detected: LOW -> HIGH (value=%d)", last_remote_channels_[4]);
                    } else if (!current_ch5_state && last_ch5_state_) {
                        should_stop_recording = true;
                        ROS_INFO("CH5 falling edge detected: HIGH -> LOW (value=%d)", last_remote_channels_[4]);
                    }
                    last_ch5_state_ = current_ch5_state;
                }
            } else {
                use_remote_direct_ = false;
            }
        }

        // Call recording functions outside the mutex lock
        if (should_start_recording) {
            startRecording();
        } else if (should_stop_recording) {
            stopRecording();
        }

        checkRecordingDuration();

        ROS_DEBUG_THROTTLE(1.0, "Received remote data with %zu channels, use_remote_direct=%d",
                           msg->channels.size(), use_remote_direct_);
    }

    void controlCallback(const subtask::ControlData::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!use_remote_direct_) {
            last_control_data_ = *msg;
            last_control_time_ = ros::Time::now();
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

            bool remote_valid = (last_remote_time_ != ros::Time(0)) &&
                                (now - last_remote_time_ < ros::Duration(remote_timeout_));
            bool control_valid = (last_control_time_ != ros::Time(0)) &&
                                 (now - last_control_time_ < ros::Duration(control_timeout_));

            if (!remote_valid) {
                use_remote_direct_ = false;
            }

            if (remote_valid && use_remote_direct_) {
                size_t channels_to_copy = std::min(last_remote_channels_.size(), static_cast<size_t>(max_channels_));
                cmd_msg.channels.assign(max_channels_, 1500);
                for (size_t i = 0; i < channels_to_copy; ++i) {
                    cmd_msg.channels[i] = last_remote_channels_[i];
                }
                ROS_DEBUG_THROTTLE(1.0, "Using remote direct mode (remote valid)");
            }
            else if (control_valid) {
                cmd_msg.channels.assign(max_channels_, 1500);
                cmd_msg.channels[0] = static_cast<uint16_t>(last_control_data_.roll);
                cmd_msg.channels[1] = static_cast<uint16_t>(last_control_data_.pitch);
                cmd_msg.channels[2] = static_cast<uint16_t>(last_control_data_.throttle);
                cmd_msg.channels[3] = static_cast<uint16_t>(last_control_data_.yaw);
                cmd_msg.channels[4] = 1000;
                cmd_msg.channels[5] = 1000;
                cmd_msg.channels[6] = 1000;
                cmd_msg.channels[7] = 1000;
                
                cmd_msg.channels[9] = 1500;
                cmd_msg.channels[10] = 1500;
                ROS_DEBUG_THROTTLE(1.0, "Using control data (control valid)");
            }
            else if (remote_valid) {
                size_t channels_to_copy = std::min(last_remote_channels_.size(), static_cast<size_t>(max_channels_));
                cmd_msg.channels.assign(max_channels_, 1500);
                for (size_t i = 0; i < channels_to_copy; ++i) {
                    cmd_msg.channels[i] = last_remote_channels_[i];
                }
                ROS_DEBUG_THROTTLE(1.0, "Using remote (default)");
            }
            else {
                cmd_msg.channels.assign(max_channels_, 1500);
                cmd_msg.channels[2] = 1000;
                ROS_DEBUG_THROTTLE(1.0, "Using default safe values (no valid data)");
            }

            if (remote_valid && use_remote_direct_ && last_remote_channels_.size() >= 10) {
                uint16_t ch9 = last_remote_channels_[8];
                uint16_t ch10 = last_remote_channels_[9];

                float roll = (ch9 - 1500.0f) / 500.0f * gimbal_angle_range_;
                float yaw   = (ch10 - 1500.0f) / 500.0f * gimbal_angle_range_;

                gimbal_control_serial::GimbalCmd gimbal_msg;
                gimbal_msg.roll  = roll;
                gimbal_msg.pitch = 0.0f;
                gimbal_msg.yaw   = yaw;
                gimbal_msg.mode  = 0;

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
    double remote_timeout_;
    double control_timeout_;
    ros::Publisher channel_pub_;
    ros::Subscriber remote_sub_;
    ros::Subscriber control_sub_;
    ros::Timer timer_;

    std::vector<uint16_t> last_remote_channels_;
    subtask::ControlData last_control_data_;
    bool use_control_data_;
    bool use_remote_direct_;
    ros::Time last_remote_time_;
    ros::Time last_control_time_;
    std::mutex mutex_;

    ros::Publisher gimbal_pub_;
    double gimbal_angle_range_;

    ros::Subscriber imu_sub_;
    ros::Subscriber altitude_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber motor_sub_;
    ros::Subscriber status_sub_;
    ros::Subscriber battery_sub_;
    ros::Subscriber esc_sub_;
    ros::Subscriber attitude_sub_;
    ros::Subscriber msp_channel_sub_;

    rosbag::Bag bag_;
    std::mutex bag_mutex_;
    bool is_recording_;
    bool last_ch5_state_;
    ros::Time record_start_time_;
    std::string bag_save_dir_;
    int max_bag_files_;
    double max_record_duration_;
    int ch5_threshold_;
    bool auto_restart_on_timeout_;
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
