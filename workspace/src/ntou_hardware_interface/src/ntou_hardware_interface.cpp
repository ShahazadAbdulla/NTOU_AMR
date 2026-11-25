#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kJointCount = 6U;
}

class NtouHardwareInterface final : public rclcpp::Node
{
public:
    NtouHardwareInterface()
    : Node("ntou_hardware_interface"),
      serialFd_(-1),
      lineOverflow_(false),
      lastDrivePwm_(std::numeric_limits<int>::min()),
      lastSteerDegrees_(std::numeric_limits<int>::min()),
      timeoutStopSent_(false),
      serialConnected_(false),
      haveAllJointSamples_(false)
    {
        serialPort_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        const int baud = declare_parameter<int>("baud_rate", 115200);
        maxLinearInput_ = declare_parameter<double>("max_linear_x_mps", 0.5);
        maxPwmPercent_ = declare_parameter<int>("max_pwm_percent", 50);
        maxAngularInput_ = declare_parameter<double>("max_angular_z_rad_s", 2.0);
        maxSteeringDegrees_ = declare_parameter<int>("max_steering_degrees", 40);
        cmdTimeoutSeconds_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.5);
        wheelJointNames_ = declare_parameter<std::vector<std::string>>(
            "wheel_joint_names",
            {"front_left_wheel_joint", "front_right_wheel_joint",
             "rear_right_wheel_joint", "rear_left_wheel_joint"});
        steeringJointNames_ = declare_parameter<std::vector<std::string>>(
            "steering_joint_names", {"steering_right_joint", "steering_left_joint"});

        if (baud != 115200) {
            throw std::invalid_argument("This firmware UART is fixed at 115200 baud.");
        }
        if (maxLinearInput_ <= 0.0 || maxAngularInput_ <= 0.0 ||
            maxPwmPercent_ < 1 || maxPwmPercent_ > 100 ||
            maxSteeringDegrees_ < 1 || maxSteeringDegrees_ > 40 ||
            cmdTimeoutSeconds_ <= 0.0 || wheelJointNames_.size() != 4U ||
            steeringJointNames_.size() != 2U) {
            throw std::invalid_argument("Invalid hardware-interface parameter values.");
        }

        jointNames_.reserve(kJointCount);
        jointNames_.insert(jointNames_.end(), wheelJointNames_.begin(), wheelJointNames_.end());
        jointNames_.insert(jointNames_.end(), steeringJointNames_.begin(), steeringJointNames_.end());

        jointStatePub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        serialFeedbackPub_ = create_publisher<std_msgs::msg::String>("/hardware/serial_feedback", 20);
        serialConnectedPub_ = create_publisher<std_msgs::msg::Bool>("/hardware/serial_connected", 10);
        cmdVelSub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&NtouHardwareInterface::cmdVelCallback, this, std::placeholders::_1));

        lastCmdTime_ = std::chrono::steady_clock::now();
        nextReconnectTime_ = std::chrono::steady_clock::now();
        openSerial();
        if (serialFd_ >= 0) {
            queueSerialLine("STOP");
            flushSerialWrites();
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&NtouHardwareInterface::timerCallback, this));

        RCLCPP_INFO(get_logger(), "NTOU serial bridge started. Port=%s baud=115200", serialPort_.c_str());
        RCLCPP_INFO(get_logger(), "Subscribing /cmd_vel; publishing /joint_states and /hardware/serial_feedback");
        RCLCPP_WARN(get_logger(),
            "Drive PWM is open-loop. Encoder distance scale is still provisional; no calibrated /odom is published.");
    }

    ~NtouHardwareInterface() override
    {
        if (serialFd_ >= 0) {
            queueSerialLine("STOP");
            flushSerialWrites();
            ::close(serialFd_);
            serialFd_ = -1;
        }
    }

private:
    void setSerialConnected(bool connected)
    {
        if (serialConnected_ == connected) return;
        serialConnected_ = connected;
        std_msgs::msg::Bool msg;
        msg.data = connected;
        serialConnectedPub_->publish(msg);
        if (connected) {
            RCLCPP_INFO(get_logger(), "Serial connected: %s", serialPort_.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Serial disconnected: %s", serialPort_.c_str());
        }
    }

    bool openSerial()
    {
        if (serialFd_ >= 0) return true;
        const auto now = std::chrono::steady_clock::now();
        if (now < nextReconnectTime_) return false;
        nextReconnectTime_ = now + std::chrono::seconds(1);

        const int fd = ::open(serialPort_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            if (!reportedOpenFailure_) {
                RCLCPP_WARN(get_logger(), "Cannot open %s: %s", serialPort_.c_str(), std::strerror(errno));
                reportedOpenFailure_ = true;
            }
            setSerialConnected(false);
            return false;
        }

        termios tty{};
        if (tcgetattr(fd, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcgetattr(%s) failed: %s", serialPort_.c_str(), std::strerror(errno));
            ::close(fd);
            return false;
        }

        cfmakeraw(&tty);
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tty.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CSIZE));
#ifdef CRTSCTS
        tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
        tty.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
        tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcsetattr(%s) failed: %s", serialPort_.c_str(), std::strerror(errno));
            ::close(fd);
            return false;
        }
        tcflush(fd, TCIOFLUSH);

        serialFd_ = fd;
        serialLineBuffer_.clear();
        pendingTx_.clear();
        reportedOpenFailure_ = false;
        setSerialConnected(true);
        queueSerialLine("STOP");
        flushSerialWrites();
        lastDrivePwm_ = 0;
        lastSteerDegrees_ = 0;
        timeoutStopSent_ = false;
        return true;
    }

    void closeSerial()
    {
        if (serialFd_ >= 0) {
            ::close(serialFd_);
            serialFd_ = -1;
        }
        pendingTx_.clear();
        serialLineBuffer_.clear();
        setSerialConnected(false);
        nextReconnectTime_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    }

    void queueSerialLine(const std::string &line)
    {
        if (serialFd_ < 0) return;
        RCLCPP_INFO(get_logger(), "Serial command queued: [%s]", line.c_str());
        pendingTx_ += line;
        pendingTx_ += '\n';
    }

    void flushSerialWrites()
    {
        if (serialFd_ < 0) return;
        while (!pendingTx_.empty()) {
            const ssize_t written = ::write(serialFd_, pendingTx_.data(), pendingTx_.size());
            if (written > 0) {
            	RCLCPP_INFO(
            	    get_logger(), "Serial write: %zd bytes: [%.*s]",
            	    written, static_cast<int>(written), pendingTx_.data());
                pendingTx_.erase(0U, static_cast<std::size_t>(written));
                continue;
            }
            if (written == 0) return;
            if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return;
            RCLCPP_ERROR(get_logger(), "Serial write failed: %s", std::strerror(errno));
            closeSerial();
            return;
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double linearRatio = std::clamp(msg->linear.x / maxLinearInput_, -1.0, 1.0);
        const int drivePwm = static_cast<int>(std::lround(linearRatio * maxPwmPercent_));

        // ROS convention: positive angular.z is a left/CCW turn.
        // NTOU firmware convention: positive steering degrees means RIGHT.
        const double angularRatio = std::clamp(msg->angular.z / maxAngularInput_, -1.0, 1.0);
        const int steeringDegrees = static_cast<int>(std::lround(-angularRatio * maxSteeringDegrees_));

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "CMD RX: linear=%.3f angular=%.3f -> drive=%d steer=%d",
            msg->linear.x, msg->angular.z, drivePwm, steeringDegrees);

        lastCmdTime_ = std::chrono::steady_clock::now();
        timeoutStopSent_ = false;

        if (serialFd_ < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No serial link; ignoring /cmd_vel command.");
            return;
        }

        if (drivePwm != lastDrivePwm_) {
            std::ostringstream command;
            command << "DRIVE " << drivePwm << ' ' << drivePwm << ' ' << drivePwm << ' ' << drivePwm;
            queueSerialLine(command.str());
            lastDrivePwm_ = drivePwm;
        }
        if (steeringDegrees != lastSteerDegrees_) {
            queueSerialLine("STEER " + std::to_string(steeringDegrees));
            lastSteerDegrees_ = steeringDegrees;
        }
    }

    static bool parseDouble(const std::string &text, double &value)
    {
        try {
            std::size_t used = 0U;
            value = std::stod(text, &used);
            return used == text.size() && std::isfinite(value);
        } catch (...) {
            return false;
        }
    }

    static bool parseKeyValue(const std::string &token, std::string &key, std::string &value)
    {
        const std::size_t equals = token.find('=');
        if (equals == std::string::npos || equals == 0U || equals + 1U >= token.size()) return false;
        key = token.substr(0U, equals);
        value = token.substr(equals + 1U);
        return true;
    }

    void updateJoint(std::size_t index, double position)
    {
        if (index >= kJointCount || !std::isfinite(position)) return;
        const auto now = std::chrono::steady_clock::now();
        if (jointSampleValid_[index]) {
            const double dt = std::chrono::duration<double>(now - jointSampleTime_[index]).count();
            if (dt > 1e-4) jointVelocity_[index] = (position - jointPosition_[index]) / dt;
        } else {
            jointVelocity_[index] = 0.0;
            jointSampleValid_[index] = true;
        }
        jointPosition_[index] = position;
        jointSampleTime_[index] = now;
        haveAllJointSamples_ = std::all_of(
            jointSampleValid_.begin(), jointSampleValid_.end(), [](bool valid) { return valid; });
    }

    void parseTelemetryLine(const std::string &line)
    {
        if (line.empty()) return;
        std_msgs::msg::String raw;
        raw.data = line;
        serialFeedbackPub_->publish(raw);

        std::istringstream stream(line);
        std::string first;
        stream >> first;
        std::size_t jointIndex = kJointCount;
        bool steeringLine = false;

        if (first == "FL") jointIndex = 0U;
        else if (first == "FR") jointIndex = 1U;
        else if (first == "RR") jointIndex = 2U;
        else if (first == "RL") jointIndex = 3U;
        else if (first == "SR" || first == "SL") {
            steeringLine = true;
            jointIndex = (first == "SR") ? 4U : 5U;
            std::string channel;
            stream >> channel;  // CH0 or CH1
        } else {
            return;
        }

        std::string token;
        while (stream >> token) {
            std::string key, value;
            if (!parseKeyValue(token, key, value)) continue;
            double parsed = 0.0;
            if (steeringLine && key == "pos" && parseDouble(value, parsed)) {
                updateJoint(jointIndex, parsed * kPi / 180.0);
            } else if (!steeringLine && key == "rot" && parseDouble(value, parsed)) {
                updateJoint(jointIndex, parsed * 2.0 * kPi);
            }
        }
    }

    void readSerial()
    {
        if (serialFd_ < 0) return;
        char buffer[256];
        for (;;) {
            const ssize_t count = ::read(serialFd_, buffer, sizeof(buffer));
            if (count > 0) {
                for (ssize_t i = 0; i < count; ++i) {
                    const char ch = buffer[i];
                    if (ch == '\n') {
                        if (!lineOverflow_) {
                            if (!serialLineBuffer_.empty() && serialLineBuffer_.back() == '\r') {
                                serialLineBuffer_.pop_back();
                            }
                            parseTelemetryLine(serialLineBuffer_);
                        }
                        serialLineBuffer_.clear();
                        lineOverflow_ = false;
                    } else if (!lineOverflow_) {
                        if (serialLineBuffer_.size() < 511U) serialLineBuffer_.push_back(ch);
                        else lineOverflow_ = true;
                    }
                }
                continue;
            }
            if (count == 0) return;
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
            RCLCPP_ERROR(get_logger(), "Serial read failed: %s", std::strerror(errno));
            closeSerial();
            return;
        }
    }

    void publishJointStates()
    {
        if (!haveAllJointSamples_) return;
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = jointNames_;
        msg.position.assign(jointPosition_.begin(), jointPosition_.end());
        msg.velocity.assign(jointVelocity_.begin(), jointVelocity_.end());
        jointStatePub_->publish(msg);
    }

    void timerCallback()
    {
        if (serialFd_ < 0) openSerial();
        if (serialFd_ >= 0) {
            readSerial();
            flushSerialWrites();
        }

        const auto nowSteady = std::chrono::steady_clock::now();
        const double commandAge = std::chrono::duration<double>(nowSteady - lastCmdTime_).count();
        if (serialFd_ >= 0 && !timeoutStopSent_ && commandAge > cmdTimeoutSeconds_) {
            queueSerialLine("DSTOP");
            lastDrivePwm_ = 0;
            timeoutStopSent_ = true;
            RCLCPP_WARN(get_logger(), "cmd_vel timeout: drive motors commanded to stop; steering hold retained.");
        }
        if (serialFd_ >= 0) flushSerialWrites();
        publishJointStates();
    }

    std::string serialPort_;
    int serialFd_;
    bool reportedOpenFailure_{false};
    std::chrono::steady_clock::time_point nextReconnectTime_{};
    std::string serialLineBuffer_;
    bool lineOverflow_;
    std::string pendingTx_;

    double maxLinearInput_{0.5};
    int maxPwmPercent_{50};
    double maxAngularInput_{2.0};
    int maxSteeringDegrees_{40};
    double cmdTimeoutSeconds_{0.5};
    std::vector<std::string> wheelJointNames_;
    std::vector<std::string> steeringJointNames_;
    std::vector<std::string> jointNames_;

    int lastDrivePwm_;
    int lastSteerDegrees_;
    bool timeoutStopSent_;
    bool serialConnected_;
    std::chrono::steady_clock::time_point lastCmdTime_{};

    std::array<double, kJointCount> jointPosition_{};
    std::array<double, kJointCount> jointVelocity_{};
    std::array<bool, kJointCount> jointSampleValid_{};
    std::array<std::chrono::steady_clock::time_point, kJointCount> jointSampleTime_{};
    bool haveAllJointSamples_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serialFeedbackPub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr serialConnectedPub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<NtouHardwareInterface>());
    } catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("ntou_hardware_interface"), "Startup failed: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
