#pragma once

#include <array>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ntou_hardware_interface
{

class WheelOdometry
{
public:
  struct Config
  {
    double odom_scale = 2.0;
    double wheel_radius = 0.10;
    std::string odom_frame = "odom";
    std::string base_frame = "base_footprint";
    std::string topic = "/odom";
  };

  WheelOdometry(rclcpp::Node & node, const Config & config);

  void setWheelRotations(const std::array<double, 4> & rot_rev);
  void setSteeringRad(double steer_rad);
  void publish(const rclcpp::Time & now);

private:
  rclcpp::Node & node_;
  Config config_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  std::array<double, 4> current_rot_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> prev_rot_{{0.0, 0.0, 0.0, 0.0}};
  double current_steer_rad_{0.0};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double vx_{0.0};
  double vyaw_{0.0};

  bool has_prev_{false};
  rclcpp::Time prev_time_;
};

}  // namespace ntou_hardware_interface
