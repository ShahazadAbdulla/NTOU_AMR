#include "ntou_hardware_interface/wheel_odometry.hpp"

#include <cmath>

namespace ntou_hardware_interface
{

WheelOdometry::WheelOdometry(rclcpp::Node & node, const Config & config)
: node_(node), config_(config)
{
  pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(config_.topic, 10);
}

void WheelOdometry::setWheelRotations(const std::array<double, 4> & rot_rev)
{
  current_rot_ = rot_rev;
}

void WheelOdometry::setSteeringRad(double steer_rad)
{
  current_steer_rad_ = steer_rad;
}

void WheelOdometry::publish(const rclcpp::Time & now)
{
  if (!has_prev_) {
    prev_time_ = now;
    prev_rot_ = current_rot_;
    has_prev_ = true;
    return;
  }

  const double dt = (now - prev_time_).seconds();
  if (dt <= 1e-4) {
    return;
  }

  double sum_d_m = 0.0;
  for (size_t i = 0; i < current_rot_.size(); ++i) {
    const double d_rot = current_rot_[i] - prev_rot_[i];
    sum_d_m += d_rot * (2.0 * M_PI * config_.wheel_radius) * config_.odom_scale;
    prev_rot_[i] = current_rot_[i];
  }
  prev_time_ = now;

  const double d_step = sum_d_m / static_cast<double>(current_rot_.size());
  vx_ = d_step / dt;

  x_ += d_step * std::cos(yaw_);
  y_ += d_step * std::sin(yaw_);

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = now;
  msg.header.frame_id = config_.odom_frame;
  msg.child_frame_id = config_.base_frame;

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0.0;

  const double half_yaw = 0.5 * yaw_;
  msg.pose.pose.orientation.z = std::sin(half_yaw);
  msg.pose.pose.orientation.w = std::cos(half_yaw);

  msg.twist.twist.linear.x = vx_;
  msg.twist.twist.angular.z = vyaw_;

  msg.pose.covariance[0] = 0.05;
  msg.pose.covariance[7] = 0.05;
  msg.pose.covariance[35] = 0.1;
  msg.twist.covariance[0] = 0.02;
  msg.twist.covariance[35] = 0.1;

  pub_->publish(msg);
}

}  // namespace ntou_hardware_interface
