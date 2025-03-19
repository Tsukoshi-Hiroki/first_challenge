// Copyright 2024 amsl

#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"
#include <functional>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

class FirstChallenge : public rclcpp::Node {
public:
  FirstChallenge();
  int hz_ = 10;

private:
  // 関数
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void can_move(std::optional<nav_msgs::msg::Odometry> odom_);
  void is_goal(std::optional<nav_msgs::msg::Odometry> odom_);
  void set_cmd_vel();
  double calc_dist();
  void run(double velocity, double omega);

  // pub sub
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr
      cmd_vel_pub_;

  // 変数
  std::optional<nav_msgs::msg::Odometry> odom_;
  roomba_500driver_meiji::msg::RoombaCtrl cmd_vel_;
  double goal_dist = 1.0;
  double velocity = 0.0;
  double omega = 0.0;
};

FirstChallenge::FirstChallenge() : Node("first_challenge") {
  hz_ = this->declare_parameter<int>("hz", 10);  // hz_のデフォルト値を10に設定

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(1).reliable(),
      std::bind(&FirstChallenge::odom_callback, this, std::placeholders::_1));
  cmd_vel_pub_ =
      this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>(
          "/roomba/control", rclcpp::QoS(1).reliable());
}

void FirstChallenge::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_ = *msg;
  can_move(odom_);
}

void FirstChallenge::can_move(std::optional<nav_msgs::msg::Odometry> odom_) {
  if (odom_.has_value()) {
    set_cmd_vel();
  }
}

void FirstChallenge::set_cmd_vel() {
  if (calc_dist() >= goal_dist) {
    velocity = 0.0;
    omega = 0.0;
    run(velocity, omega);
  } else {
    velocity = 0.1;
    omega = 0.0;
    run(velocity, omega);
  }
}

double FirstChallenge::calc_dist() {
  return hypot(odom_.value().pose.pose.position.x,
               odom_.value().pose.pose.position.y);
}

void FirstChallenge::run(double velocity, double omega) {
  RCLCPP_INFO(this->get_logger(), "run");

  cmd_vel_.mode = 11;
  cmd_vel_.cntl.linear.x = velocity;
  cmd_vel_.cntl.angular.z = omega;

  cmd_vel_pub_->publish(cmd_vel_);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<FirstChallenge> fchallenge =
      std::make_shared<FirstChallenge>();
  rclcpp::spin(fchallenge);
  rclcpp::shutdown();

  return 0;
}
