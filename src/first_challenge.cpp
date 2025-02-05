// Copyright 2024 amsl

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <nav_msgs/msg/odometry.hpp>
#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"

class FirstChallenge : public rclcpp::Node
{
public:
  FirstChallenge();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pub_cmd_vel(bool can_move);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; 
  rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr cmd_vel_pub_;

  bool can_move_ = false;
  roomba_500driver_meiji::msg::RoombaCtrl cmd_vel;

};

FirstChallenge::FirstChallenge() : Node("first_challenge")
{
  // ctrl_pub_ = nh_.advertise<roomba_500driver_meiji::msg::RoombaCtrl>("/roomba/control", 1, true);
  // odom_sub_ = nh_.subscribe("odom", 1, &FirstChallenge::odom_callback, this);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(1).reliable(), std::bind(&FirstChallenge::odom_callback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("/roomba/control", rclcpp::QoS(1).reliable());
}

void FirstChallenge::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if(msg != NULL)
  {
    can_move_ = true;
  }
  pub_cmd_vel(can_move_);
}


void FirstChallenge::pub_cmd_vel(bool can_move)
{
  if(can_move_)
  {
    cmd_vel.velocity = 1;
    cmd_vel_pub_->publish(cmd_vel);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if(rclcpp::ok()){
    std::shared_ptr<FirstChallenge> fchallenge = std::make_shared<FirstChallenge>();
    rclcpp::spin_some(fchallenge);
  }
  return 0;
}
