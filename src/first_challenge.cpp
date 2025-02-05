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
  int hz_ = 10;

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pub_cmd_vel(bool can_move);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; 
  rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr cmd_vel_pub_;

  bool can_move_ = false;
  roomba_500driver_meiji::msg::RoombaCtrl cmd_vel_;

};

FirstChallenge::FirstChallenge() : Node("first_challenge")
{
  hz_ = this->declare_parameter<int>("hz", 10);  // hz_のデフォルト値を10に設定

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(1).reliable(), std::bind(&FirstChallenge::odom_callback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("/roomba/control", rclcpp::QoS(1).reliable());
}

void FirstChallenge::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

  RCLCPP_INFO(this->get_logger(),"odom_callback");
  if(msg != NULL)
  {
    RCLCPP_INFO(this->get_logger(),"sub odom");
    can_move_ = true;
  }
  else{
    RCLCPP_INFO(this->get_logger(),"none sub odom");
  }

  pub_cmd_vel(can_move_);
}


void FirstChallenge::pub_cmd_vel(bool can_move)
{
  RCLCPP_INFO(this->get_logger(),"determine_cmd_vel");
  if(can_move_)
  {
    RCLCPP_INFO(this->get_logger(),"pub_cmd_vel");

    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.1;
    cmd_vel_.cntl.angular.z = 0;

    cmd_vel_pub_->publish(cmd_vel_);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<FirstChallenge> fchallenge = std::make_shared<FirstChallenge>();
  rclcpp::Rate loop_rate(fchallenge->hz_);

  while(rclcpp::ok())
  {
    rclcpp::spin_some(fchallenge);
    loop_rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}
