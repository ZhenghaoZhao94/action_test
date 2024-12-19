#ifndef ACTION_CLIENT_PKG__ACTION_CLIENT_HPP_
#define ACTION_CLIENT_PKG__ACTION_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <std_msgs/msg/bool.hpp>

class ActionClient
{
public:
  ActionClient();
  void spin();  // 启动spin循环

private:
  void topic1_callback(const std_msgs::msg::Bool::SharedPtr msg);  // 取消callback
  void topic2_callback(const std_msgs::msg::Bool::SharedPtr msg);  // 发送action callback
  void send_goal();  // 发送action目标

  void feedback_callback(rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::SharedPtr,
                         const std::shared_ptr<const example_interfaces::action::Fibonacci::Feedback> feedback);
  void result_callback(const rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::WrappedResult & result);
  void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);

  std::shared_ptr<rclcpp::Node> node_;  // ROS 2 节点
  rclcpp_action::Client<example_interfaces::action::Fibonacci>::SharedPtr action_client_;  // Action客户端
  rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::SharedPtr goal_handle_;  // 活跃goal handle

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_topic1_;  // 订阅1
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_topic2_;  // 订阅2
};

#endif  // ACTION_CLIENT_PKG__ACTION_CLIENT_HPP_