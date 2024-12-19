#ifndef ACTION_SERVER_PKG__ACTION_SERVER_HPP_
#define ACTION_SERVER_PKG__ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>

class ActionServer
{
public:
  ActionServer();
  void spin();  // 启动spin循环

private:
  // 处理接收到的目标请求
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal);

  // 处理取消请求
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);

  // 处理目标的执行
  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);

  // 实际的执行函数，运行在一个分离的线程中
  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);

  std::shared_ptr<rclcpp::Node> node_;  // ROS 2 节点
  rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr action_server_;  // Action服务端
};

#endif  // ACTION_SERVER_PKG__ACTION_SERVER_HPP_