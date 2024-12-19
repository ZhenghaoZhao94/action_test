#include "action_server.hpp"
#include <thread>

ActionServer::ActionServer()
: node_(std::make_shared<rclcpp::Node>("action_server_node"))
{
  // 创建Action服务端
  action_server_ = rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
    node_,
    "fibonacci",
    std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));
}

void ActionServer::spin()
{
  rclcpp::spin(node_);
}

// 处理目标请求
rclcpp_action::GoalResponse ActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(), "Received goal request with order %d", goal->order);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理取消请求
rclcpp_action::CancelResponse ActionServer::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 处理被接受的目标（执行目标）
void ActionServer::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle)
{
  std::thread{std::bind(&ActionServer::execute, this, goal_handle)}.detach();
}

// 实际的执行函数
void ActionServer::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<example_interfaces::action::Fibonacci::Feedback>();
  auto & sequence = feedback->sequence;
  sequence.push_back(0);
  sequence.push_back(1);

  auto result = std::make_shared<example_interfaces::action::Fibonacci::Result>();

  rclcpp::Rate loop_rate(1);  // 每秒发送一次反馈
  for (int i = 2; i < goal->order && rclcpp::ok(); ++i) {
    // 检查是否被取消
    if (goal_handle->is_canceling()) {
      result->sequence = sequence;
      goal_handle->canceled(result);
      RCLCPP_INFO(node_->get_logger(), "Goal canceled");
      return;
    }

    // 计算下一个Fibonacci数
    sequence.push_back(sequence[i - 1] + sequence[i - 2]);

    // 发布反馈
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(node_->get_logger(), "Publish feedback: %ld", sequence.back());

    loop_rate.sleep();
  }

  // 检查目标是否完成
  if (rclcpp::ok()) {
    result->sequence = sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<ActionServer>();
  action_server->spin();
  rclcpp::shutdown();
  return 0;
}