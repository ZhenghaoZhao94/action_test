#include "action_client.hpp"

ActionClient::ActionClient()
: node_(std::make_shared<rclcpp::Node>("action_client_node"))
{
  // 订阅两个话题，分别控制取消和调用Action
  subscription_topic1_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/topic1", 10, std::bind(&ActionClient::topic1_callback, this, std::placeholders::_1));

  subscription_topic2_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/topic2", 10, std::bind(&ActionClient::topic2_callback, this, std::placeholders::_1));

  // 创建Action客户端
  action_client_ = rclcpp_action::create_client<example_interfaces::action::Fibonacci>(node_, "fibonacci");

  // 等待Action服务可用
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
  }
}

void ActionClient::spin()
{
  rclcpp::spin(node_);
}

void ActionClient::topic1_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    if (goal_handle_) {
      RCLCPP_INFO(node_->get_logger(), "Cancelling current action...");
      auto cancel_result_future = action_client_->async_cancel_goal(goal_handle_);
      if (rclcpp::spin_until_future_complete(node_, cancel_result_future) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Action successfully cancelled");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to cancel action");
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "No active goal to cancel");
    }
  }
}

void ActionClient::topic2_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    send_goal();
  }
}

void ActionClient::send_goal()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available");
    return;
  }

  auto goal_msg = example_interfaces::action::Fibonacci::Goal();
  goal_msg.order = 10;  // Fibonacci序列的长度

  RCLCPP_INFO(node_->get_logger(), "Sending goal...");

  auto send_goal_options = rclcpp_action::Client<example_interfaces::action::Fibonacci>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ActionClient::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ActionClient::result_callback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionClient::feedback_callback(rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::SharedPtr,
                                     const std::shared_ptr<const example_interfaces::action::Fibonacci::Feedback> feedback)
{
//   RCLCPP_INFO(node_->get_logger(), "Next number in sequence: %ld", feedback->partial_sequence.back());
    RCLCPP_INFO(node_->get_logger(), "Next number in sequence: %ld", feedback->sequence.back());
}

void ActionClient::result_callback(const rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Result received");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(node_->get_logger(), "Final sequence: ");
  for (auto number : result.result->sequence) {
    RCLCPP_INFO(node_->get_logger(), "%ld", number);
  }

  goal_handle_.reset();
}

void ActionClient::goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>> goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by the action server");
    goal_handle_ = goal_handle;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<ActionClient>();
  action_client->spin();
  rclcpp::shutdown();
  return 0;
}