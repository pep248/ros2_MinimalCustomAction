#include <minimal_action/minimal_action_client_class.hpp>

using namespace std;

MinimalActionClient::MinimalActionClient(const rclcpp::NodeOptions &node_options)
  : Node("minimal_action_client", node_options)
{
  this->client_ptr_ = rclcpp_action::create_client<CustomAction>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "/custom_action"); // "topic" where to request the action
}

void MinimalActionClient::send_goal(int objective)
{
  // Check if action server is available
  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Client: Action server not available after waiting");
    rclcpp::shutdown();
    return;
  }

  // Bind the actual goal, feedback, and results with our desired callbacks
  auto send_goal_options = rclcpp_action::Client<CustomAction>::SendGoalOptions();
  using namespace std::placeholders;
  send_goal_options.goal_response_callback =
    std::bind(&MinimalActionClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&MinimalActionClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&MinimalActionClient::result_callback, this, _1);
  
  // create the goal message
  auto goal_msg = CustomAction::Goal();
  goal_msg.goal = objective;
  // Send the actual goal
  RCLCPP_INFO(this->get_logger(), "Client: Sending goal");
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

// Response Callback
void MinimalActionClient::goal_response_callback(GoalHandle::SharedPtr goal_message)
{
  if (!goal_message)
  {
    RCLCPP_ERROR(this->get_logger(), "Client: Goal was rejected by server");
    rclcpp::shutdown(); // Shut down client node
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Client: Goal accepted by server, waiting for result");
  }
}

// Feedback Callback
void MinimalActionClient::feedback_callback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const CustomAction::Feedback> feedback_message)
{
  // Load the array of numbers into a string and print it
  std::stringstream ss;
  ss << "Client: Current sequence: ";
  for (auto number : feedback_message->sequence)
  {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

// Result Callback
void MinimalActionClient::result_callback(const GoalHandle::WrappedResult &result_message)
{
  switch (result_message.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Client: Goal was aborted");
      rclcpp::shutdown(); // Shut down client node
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Client: Goal was canceled");
      rclcpp::shutdown(); // Shut down client node
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Client: Unknown result code");
      rclcpp::shutdown(); // Shut down client node
      return;
  }
  RCLCPP_INFO(this->get_logger(), "Client: Result received: %s", (result_message.result->succeeded ? "true" : "false"));
  rclcpp::shutdown(); // Shut down client node
}
