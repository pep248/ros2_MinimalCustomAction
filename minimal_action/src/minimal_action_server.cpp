#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_action/action/customaction.hpp"

using namespace std;

class MinimalActionServer : public rclcpp::Node
{
public:
  // CustomAction class
  using CustomAction = custom_action::action::Customaction;
  // GoalHandle class (handles the accept, the cancel, and the execute functions)
  using GoalHandle = rclcpp_action::ServerGoalHandle<CustomAction>;

  explicit MinimalActionServer(const rclcpp::NodeOptions & action_server_options = rclcpp::NodeOptions())
  : Node("minimal_action_server", action_server_options)
  {
    // Bind the actual goal, cancel and accepted with our desired callbacks (handles)
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<CustomAction>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "/custom_action", // "topic" where to request the action
      std::bind(&MinimalActionServer::handle_goal, this, _1, _2),
      std::bind(&MinimalActionServer::handle_cancel, this, _1),
      std::bind(&MinimalActionServer::handle_accepted, this, _1));
  }

private:
  // Action Server
  rclcpp_action::Server<CustomAction>::SharedPtr action_server_;

  // Handle Goal Request
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CustomAction::Goal> goal_request)
  {
    RCLCPP_INFO(this->get_logger(), "Server: Received goal request: %d", goal_request->goal);
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal_request->goal > 11)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    else
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  }

  // Handle cancel
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle_canceled_)
  {
    RCLCPP_INFO(this->get_logger(), "Server: Received request to cancel action");
    (void) goal_handle_canceled_; //suppress warning due to unused variable (it would compile anyway)
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Handle Accepted
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle_accepted_)
  {
    using namespace std::placeholders;
    // Execute in a thread, the "execute" function
    std::thread{std::bind(&MinimalActionServer::execute, this, _1), goal_handle_accepted_}.detach(); 
  }

  // Execute 
  void execute(const std::shared_ptr<GoalHandle> goal_handle_)
  {
    // Goal
    const auto requested_goal = goal_handle_->get_goal();
    // Feedback
    auto feedback = std::make_shared<CustomAction::Feedback>();
    // Result
    auto result = std::make_shared<CustomAction::Result>();
    
    
    // Let's make the action count up to goal
    RCLCPP_INFO(this->get_logger(), "Server: Executing goal");
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < requested_goal->goal) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      if (goal_handle_->is_canceling())
      {
        result->succeeded = false;
        goal_handle_->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Server: Goal canceled");
        return;
      }

      // Update sequence
      feedback->sequence.push_back(i);
      // Publish feedback
      goal_handle_->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Server: Publish feedback");

      loop_rate.sleep();
    }

    // Goal reached
    if (rclcpp::ok())
    {
      result->succeeded = true;
      goal_handle_->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Server: Goal succeeded");
    }
  }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto action_server = std::make_shared<MinimalActionServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}