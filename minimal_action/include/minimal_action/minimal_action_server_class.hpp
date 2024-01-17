#ifndef MINIMAL_ACTION_SERVER_HPP_
#define MINIMAL_ACTION_SERVER_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_action/action/custom_action.hpp"

class MinimalActionServer : public rclcpp::Node
{
public:
  // CustomAction class
  using CustomAction = custom_action::action::CustomAction;
  // GoalHandle class (handles the accept, the cancel, and the execute functions)
  using GoalHandle = rclcpp_action::ServerGoalHandle<CustomAction>;

  explicit MinimalActionServer(const rclcpp::NodeOptions &action_server_options = rclcpp::NodeOptions());

private:
  // Action Server
  rclcpp_action::Server<CustomAction>::SharedPtr action_server_;

  // Handle Goal Request
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const CustomAction::Goal> goal_request);

  // Handle cancel
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle_canceled_);

  // Handle Accepted
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle_accepted_);

  // Execute
  void execute(const std::shared_ptr<GoalHandle> goal_handle_);
};

#endif  // MINIMAL_ACTION_SERVER_HPP_
