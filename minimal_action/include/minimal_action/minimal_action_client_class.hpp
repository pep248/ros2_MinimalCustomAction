#ifndef MINIMAL_ACTION_CLIENT_HPP_
#define MINIMAL_ACTION_CLIENT_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_action/action/custom_action.hpp"


class MinimalActionClient : public rclcpp::Node
{
public:
  // CustomAction class
  using CustomAction = custom_action::action::CustomAction;
  // GoalHandle class (handles the accept, the cancel, and the execute functions)
  using GoalHandle = rclcpp_action::ClientGoalHandle<CustomAction>;

  explicit MinimalActionClient(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());

  void send_goal(int objective);

private:
  rclcpp_action::Client<CustomAction>::SharedPtr client_ptr_;

  // Response Callback
  void goal_response_callback(GoalHandle::SharedPtr goal_message);
  // Feedback Callback
  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const CustomAction::Feedback> feedback_message);
  // Result Callback
  void result_callback(const GoalHandle::WrappedResult &result_message);
};

#endif  // MINIMAL_ACTION_CLIENT_HPP_
