#include <minimal_action/minimal_action_client_class.hpp>

using namespace std;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Create an instance of the MinimalActionClient()
    auto action_client = std::make_shared<MinimalActionClient>();

    // Create an executor and put the action_client inside
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_client);
    // Send the goal
    action_client->send_goal(10);

    // Check if the action was successful and kill the node once finished
    executor.spin();

    rclcpp::shutdown();
    return 0;
}