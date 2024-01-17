#include <minimal_action/minimal_action_server_class.hpp>

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