cmake_minimum_required(VERSION 3.8)
project(minimal_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(custom_action REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add the action client executable
ament_auto_add_executable(minimal_action_client src/minimal_action_client.cpp)
# Install the executable
install(TARGETS minimal_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# Add the action server executable
ament_auto_add_executable(minimal_action_server src/minimal_action_server.cpp)
# Install the executable
install(TARGETS minimal_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launchfile
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
