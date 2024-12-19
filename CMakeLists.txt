cmake_minimum_required(VERSION 3.8)
project(action_client_pkg)

# 找到ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

# 包含目录
include_directories(include)

# 编译C++文件
add_executable(action_client_node src/action_client.cpp)

# 链接依赖
ament_target_dependencies(action_client_node rclcpp rclcpp_action std_msgs example_interfaces)

# 安装可执行文件
install(TARGETS 
  action_client_node
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# 编译C++文件
add_executable(action_server_node src/action_server.cpp)

# 链接依赖
ament_target_dependencies(action_server_node rclcpp rclcpp_action example_interfaces)

# 安装可执行文件
install(TARGETS 
  action_server_node
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()