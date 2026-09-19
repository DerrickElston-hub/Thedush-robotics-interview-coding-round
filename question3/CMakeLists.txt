cmake_minimum_required(VERSION 3.8)

project(ros2_q3)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------
# Dependencies
# ---------------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

find_package(action_msgs REQUIRED)

# ---------------------------------------------------------
# Custom interfaces
# ---------------------------------------------------------

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToCharger.srv"
  "action/GoToCharger.action"
)

# ---------------------------------------------------------
# Q3 - Service Server
# ---------------------------------------------------------

add_executable(service_server
  src/service_server.cpp
)

ament_target_dependencies(service_server
  rclcpp
)

# ---------------------------------------------------------
# Q3 - Service Client
# ---------------------------------------------------------

add_executable(service_client
  src/service_client.cpp
)

ament_target_dependencies(service_client
  rclcpp
)

# ---------------------------------------------------------
# Q3 - Action Server
# ---------------------------------------------------------

add_executable(action_server
  src/action_server.cpp
)

ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
)

# ---------------------------------------------------------
# Q3 - Action Client
# ---------------------------------------------------------

add_executable(action_client
  src/action_client.cpp
)

ament_target_dependencies(action_client
  rclcpp
  rclcpp_action
)

# ---------------------------------------------------------
# Link generated interface typesupport
# ---------------------------------------------------------

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(service_server
  "${cpp_typesupport_target}"
)

target_link_libraries(service_client
  "${cpp_typesupport_target}"
)

target_link_libraries(action_server
  "${cpp_typesupport_target}"
)

target_link_libraries(action_client
  "${cpp_typesupport_target}"
)

# ---------------------------------------------------------
# Install executables
# ---------------------------------------------------------

install(
  TARGETS
    service_server
    service_client
    action_server
    action_client
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------
# Export runtime interface dependency
# ---------------------------------------------------------

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()

