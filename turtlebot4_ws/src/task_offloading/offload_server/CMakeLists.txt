cmake_minimum_required(VERSION 3.8)
project(offload_server)

# Set Default C++ version at C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Include base ROS2 and Nav2 packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)

find_package(nav2_common REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

# Include Task action interfaces
find_package(task_action_interfaces REQUIRED)

#Build

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/main.cpp"
  "src/offload_server.cpp"
  "src/offload_amcl.cpp"
  "src/scheduler.cpp"
)


set(DEPENDENCIES
  "rclcpp"
  "rclcpp_action"
  "rcutils"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "irobot_create_msgs"
  "nav2_common"
  "nav2_amcl"
  "nav2_costmap_2d"
  "task_action_interfaces"
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXECUTABLE_NAME "offload_server_node")

add_executable(${EXECUTABLE_NAME} src/main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
