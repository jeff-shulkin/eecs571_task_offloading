cmake_minimum_required(VERSION 3.5)
project(offload_agent)

# Set default C++ version at C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Include base ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)

# Include Turtlebot4 dependencies
find_package(turtlebot4_description REQUIRED)
find_package(turtlebot4_msgs REQUIRED)
find_package(turtlebot4_navigation REQUIRED)
find_package(turtlebot4_node REQUIRED)

# Include defined task action interfaces
find_package(task_action_interfaces REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/main.cpp"
  "src/offload_agent.cpp"
  "src/turtlebot4_functions.cpp"
)

set(DEPENDENCIES
  "rclcpp"
  "rclcpp_action"
  "rcutils"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "irobot_create_msgs"
  "turtlebot4_description"
  "turtlebot4_msgs"
  "turtlebot4_navigation"
  "turtlebot4_node"
  "task_action_interfaces"
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})


set(EXECUTABLE_NAME "offload_agent")


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
