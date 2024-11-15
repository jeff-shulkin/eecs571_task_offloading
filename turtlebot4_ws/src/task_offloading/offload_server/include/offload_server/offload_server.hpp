/*
 * Copyright 2021 Clearpath Robotics, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef OFFLOAD_SERVER_NODE__HPP_
#define OFFLOAD_SERVER_NODE__HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_util/service_client.hpp>


#include "irobot_create_msgs/msg/wheel_status.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/wall_follow.hpp"
#include "irobot_create_msgs/action/led_animation.hpp"
#include "irobot_create_msgs/srv/e_stop.hpp"
#include "irobot_create_msgs/srv/robot_power.hpp"

#include "task_action_interfaces/action/offloadlocalization.hpp"
#include "offload_server/scheduler.hpp"
#include "offload_server/utils.hpp"

/** Supported functions
 * Offload Localization
 * Power off
 */

namespace offload_server
{

// Timer Periods
// TODO: DEFINE FURTHER TIMER DEADLINES
static constexpr auto LIDAR_5HZ_TIMER_DEADLINE_MS = 200;
static constexpr auto LIDAR_10HZ_TIMER_DEADLINE_MS = 100;
static constexpr auto WIFI_TIMER_LATENCY_DEADLINE_MS = 70; // TODO: ALTER WIFI DEADLINE BASED ON EXPERIENCE/MAKE VARIABLE
class OffloadServer : public rclcpp::Node
{

public:

  using OffloadLocalization = task_action_interfaces::action::Offloadlocalization;
  using GoalHandleOffloadLocalization = rclcpp_action::ServerGoalHandle<OffloadLocalization>;

  // Constructor and Destructor
  explicit OffloadServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~OffloadServer() {}

private:
  void run();

  // Subscription callbacks
  void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_pose_msg);
  void local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_msg);

  // Function callbacks
  void offload_localization_function_callback();
  void help_function_callback();

  // Run localization deadline timer
  void localization_timer(const std::chrono::milliseconds deadline);

  // Run costmap computation deadline timer
  void costmap_timer(const std::chrono::milliseconds deadline);

  // Run wifi deadline timer
  void wifi_timer(const std::chrono::milliseconds deadline);

  // Run power off timer
  void power_off_timer(const std::chrono::milliseconds timeout);

  // Callback functions for offload localization action server
  rclcpp_action::GoalResponse handle_offload_localization_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const OffloadLocalization::Goal> goal);

  rclcpp_action::CancelResponse handle_offload_localization_cancel(
      const std::shared_ptr<GoalHandleOffloadLocalization> goal_handle);

  void handle_offload_localization_accepted(const std::shared_ptr<GoalHandleOffloadLocalization> goal_handle);

  void offload_localization_execute(const std::shared_ptr<GoalHandleOffloadLocalization> goal_handle);

  // IP
  std::string get_ip();
  std::string wifi_interface_;

  // Node
  rclcpp::Node::SharedPtr node_handle_;

  // Offload Server Functions
  std::map<std::string, offload_server_function_callback_t> function_callbacks_;

  // Action Servers
  rclcpp_action::Server<OffloadLocalization>::SharedPtr offload_localization_action_server_;

  // Timers
  rclcpp::TimerBase::SharedPtr localization_timer_;
  rclcpp::TimerBase::SharedPtr costmap_timer_;
  rclcpp::TimerBase::SharedPtr wifi_timer_;
  rclcpp::TimerBase::SharedPtr power_off_timer_;

  // General Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ip_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr function_call_pub_;

  // Nav2 Publishers for server-side processing
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr nav2_laser_scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr nav2_ipose_pub_;

  // Nav2 Subscribers for server-side data reception
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr nav2_amcl_rpose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr  local_costmap_map_sub_;

  // Nav2 Service for managing localization lifecycle
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr nav2_localization_manager_client_;

  // Store power saver mode
  bool power_saver_;

  // Store latest LiDAR data sent from offload_agent
  sensor_msgs::msg::LaserScan latest_lidar_msg_;

  // Store initial pose sent from offload agent
  geometry_msgs::msg::PoseWithCovarianceStamped offload_amcl_ipose_;

  // Store localization pose received from offload_server
  geometry_msgs::msg::PoseWithCovarianceStamped offload_amcl_rpose_;

  // Store costmap received from offload_server
  nav_msgs::msg::OccupancyGrid offload_local_rcostmap_;

  // Store server scheduling algorithm
  ServerSchedulingAlgo algo_;

  // Task Scheduler
  JobScheduler sched_;
};

}  // namespace offload_server

RCLCPP_COMPONENTS_REGISTER_NODE(offload_server::OffloadServer)

#endif  // OFFLOAD_SERVER_NODE__HPP_
