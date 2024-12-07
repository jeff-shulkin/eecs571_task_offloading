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
#include <mutex>
#include <condition_variable>
#include <string>
#include <vector>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_util/service_client.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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
#include "offload_server/utils.hpp"

/** Supported functions
 * Offload Localization
 * Power off
 */

namespace offload_server
{

struct ROS2Job {
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<task_action_interfaces::action::Offloadlocalization>> goal_handle;
        std::string agent_id;
        uint32_t job_id;
        std::chrono::milliseconds deadline;
	sensor_msgs::msg::LaserScan laserscan;
	geometry_msgs::msg::PoseWithCovarianceStamped ipose;
};

// Custom priority queue job comparator
// struct ROS2JobCompare {
//     bool operator()(ROS2Job& a, ROS2Job& b){
//             return a->callback. > b;
//     }
// };

// Timer Periods
static constexpr auto LIDAR_5HZ_TIMER_DEADLINE_MS = 200;
static constexpr auto LIDAR_10HZ_TIMER_DEADLINE_MS = 100;
static constexpr auto WIFI_TIMER_LATENCY_DEADLINE_MS = 70; 

class OffloadServer : public rclcpp::Node
{

public:

  using OffloadLocalization = task_action_interfaces::action::Offloadlocalization;
  using GoalHandleOffloadLocalization = rclcpp_action::ServerGoalHandle<OffloadLocalization>;

  // Constructor and Destructor
  explicit OffloadServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~OffloadServer();

  // nav2 publisher and subscriber getters and setters
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr get_nav2_ipose_pub_();

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr get_nav2_laser_scan_pub_();

  geometry_msgs::msg::PoseWithCovarianceStamped get_offload_amcl_fpose_();

  bool get_FPOSE_READY_flag();
  void set_FPOSE_READY_flag(bool value);
  void add_job(ROS2Job j);
  void remove_job(ROS2Job j);
  void execute();
  void stop();
  void print_all_jobs();

private:
  void run();

  // Subscription callbacks
  void nav2_amcl_fpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr nav2_amcl_fpose_msg);
  void nav2_local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr nav2_local_costmap_msg);
  void nav2_amcl_transform_callback(const geometry_msgs::msg::TransformStamped::SharedPtr nav2_amcl_transform_msg);
  void nav2_static_transform_callback(const geometry_msgs::msg::TransformStamped::SharedPtr nav2_static_transform_msg);

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

  // Function for querying localization node statuses
  bool query_localization_node_status();

  // Function for querying transforms
  void query_localization_transforms();

  // Function for sending over intial pose to AMCL
  void send_initial_pose(geometry_msgs::msg::PoseWithCovarianceStamped& ipose);

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
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr nav2_amcl_fpose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr nav2_local_costmap_map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr nav2_amcl_transforms_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr nav2_static_transforms_sub_;

  // Nav2 Service for sending initial pose to amcl
  rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr nav2_initial_pose_client_;

  // Nav2 Service for querying localization lifecycle status
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr nav2_localization_status_client_;

  // Nav2 Service for managing localization lifecycle
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr nav2_localization_manager_client_;

  // Nav2 data ready flags
  bool FPOSE_READY;
  bool COSTMAP_READY;
  bool TRANSFORMS_READY;

  // Store latest LiDAR data sent from offload_agent
  sensor_msgs::msg::LaserScan latest_lidar_msg_;

  // Store initial pose sent from offload agent
  geometry_msgs::msg::PoseWithCovarianceStamped offload_amcl_ipose_;

  // Store localization pose received from offload_server
  geometry_msgs::msg::PoseWithCovarianceStamped offload_amcl_fpose_;

  // Store costmap received from offload_server
  nav_msgs::msg::OccupancyGrid offload_local_rcostmap_;

  // Store the Nav2 transforms
  geometry_msgs::msg::TransformStamped offload_map_to_odom_transform_;
  geometry_msgs::msg::TransformStamped offload_odom_to_base_transform_;
  geometry_msgs::msg::TransformStamped offload_base_to_laser_transform_;

  // Nav2 TF2 transform listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool offload_status_;

  // Store server scheduling algorithm
  ServerSchedulingAlgo algo_;

  // All scheduler data structures
  std::queue<ROS2Job> fifo_sched_;
  std::thread scheduler_;
  std::atomic<bool> running_;

  // Mutex for accessing fifo queue
  std::mutex fifo_lock_;

  // Mutex for accessing FPOSE flag
  std::mutex FPOSE_lock;

  // Condition variable for safely accessing FPOSE_READY
  std::condition_variable FPOSE_condition;

  // Keep track of how many times we have entered the fpose callback
  uint32_t fpose_callback_count = 0;
};

}  // namespace offload_server

RCLCPP_COMPONENTS_REGISTER_NODE(offload_server::OffloadServer)

#endif  // OFFLOAD_SERVER_NODE__HPP_
