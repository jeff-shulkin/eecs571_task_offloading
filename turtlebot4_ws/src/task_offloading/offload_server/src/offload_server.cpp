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

#include "offload_server/offload_server.hpp"
#include "offload_server/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>

using namespace offload_server;
using offload_server::OffloadServer;
//using Costmap = action_server_interfaces::action::OffloadCostmap;

/**
 * @brief OffloadServer Node constructor
 */
OffloadServer::OffloadServer(const rclcpp::NodeOptions & options)
: Node("offload_server", options)
{
  RCLCPP_INFO(get_logger(), "Init Offload Server Node");

  // Create node handle
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  // ROS parameters
  this->declare_parameter("algo", SchedAlgoName[ServerSchedulingAlgo::FIFO_QUEUE]);
  std::string algorithm = this->get_parameter("algo").as_string();

  if (algorithm == offload_server::SchedAlgoName[ServerSchedulingAlgo::FIFO_QUEUE]) {
    algo_ = offload_server::ServerSchedulingAlgo::FIFO_QUEUE;
  } else if (algorithm == offload_server::SchedAlgoName[ServerSchedulingAlgo::ROUND_ROBIN]) {
    algo_ = offload_server::ServerSchedulingAlgo::ROUND_ROBIN;
  } else if (algorithm == offload_server::SchedAlgoName[ServerSchedulingAlgo::RMS]) {
    algo_ = offload_server::ServerSchedulingAlgo::RMS;
  } else if (algorithm == offload_server::SchedAlgoName[ServerSchedulingAlgo::EDF]) {
    algo_ = offload_server::ServerSchedulingAlgo::EDF;
  } else if (algorithm == offload_server::SchedAlgoName[ServerSchedulingAlgo::LSTF]) {
    algo_ = offload_server::ServerSchedulingAlgo::LSTF;
  } else {
    RCLCPP_ERROR(
      node_handle_->get_logger(), "Invalid Scheduling Algorithm %s",
      algorithm.c_str());
    return;
  }

  RCLCPP_INFO(node_handle_->get_logger(), "Using Scheduling algorithm %s", algorithm.c_str());
  this->declare_parameter("wifi.interface", "wlan0");
  wifi_interface_ = this->get_parameter("wifi.interface").as_string();

  // ROS Action Servers
  offload_localization_action_server_ = rclcpp_action::create_server<OffloadLocalization>(
  node_handle_,
  "offload_localization",
  std::bind(&OffloadServer::handle_offload_localization_goal, this, std::placeholders::_1, std::placeholders::_2),
  std::bind(&OffloadServer::handle_offload_localization_cancel, this, std::placeholders::_1),
  std::bind(&OffloadServer::handle_offload_localization_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(node_handle_->get_logger(), "offload_localization action server instantiated");

  // Nav2 manager client service
  nav2_localization_manager_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");

  // Nav2 Subscriptions
  nav2_amcl_fpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose",
    rclcpp::SensorDataQoS(),
    std::bind(&OffloadServer::nav2_amcl_fpose_callback, this, std::placeholders::_1));

  nav2_local_costmap_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "local_costmap/local_costmap",
    rclcpp::SensorDataQoS(),
    std::bind(&OffloadServer::nav2_local_costmap_callback, this, std::placeholders::_1));

  // Nav2 flags
  FPOSE_READY = false;
  COSTMAP_READY = false;

  // Publishers
  ip_pub_ = this->create_publisher<std_msgs::msg::String>(
    "ip",
    rclcpp::QoS(rclcpp::KeepLast(10)));

  nav2_ipose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose",
   rclcpp::QoS(rclcpp::KeepLast(10)));

  nav2_laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan",
  rclcpp::QoS(rclcpp::KeepLast(10)));

  // Scheduler
  std::thread scheduler(std::bind(&OffloadServer::execute, this));
  scheduler.detach();


  run();
}

/**
 * @brief Offload Server Node run
 */
void OffloadServer::run()
{
  RCLCPP_INFO(this->get_logger(), "Offload Server running.");

  // Start scheduler with given scheduling algorithm
  //scheduler_.start();

  //wifi_timer(std::chrono::milliseconds(WIFI_TIMER_LATENCY_DEADLINE_MS));
}

/**
 * @brief Creates and runs timer to check Wifi connection
 * @input timeout - Sets timer period in milliseconds
 */
void OffloadServer::wifi_timer(const std::chrono::milliseconds timeout)
{
  wifi_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      std::string ip = this->get_ip();

      // Publish IP
      std_msgs::msg::String msg;
      msg.data = ip;
      this->ip_pub_->publish(std::move(msg));

      if (ip == std::string(UNKNOWN_IP)) {
        RCLCPP_INFO(this->get_logger(), "Unknown Server IP");
      }

    });
}

/**
 * @brief Get IP of network interface specified in ROS parameters
 */
std::string OffloadServer::get_ip()
{
  struct ifaddrs * ifAddrStruct = NULL;
  struct ifaddrs * ifa = NULL;
  void * tmpAddrPtr = NULL;

  getifaddrs(&ifAddrStruct);

  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr) {
      continue;
    }
    // IPv4
    if (ifa->ifa_addr->sa_family == AF_INET) {
      struct sockaddr_in * ifa_in_addr = (struct sockaddr_in *)ifa->ifa_addr;
      tmpAddrPtr = &(ifa_in_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
      // Find specified network interface
      if (strcmp(ifa->ifa_name, wifi_interface_.c_str()) == 0) {
        if (ifAddrStruct != NULL) {
          freeifaddrs(ifAddrStruct);
        }
        return static_cast<std::string>(addressBuffer);
      }
    }
  }
  if (ifAddrStruct != NULL) {
    freeifaddrs(ifAddrStruct);
  }
  return std::string(UNKNOWN_IP);
}

void OffloadServer::nav2_amcl_fpose_callback(std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> nav2_amcl_fpose_msg) {
    // Handle final pose
    RCLCPP_INFO(this->get_logger(), "received amcl pose from offload_server");
    offload_amcl_fpose_ = *nav2_amcl_fpose_msg;
    FPOSE_READY = true;
}

void OffloadServer::nav2_local_costmap_callback(std::shared_ptr<nav_msgs::msg::OccupancyGrid> nav2_local_costmap_msg) {
    RCLCPP_INFO(this->get_logger(), "received costmap from offload_server");
    offload_local_rcostmap_ = *nav2_local_costmap_msg;
}


void OffloadServer::add_job(ROS2Job j) {
    RCLCPP_INFO(this->get_logger(), "adding job to fifo scheduler");
    fifo_lock_.lock();
    fifo_sched_.push(j);
    fifo_lock_.unlock();
}


void OffloadServer::execute() {
    while(1) {
      fifo_lock_.lock();
      if (!fifo_sched_.empty()) {
          ROS2Job curr_job = fifo_sched_.front();
          RCLCPP_INFO(this->get_logger(), "publishing initial pose and LiDAR data to offload_server nav2 stack");

	  nav2_ipose_pub_->publish(curr_job.ipose);
	  nav2_laser_scan_pub_->publish(curr_job.laserscan);

	  while(!FPOSE_READY) { continue; } // CAUTION: busy looping is bad
          RCLCPP_INFO(this->get_logger(), "received amcl pose back from nav2 stack");
	  FPOSE_READY = false;

	  auto result = std::make_shared<task_action_interfaces::action::Offloadlocalization::Result>();
	  result->success = true;
	  result->final_pose = offload_amcl_fpose_;
	  result->exec_time = 0.0f;
	  curr_job.goal_handle->succeed(result);

          RCLCPP_INFO(this->get_logger(), "sent goal back to offload_agent");
          fifo_sched_.pop();
      }
      else {
          RCLCPP_INFO(this->get_logger(), "fifo queue empty");
      }
    fifo_lock_.unlock();
    }
}

