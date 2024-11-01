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

using offload_server::OffloadServer;
using AMCL = action_server_interfaces::action::OffloadAMCL;
using Costmap = action_server_interfaces::action::OffloadCostmap;

/**
 * @brief OffloadServer Node constructor
 */
OffloadServer::OffloadServer()
: Node("offload_server_node",
    rclcpp::NodeOptions().use_intra_process_comms(true)),
  power_saver_(true)
{
  RCLCPP_INFO(get_logger(), "Init Offload Server Node");

  // Create node handle
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  // ROS parameters
  this->declare_parameter("sched_algo", [ServerSchedAlgo::FIFO_QUEUE]);
  std::string algorithm = this->get_parameter("algo").as_string();

  if (algorithm == SchedAlgoName[ServerSchedulingAlgo::FIFO_QUEUE]) {
    algo_ = ServerSchedulingAlgo::FIFO_QUEUE;
  } else if (algorithm == SchedAlgoName[ServerSchedulingAlgo::ROUND_ROBIN]) {
    algo_ = ServerSchedulingAlgo::ROUND_ROBIN;
  } else if (algorithm == SchedAlgoName[ServerSchedulingAlgo::RMS]) {
    algo_ = ServerSchedulingAlgo::RMS;
  } else if (algorithm == SchedAlgoName[ServerSchedulingAlgo::EDF]) {
    algo_ = ServerSchedulingAlgo::EDF;
  } else if (algorithm == SchedAlgoName[ServerSchedulingAlgo::LSTF]) {
    algo_ = ServerSchedulingAlgo::LSTF;
  } else {
    RCLCPP_ERROR(
      node_handle_->get_logger(), "Invalid Scheduling Algorithm %s",
      algorithm.c_str());
    return;
  }

  this->declare_parameter("wifi.interface", "wlan0");
  wifi_interface_ = this->get_parameter("wifi.interface").as_string();

  this->declare_parameter("power_saver", true);
  power_saver_ = this->get_parameter("power_saver").as_bool();

  // ROS Action Servers
  this->offload_amcl_action_server_ = rclcpp::create_server<AMCL>(
  this->get_node_base_interface(),
  this->get_node_clock_interface(),
  this->get_node_logging_interface(),
  this->get_node_waitables_interface(),
  "offload_amcl",
  std::bind(&OffloadAMCLActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
  std::bind(&OffloadAMCLActionServer::handle_cancel, this, std::placeholders::_1),
  std::bind(&OffloadAMCLActionServer::handle_accepted, this, std::placeholders::_1));

  // TODO: ADD BACK IN COSTMAP ACTION SERVER WHEN READY
  //this->offload_costmap_action_server_ = rclcpp::create_serer<AMCL>(
  //this->get_node_base_interface(),
  //this->get_node_clock_interface(),
  //this->get_node_logging_interface(),
  //this->get_node_waitables_interface(),
  //"offload_costmap",
  //std::bind(&OffloadCostmapActionServer::handle_goal, this, std::placeholders::_1, std::place>
  //std::bind(&OffloadCostmapActionServer::handle_cancel, this, std::placeholders::_1),
  //std::bind(&OffloadCostmapActionServer::handle_accepted, this, std::placeholders::_1));

  // Subscriptions
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state",
    rclcpp::SensorDataQoS(),
    std::bind(&OffloadServer::battery_callback, this, std::placeholders::_1));

  laser_scan_sub_ = this->create-subscription<sensor_msgs::msg::LaserScan>(
    "laser_scan",
    rclcpp::SensorDataQos(),
    std::bind(&OffloadServer::laser_scan_callback, this, std::placeholders::_1));

  // Publishers
  ip_pub_ = this->create_publisher<std_msgs::msg::String>(
    "ip",
    rclcpp::QoS(rclcpp::KeepLast(10)));

  function_call_pub_ = this->create_publisher<std_msgs::msg::String>(
    "function_calls",
    rclcpp::QoS(rclcpp::KeepLast(10)));

  run();
}

/**
 * @brief Offload Server Node run
 */
void Offload_Server::run()
{
  RCLCPP_INFO(this->get_logger(), "Offload Server running.");
  RCLCPP_DEBUG(this->get_logger(), "Offload Server using %s algo.", algo_.c_str());
  
  // Start scheduler with given scheduling algorithm
  //scheduler_.start();

  wifi_timer(std::chrono::milliseconds(WIFI_TIMER_PERIOD));
  //power_off_timer(std::chrono::milliseconds(
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
          RCLCPP_DEBUG(this->get_logger(), "IP unknown.");
        }
      }
    });
}

/**
 * @brief Creates and runs timer for powering off the robot when low battery
 * @input timeout - Sets timer period in milliseconds
 */
void Turtlebot4::power_off_timer(const std::chrono::milliseconds timeout)
{
  power_off_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      RCLCPP_INFO(this->get_logger(), "Powering off");
      power_function_callback();
    });
}

/**
 * @brief Battery subscription callback
 * @input battery_state_msg - Received message on battery topic
 */
void Turtlebot4::dock_status_callback(
  const irobot_create_msgs::msg::DockStatus::SharedPtr dock_status_msg)
{
  // Dock status has changed and power saver is enabled
  if (dock_status_msg->is_docked != is_docked_ && power_saver_) {
    // The robot has docked, turn off the camera and lidar
    if (dock_status_msg->is_docked) {
      oakd_stop_function_callback();
      rplidar_stop_function_callback();
    } else {  // The robot has undocked, turn on the camera and lidar
      oakd_start_function_callback();
      rplidar_start_function_callback();
    }
    is_docked_ = dock_status_msg->is_docked;
  }
}

/**
 * @brief Sends dock action goal
 */
void Turtlebot4::dock_function_callback()
{
  if (dock_client_ != nullptr) {
    RCLCPP_INFO(this->get_logger(), "Docking");
    dock_client_->send_goal();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Dock client NULL");
  }
}

/**
 * @brief Sends undock action goal
 */
void Turtlebot4::undock_function_callback()
{
  if (undock_client_ != nullptr) {
    RCLCPP_INFO(this->get_logger(), "Undocking");
    undock_client_->send_goal();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Undock client NULL");
  }
}

/**
 * @brief Sends power service request
 */
void Turtlebot4::power_function_callback()
{
  if (power_client_ != nullptr) {
    // Make power off request
    auto request = std::make_shared<Power::Request>();

    RCLCPP_ERROR(this->get_logger(), "Power OFF");
    power_client_->make_request(request);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Power client NULL");
  }
}

/**
 * @brief Callback for when a function call is executed
 */
void OffloadServer::function_call_callback(std::string function_name)
{
  std_msgs::msg::String msg;
  msg.data = function_name;

  function_call_pub_->publish(msg);
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
