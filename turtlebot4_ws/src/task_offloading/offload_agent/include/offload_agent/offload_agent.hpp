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

#ifndef OFFLOAD_AGENT_NODE_HPP_
#define OFFLOAD_AGENT_NODE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "turtlebot4_node/turtlebot4.hpp"

/** Supported functions
 * Dock
 * Undock
 * Follow
 * Power off
 * EStop
 */

namespace offload_agent
{

// Timer Periods
static constexpr auto BUTTONS_TIMER_PERIOD = 10;
static constexpr auto COMMS_TIMER_PERIOD = 30000;
static constexpr auto DISPLAY_TIMER_PERIOD = 50;
static constexpr auto LEDS_TIMER_PERIOD = 50;
static constexpr auto POWER_OFF_TIMER_PERIOD = 60000;
static constexpr auto WIFI_TIMER_PERIOD = 5000;

class OffloadAgent : public turtlebot4::Turtlebot4
{
public:
  // Type alias for actions and services
  using AMCL = task_action_interfaces::action::OffloadAMCL;
  using Costmap = irobot_create_msgs::action::OffloadCostmap;

  // Constructor and Destructor
  OffloadAgent();
  virtual ~OffloadAgent() {}

private:
  void run();

  // Subscription callbacks

  // Actions
  std::unique_ptr<Turtlebot4Action<AMCL>> offload_amcl_client_;
  std::unique_ptr<Turtlebot4Action<Costmap>> offload_costmap_client_;

};

}  // namespace offload_agent

#endif  // OFFLOAD_AGENT_NODE_HPP_
