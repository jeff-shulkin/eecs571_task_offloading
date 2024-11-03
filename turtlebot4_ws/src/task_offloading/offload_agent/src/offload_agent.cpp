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

#include "offload_agent/offload_agent.hpp"
#include <turtlebot4_node/turtlebot4.hpp>
#include <turtlebot4_node/action.hpp>
#include <turtlebot4_node/leds.hpp>
#include <turtlebot4_node/display.hpp>
#include <turtlebot4_node/service.hpp>
#include <turtlebot4_node/utils.hpp>
#include "task_action_interfaces/action/offloadamcl.hpp"


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

using namespace turtlebot4;

using offload_agent::OffloadAgent;
using turtlebot4::Turtlebot4;
using turtlebot4::Leds;
using Dock = irobot_create_msgs::action::Dock;
using Undock = irobot_create_msgs::action::Undock;
using WallFollow = irobot_create_msgs::action::WallFollow;
using LedAnimation = irobot_create_msgs::action::LedAnimation;
using EStop = irobot_create_msgs::srv::EStop;
using Power = irobot_create_msgs::srv::RobotPower;
using EmptySrv = std_srvs::srv::Empty;
using TriggerSrv = std_srvs::srv::Trigger;

/**
 * @brief Turtlebot4 Node constructor
 */
OffloadAgent::OffloadAgent()
: Node("turtlebot4_offload_agent",
    rclcpp::NodeOptions().use_intra_process_comms(true)),
  wheels_enabled_(true),
  is_docked_(false)
{
  RCLCPP_INFO(get_logger(), "Init Turtlebot4 With Offloading Node Main");

  // Create node handle
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  // ROS parameters
  this->declare_parameter("model", turtlebot4::Turtlebot4ModelName[turtlebot4::Turtlebot4Model::STANDARD]);
  std::string model = this->get_parameter("model").as_string();
  if (model == turtlebot4::Turtlebot4ModelName[turtlebot4::Turtlebot4Model::STANDARD]) {
    model_ = turtlebot4::Turtlebot4Model::STANDARD;
  } else if (model == turtlebot4::Turtlebot4ModelName[turtlebot4::Turtlebot4Model::LITE]) {
    model_ = turtlebot4::Turtlebot4Model::LITE;
  } else {
    RCLCPP_ERROR(
      node_handle_->get_logger(), "Invalid Model %s",
      model.c_str());
    return;
  }

  this->declare_parameter("wifi.interface", "wlan0");
  wifi_interface_ = this->get_parameter("wifi.interface").as_string();

  this->declare_parameter("power_saver", true);
  power_saver_ = this->get_parameter("power_saver").as_bool();

  button_parameters_ = {
    {turtlebot4::CREATE3_1, "buttons.create3_1"},
    {turtlebot4::CREATE3_POWER, "buttons.create3_power"},
    {turtlebot4::CREATE3_2, "buttons.create3_2"},
    {turtlebot4::CONTROLLER_A, "controller.a"},
    {turtlebot4::CONTROLLER_B, "controller.b"},
    {turtlebot4::CONTROLLER_X, "controller.x"},
    {turtlebot4::CONTROLLER_Y, "controller.y"},
    {turtlebot4::CONTROLLER_UP, "controller.up"},
    {turtlebot4::CONTROLLER_DOWN, "controller.down"},
    {turtlebot4::CONTROLLER_LEFT, "controller.left"},
    {turtlebot4::CONTROLLER_RIGHT, "controller.right"},
    {turtlebot4::CONTROLLER_L1, "controller.l1"},
    {turtlebot4::CONTROLLER_L2, "controller.l2"},
    {turtlebot4::CONTROLLER_L3, "controller.l3"},
    {turtlebot4::CONTROLLER_R1, "controller.r1"},
    {turtlebot4::CONTROLLER_R2, "controller.r2"},
    {turtlebot4::CONTROLLER_R3, "controller.r3"},
    {turtlebot4::CONTROLLER_SHARE, "controller.share"},
    {turtlebot4::CONTROLLER_OPTIONS, "controller.options"},
    {turtlebot4::CONTROLLER_HOME, "controller.home"},
    {turtlebot4::HMI_1, "buttons.hmi_1"},
    {turtlebot4::HMI_2, "buttons.hmi_2"},
    {turtlebot4::HMI_3, "buttons.hmi_3"},
    {turtlebot4::HMI_4, "buttons.hmi_4"},
  };

  turtlebot4::Turtlebot4ButtonEnum last;

  if (model_ == turtlebot4::Turtlebot4Model::STANDARD) {
    last = turtlebot4::Turtlebot4ButtonEnum::HMI_4;
  } else {
    last = turtlebot4::Turtlebot4ButtonEnum::CONTROLLER_HOME;
  }

  // Declare and add buttons
  for (uint8_t i = turtlebot4::Turtlebot4ButtonEnum::CREATE3_1; i <= last; i++) {
    this->declare_parameter(
      button_parameters_[static_cast<turtlebot4::Turtlebot4ButtonEnum>(i)],
      std::vector<std::string>());
    turtlebot4_buttons_.push_back(
      turtlebot4::Turtlebot4Button(
        this->get_parameter(
          button_parameters_[static_cast<turtlebot4::Turtlebot4ButtonEnum>(i)]).as_string_array()));
  }

  this->declare_parameter("menu.entries", std::vector<std::string>());
  auto entries = this->get_parameter("menu.entries").as_string_array();

  for (auto entry : entries) {
    turtlebot4_menu_entries_.push_back(turtlebot4::Turtlebot4MenuEntry(entry));
  }

  // Subscriptions
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state",
    rclcpp::SensorDataQoS(),
    std::bind(&OffloadAgent::battery_callback, this, std::placeholders::_1));

  dock_status_sub_ = this->create_subscription<irobot_create_msgs::msg::DockStatus>(
    "dock_status",
    rclcpp::SensorDataQoS(),
    std::bind(&OffloadAgent::dock_status_callback, this, std::placeholders::_1));

  wheel_status_sub_ = this->create_subscription<irobot_create_msgs::msg::WheelStatus>(
    "wheel_status",
    rclcpp::SensorDataQoS(),
    std::bind(&OffloadAgent::wheel_status_callback, this, std::placeholders::_1));

  // Publishers
  ip_pub_ = this->create_publisher<std_msgs::msg::String>(
    "ip",
    rclcpp::QoS(rclcpp::KeepLast(10)));

  function_call_pub_ = this->create_publisher<std_msgs::msg::String>(
    "function_calls",
    rclcpp::QoS(rclcpp::KeepLast(10)));

  // Create action/service clients
  // OUR ADDITIONAL ACTION CLIENTS: OFFLOAD AMCL, OFFLOAD COSTMAPS
  offload_amcl_client_ = std::make_unique<turtlebot4::Turtlebot4Action<AMCL>>(node_handle_, "offload_amcl");
  //offload_costmap_client_ = std::make_unique<Turtlebot4Action<Costmap>>(node_handle_, "offload_costmap");
  dock_client_ = std::make_unique<turtlebot4::Turtlebot4Action<Dock>>(node_handle_, "dock");
  undock_client_ = std::make_unique<turtlebot4::Turtlebot4Action<Undock>>(node_handle_, "undock");
  wall_follow_client_ = std::make_unique<turtlebot4::Turtlebot4Action<WallFollow>>(node_handle_, "wall_follow");
  led_animation_client_ = std::make_unique<turtlebot4::Turtlebot4Action<LedAnimation>>(
    node_handle_,
    "led_animation");
  estop_client_ = std::make_unique<turtlebot4::Turtlebot4Service<EStop>>(node_handle_, "e_stop");
  power_client_ = std::make_unique<turtlebot4::Turtlebot4Service<Power>>(node_handle_, "robot_power");
  rplidar_start_client_ = std::make_unique<turtlebot4::Turtlebot4EmptyService<EmptySrv>>(
    node_handle_,
    "start_motor");
  rplidar_stop_client_ = std::make_unique<turtlebot4::Turtlebot4EmptyService<EmptySrv>>(
    node_handle_,
    "stop_motor");
  oakd_start_client_ = std::make_unique<turtlebot4::Turtlebot4Service<TriggerSrv>>(
    node_handle_,
    "oakd/start_camera");
  oakd_stop_client_ = std::make_unique<turtlebot4::Turtlebot4Service<TriggerSrv>>(
    node_handle_,
    "oakd/stop_camera");

  function_callbacks_ = {
    //ADD OUR FUNCTION CALLBACKS HERE
    {"Offload AMCL", std::bind(&OffloadAgent::offload_amcl_function_callback, this)},
    //{"Offload Costmap", std::bind(&OffloadAgent::offload_costmap_function_callback, this)},
    {"Dock", std::bind(&OffloadAgent::dock_function_callback, this)},
    {"Undock", std::bind(&OffloadAgent::undock_function_callback, this)},
    {"Wall Follow Left", std::bind(&OffloadAgent::wall_follow_left_function_callback, this)},
    {"Wall Follow Right", std::bind(&OffloadAgent::wall_follow_right_function_callback, this)},
    {"EStop", std::bind(&OffloadAgent::estop_function_callback, this)},
    {"Power", std::bind(&OffloadAgent::power_function_callback, this)},
    {"RPLIDAR Start", std::bind(&OffloadAgent::rplidar_start_function_callback, this)},
    {"RPLIDAR Stop", std::bind(&OffloadAgent::rplidar_stop_function_callback, this)},
    {"OAKD Start", std::bind(&OffloadAgent::oakd_start_function_callback, this)},
    {"OAKD Stop", std::bind(&OffloadAgent::oakd_stop_function_callback, this)},
    {"Scroll Up", std::bind(&OffloadAgent::scroll_up_function_callback, this)},
    {"Scroll Down", std::bind(&OffloadAgent::scroll_down_function_callback, this)},
    {"Select", std::bind(&OffloadAgent::select_function_callback, this)},
    {"Back", std::bind(&OffloadAgent::back_function_callback, this)},
    {"Help", std::bind(&OffloadAgent::help_function_callback, this)},
  };

  // Set function callbacks
  add_button_function_callbacks();
  add_menu_function_callbacks();

  // Buttons
  buttons_ = std::make_unique<turtlebot4::Buttons>(model_, turtlebot4_buttons_, node_handle_);

  if (model_ == turtlebot4::Turtlebot4Model::STANDARD) {
    // Display
    display_ = std::make_unique<turtlebot4::Display>(turtlebot4_menu_entries_, node_handle_);

    // Leds
    leds_ = std::make_unique<turtlebot4::Leds>(node_handle_);
  }

  run();
}

/**
 * @brief Offload Agent Node run
 */
void OffloadAgent::run()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot4 With Offloading %s running.", Turtlebot4ModelName[model_].c_str());

  if (model_ == turtlebot4::Turtlebot4Model::STANDARD) {
    // Set Power LED
    leds_->set_led(POWER, GREEN);
    // Set Motors LED
    leds_->set_led(MOTORS, GREEN);

    display_timer(std::chrono::milliseconds(turtlebot4::DISPLAY_TIMER_PERIOD));
    leds_timer(std::chrono::milliseconds(turtlebot4::LEDS_TIMER_PERIOD));
  }

  buttons_timer(std::chrono::milliseconds(turtlebot4::BUTTONS_TIMER_PERIOD));
  wifi_timer(std::chrono::milliseconds(turtlebot4::WIFI_TIMER_PERIOD));
  comms_timer(std::chrono::milliseconds(turtlebot4::COMMS_TIMER_PERIOD));
}

/**
 * @brief Sends offload_amcl action goal
 */
void OffloadAgent::offload_amcl_function_callback()
{
  if (offload_amcl_client_ != nullptr) {
    RCLCPP_INFO(this->get_logger(), "Offloading AMCL Calculations");
    offload_amcl_client_->send_goal();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Offload AMCL client NULL");
  }
}

/**
 * @brief Sends offload_costmaps action goal
// */
//void OffloadAgent::offload_costmaps_function_callback()
//{
//  if (offload_costmap_client_ != nullptr) {
//    RCLCPP_INFO(this->get_logger(), "Offloading local costmap computations");
//    offload_costmap_client_->send_goal();
//  } else {
//    RCLCPP_ERROR(this->get_logger(), "Undock client NULL");
//  }
//}
