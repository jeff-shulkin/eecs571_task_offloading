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

#include "offload_agent/turtlebot4.hpp"
#include "task_action_interfaces/action/offloadlocalization.hpp"

#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

#include <chrono>
#include <regex>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <fstream>

using turtlebot4::Turtlebot4;
using OffloadLocalization = task_action_interfaces::action::Offloadlocalization;
using GoalHandleOffloadLocalization = rclcpp_action::ServerGoalHandle<OffloadLocalization>; // TODO :: ruiying
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
Turtlebot4::Turtlebot4()
: Node("turtlebot4_node",
    rclcpp::NodeOptions().use_intra_process_comms(true)),
  wheels_enabled_(true),
  is_docked_(false)
{
  RCLCPP_INFO(get_logger(), "Init Turtlebot4 Node Main");

  // Create node handle
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  // ROS parameters
  this->declare_parameter("model", Turtlebot4ModelName[Turtlebot4Model::STANDARD]);
  std::string model = this->get_parameter("model").as_string();
  if (model == Turtlebot4ModelName[Turtlebot4Model::STANDARD]) {
    model_ = Turtlebot4Model::STANDARD;
  } else if (model == Turtlebot4ModelName[Turtlebot4Model::LITE]) {
    model_ = Turtlebot4Model::LITE;
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

  this->declare_parameter("robot_id", "turtlebot4_default");
  robot_id_ = this->get_parameter("robot_id").as_string();

  this->declare_parameter("server_ip", "127.0.0.1");
  server_ip_ = this->get_parameter("server_ip").as_string();

  RCLCPP_INFO(this->get_logger(), "robot_id: %s", robot_id_.c_str());

  // Grab the initial pose
  this->declare_parameter("initial_pose.x", 0.0);
  this->declare_parameter("initial_pose.y", 0.0);
  this->declare_parameter("initial_pose.z", 0.0);
  this->declare_parameter("initial_pose.qx", 0.0);
  this->declare_parameter("initial_pose.qy", 0.0);
  this->declare_parameter("initial_pose.qz", 0.0);
  this->declare_parameter("initial_pose.qw", 0.0);

  initial_pose_.position.x = this->get_parameter("initial_pose.x").as_double();
  initial_pose_.position.y = this->get_parameter("initial_pose.y").as_double();
  initial_pose_.position.z = this->get_parameter("initial_pose.z").as_double();
  initial_pose_.orientation.x = this->get_parameter("initial_pose.qx").as_double();
  initial_pose_.orientation.y = this->get_parameter("initial_pose.qy").as_double();
  initial_pose_.orientation.z = this->get_parameter("initial_pose.qz").as_double();
  initial_pose_.orientation.w = this->get_parameter("initial_pose.qw").as_double();

  button_parameters_ = {
    {CREATE3_1, "buttons.create3_1"},
    {CREATE3_POWER, "buttons.create3_power"},
    {CREATE3_2, "buttons.create3_2"},
    {CONTROLLER_A, "controller.a"},
    {CONTROLLER_B, "controller.b"},
    {CONTROLLER_X, "controller.x"},
    {CONTROLLER_Y, "controller.y"},
    {CONTROLLER_UP, "controller.up"},
    {CONTROLLER_DOWN, "controller.down"},
    {CONTROLLER_LEFT, "controller.left"},
    {CONTROLLER_RIGHT, "controller.right"},
    {CONTROLLER_L1, "controller.l1"},
    {CONTROLLER_L2, "controller.l2"},
    {CONTROLLER_L3, "controller.l3"},
    {CONTROLLER_R1, "controller.r1"},
    {CONTROLLER_R2, "controller.r2"},
    {CONTROLLER_R3, "controller.r3"},
    {CONTROLLER_SHARE, "controller.share"},
    {CONTROLLER_OPTIONS, "controller.options"},
    {CONTROLLER_HOME, "controller.home"},
    {HMI_1, "buttons.hmi_1"},
    {HMI_2, "buttons.hmi_2"},
    {HMI_3, "buttons.hmi_3"},
    {HMI_4, "buttons.hmi_4"},
  };

  Turtlebot4ButtonEnum last;

  if (model_ == Turtlebot4Model::STANDARD) {
    last = Turtlebot4ButtonEnum::HMI_4;
  } else {
    last = Turtlebot4ButtonEnum::CONTROLLER_HOME;
  }

  // Declare and add buttons
  for (uint8_t i = Turtlebot4ButtonEnum::CREATE3_1; i <= last; i++) {
    this->declare_parameter(
      button_parameters_[static_cast<Turtlebot4ButtonEnum>(i)],
      std::vector<std::string>());
    turtlebot4_buttons_.push_back(
      Turtlebot4Button(
        this->get_parameter(
          button_parameters_[static_cast<Turtlebot4ButtonEnum>(i)]).as_string_array()));
  }

  this->declare_parameter("menu.entries", std::vector<std::string>());
  auto entries = this->get_parameter("menu.entries").as_string_array();

  for (auto entry : entries) {
    turtlebot4_menu_entries_.push_back(Turtlebot4MenuEntry(entry));
  }

  // Subscriptions
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state",
    rclcpp::SensorDataQoS(),
    std::bind(&Turtlebot4::battery_callback, this, std::placeholders::_1));

  dock_status_sub_ = this->create_subscription<irobot_create_msgs::msg::DockStatus>(
    "dock_status",
    rclcpp::SensorDataQoS(),
    std::bind(&Turtlebot4::dock_status_callback, this, std::placeholders::_1));

  wheel_status_sub_ = this->create_subscription<irobot_create_msgs::msg::WheelStatus>(
    "wheel_status",
    rclcpp::SensorDataQoS(),
    std::bind(&Turtlebot4::wheel_status_callback, this, std::placeholders::_1));

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan",
    rclcpp::SensorDataQoS(),
    std::bind(&Turtlebot4::lidar_callback, this, std::placeholders::_1));

  // Publishers
  ip_pub_ = this->create_publisher<std_msgs::msg::String>(
    "ip",
    rclcpp::QoS(rclcpp::KeepLast(10)));

  function_call_pub_ = this->create_publisher<std_msgs::msg::String>(
    "function_calls",
    rclcpp::QoS(rclcpp::KeepLast(10)));

  // Create action/service clients
  offload_localization_client_ = std::make_unique<turtlebot4::Turtlebot4Action<OffloadLocalization>>(node_handle_, "offload_localization");
  dock_client_ = std::make_unique<Turtlebot4Action<Dock>>(node_handle_, "dock");
  undock_client_ = std::make_unique<Turtlebot4Action<Undock>>(node_handle_, "undock");
  wall_follow_client_ = std::make_unique<Turtlebot4Action<WallFollow>>(node_handle_, "wall_follow");
  led_animation_client_ = std::make_unique<Turtlebot4Action<LedAnimation>>(
    node_handle_,
    "led_animation");
  estop_client_ = std::make_unique<Turtlebot4Service<EStop>>(node_handle_, "e_stop");
  power_client_ = std::make_unique<Turtlebot4Service<Power>>(node_handle_, "robot_power");
  rplidar_start_client_ = std::make_unique<Turtlebot4EmptyService<EmptySrv>>(
    node_handle_,
    "start_motor");
  rplidar_stop_client_ = std::make_unique<Turtlebot4EmptyService<EmptySrv>>(
    node_handle_,
    "stop_motor");
  oakd_start_client_ = std::make_unique<Turtlebot4Service<TriggerSrv>>(
    node_handle_,
    "oakd/start_camera");
  oakd_stop_client_ = std::make_unique<Turtlebot4Service<TriggerSrv>>(
    node_handle_,
    "oakd/stop_camera");

  // TODO :: ruiying
  // bind the result and feedback callback functions from localization server
  auto localization_send_goal_options = rclcpp_action::Client<OffloadLocalization>::SendGoalOptions();
  localization_send_goal_options.goal_response_callback =
    std::bind(&Turtlebot4::localization_goal_response_callback, this, std::placeholders::_1);
  localization_send_goal_options.feedback_callback =
    std::bind(&Turtlebot4::localization_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  localization_send_goal_options.result_callback =
    std::bind(&Turtlebot4::localization_result_callback, this, std::placeholders::_1);
  // -----------------


  // planner_client_ = std::make_unique<turtlebot4::Turtlebot4Action<nav2_msgs::action::ComputePathToPose>>(node_handle_, "compute_path_to_pose");

  // controller_client_ = std::make_unique<turtlebot4::Turtlebot4Action<nav2_msgs::action::FollowPath>>(node_handle_, "follow_path");
  planner_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(this, "compute_path_to_pose");
  controller_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "follow_path");

  function_callbacks_ = {
    {"Offload Localization", std::bind(&Turtlebot4::offload_localization_function_callback, this)},
    {"Dock", std::bind(&Turtlebot4::dock_function_callback, this)},
    {"Undock", std::bind(&Turtlebot4::undock_function_callback, this)},
    {"Wall Follow Left", std::bind(&Turtlebot4::wall_follow_left_function_callback, this)},
    {"Wall Follow Right", std::bind(&Turtlebot4::wall_follow_right_function_callback, this)},
    {"EStop", std::bind(&Turtlebot4::estop_function_callback, this)},
    {"Power", std::bind(&Turtlebot4::power_function_callback, this)},
    {"RPLIDAR Start", std::bind(&Turtlebot4::rplidar_start_function_callback, this)},
    {"RPLIDAR Stop", std::bind(&Turtlebot4::rplidar_stop_function_callback, this)},
    {"OAKD Start", std::bind(&Turtlebot4::oakd_start_function_callback, this)},
    {"OAKD Stop", std::bind(&Turtlebot4::oakd_stop_function_callback, this)},
    {"Scroll Up", std::bind(&Turtlebot4::scroll_up_function_callback, this)},
    {"Scroll Down", std::bind(&Turtlebot4::scroll_down_function_callback, this)},
    {"Select", std::bind(&Turtlebot4::select_function_callback, this)},
    {"Back", std::bind(&Turtlebot4::back_function_callback, this)},
    {"Help", std::bind(&Turtlebot4::help_function_callback, this)},
  };

  // Set function callbacks
  add_button_function_callbacks();
  add_menu_function_callbacks();

  // Buttons
  buttons_ = std::make_unique<Buttons>(model_, turtlebot4_buttons_, node_handle_);

  if (model_ == Turtlebot4Model::STANDARD) {
    // Display
    display_ = std::make_unique<Display>(turtlebot4_menu_entries_, node_handle_);

    // Leds
    leds_ = std::make_unique<Leds>(node_handle_);
  }

  run();
}

/**
 * @brief Turtlebot4 Node run
 */
void Turtlebot4::run()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot4 %s running.", Turtlebot4ModelName[model_].c_str());

  if (model_ == Turtlebot4Model::STANDARD) {
    // Set Power LED
    leds_->set_led(POWER, GREEN);
    // Set Motors LED
    leds_->set_led(MOTORS, GREEN);

    display_timer(std::chrono::milliseconds(DISPLAY_TIMER_PERIOD));
    leds_timer(std::chrono::milliseconds(LEDS_TIMER_PERIOD));
  }

  latency_timer(std::chrono::milliseconds(LATENCY_TIMER_PERIOD));
  buttons_timer(std::chrono::milliseconds(BUTTONS_TIMER_PERIOD));
  wifi_timer(std::chrono::milliseconds(WIFI_TIMER_PERIOD));
  comms_timer(std::chrono::milliseconds(COMMS_TIMER_PERIOD));
  offload_timer(std::chrono::milliseconds(OFFLOAD_TIMER_PERIOD));
}


void Turtlebot4::latency_timer(const std::chrono::milliseconds timeout)
{
  latency_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      char buffer[128];
      static std::string command = "ping -W 2 -c 1 " + server_ip_;
      auto startTime = std::chrono::high_resolution_clock::now();
      system(command.c_str());
      auto endTime = std::chrono::high_resolution_clock::now();
      auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
      // Parse the command for the latency
      FILE* fp = popen(command.c_str(), "r");
      if (fp == nullptr) {
        std::cerr << "Failed to run command. Offloading turned off." << std::endl;
        offload_status_ = false;
      }

      // Read the command output into a string
      std::string output = "";
      while (fgets(buffer, sizeof(buffer), fp) != nullptr) {
          output += buffer;
      }

      // Close the file pointer
      fclose(fp);

      // Define the regex pattern to extract the latency
      std::regex pattern("time=([0-9]+\\.[0-9]+)");  // Pattern to match time=X.XXX (latency in ms)
      std::smatch match;
      if (std::regex_search(output, match, pattern) && match.size() > 1) {
        latency_ = std::stod(match.str(1));  // Extract the first capture group
      } else {
        offload_status_ = false; // No connection to server: can't offload
        std::cout << "No connection to server" << std::endl;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Elapsed time for command: %ld", elapsed_time);
      RCLCPP_INFO(this->get_logger(), "Current Latency: %f", latency_);
    });
}

/**
 * @brief Creates and runs timer to update display
 * @input timeout - Sets timer period in milliseconds
 */
void Turtlebot4::display_timer(const std::chrono::milliseconds timeout)
{
  display_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      static uint8_t counter = 0;
      // Force update display at 1hz
      if (++counter == 1000 / DISPLAY_TIMER_PERIOD) {
        display_->request_update();
        counter = 0;
      }
      display_->spin_once();
    });
}

/**
 * @brief Creates and runs timer to poll buttons
 * @input timeout - Sets timer period in milliseconds
 */
void Turtlebot4::buttons_timer(const std::chrono::milliseconds timeout)
{
  buttons_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      buttons_->spin_once();
    });
}

/**
 * @brief Creates and runs timer to poll buttons
 * @input timeout - Sets timer period in milliseconds
 */
void Turtlebot4::leds_timer(const std::chrono::milliseconds timeout)
{
  leds_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      leds_->spin_once();
    });
}

/**
 * @brief Creates and runs timer to check Wifi connection
 * @input timeout - Sets timer period in milliseconds
 */
void Turtlebot4::wifi_timer(const std::chrono::milliseconds timeout)
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

      if (this->model_ == Turtlebot4Model::STANDARD) {
        display_->set_ip(ip);

        if (ip == std::string(UNKNOWN_IP)) {
          leds_->set_led(WIFI, OFF);
        } else {
          leds_->set_led(WIFI, GREEN);
        }
      }
    });
}

/**
 * @brief Creates and runs timer for comms timeout
 * @input timeout - Sets timer period in milliseconds
 */
void Turtlebot4::comms_timer(const std::chrono::milliseconds timeout)
{
  comms_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      if (this->model_ == Turtlebot4Model::STANDARD) {
        leds_->set_led(COMMS, OFF);
      }
    });
}

void Turtlebot4::offload_timer(const std::chrono::milliseconds timeout)
{
  offload_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      RCLCPP_INFO(this->get_logger(), "Sending Action to Offload Server");
      offload_localization_function_callback();
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

void Turtlebot4::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
{
  RCLCPP_INFO(this->get_logger(), "Grabbing latest lidar data");
  latest_lidar_msg_ = *lidar_msg;
}

/**
 * @brief Battery subscription callback
 * @input battery_state_msg - Received message on battery topic
 */
void Turtlebot4::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_msg)
{
  if (battery_state_msg->percentage <= 0.12) {
    // Discharging
    if (!is_docked_) {
      // Wait 60s before powering off
      if (power_off_timer_ == nullptr || power_off_timer_->is_canceled()) {
        RCLCPP_WARN(this->get_logger(), "Low battery, starting power off timer");
        power_off_timer(std::chrono::milliseconds(POWER_OFF_TIMER_PERIOD));
      }
    } else {
      if (power_off_timer_ != nullptr && !power_off_timer_->is_canceled()) {
        RCLCPP_INFO(this->get_logger(), "Charging, canceling power off timer");
        power_off_timer_->cancel();
      }
    }
  } else if (battery_state_msg->percentage <= 0.2) {
    low_battery_animation();
  }

  // Set Battery LED on standard robots
  if (model_ == Turtlebot4Model::STANDARD) {
    display_->set_battery(battery_state_msg);

    // Set Battery LED
    if (battery_state_msg->percentage > 0.5) {
      leds_->set_led(BATTERY, GREEN);
    } else if (battery_state_msg->percentage > 0.2) {
      leds_->set_led(BATTERY, YELLOW);
    } else if (battery_state_msg->percentage > 0.12) {
      leds_->set_led(BATTERY, RED);
    } else {
      leds_->blink(BATTERY, 200, 0.5, RED);
    }
  }
}

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

void Turtlebot4::wheel_status_callback(
  const irobot_create_msgs::msg::WheelStatus::SharedPtr wheel_status_msg)
{
  wheels_enabled_ = wheel_status_msg->wheels_enabled;

  if (model_ == Turtlebot4Model::STANDARD) {
    // Reset Comms timer
    comms_timer_->cancel();
    leds_->set_led(COMMS, GREEN);
    comms_timer(std::chrono::milliseconds(COMMS_TIMER_PERIOD));

    // Set Motors LED
    if (wheels_enabled_) {
      leds_->set_led(MOTORS, GREEN);
    } else {
      leds_->set_led(MOTORS, OFF);
    }
  }
}



/**
 * @brief Sends lightring action goal
 */
void Turtlebot4::low_battery_animation()
{
  if (led_animation_client_ != nullptr) {
    auto animation_msg = std::make_shared<LedAnimation::Goal>();
    auto lightring_msg = irobot_create_msgs::msg::LightringLeds();

    for (int i = 0; i < 6; i++) {
      lightring_msg.leds[i].red = 255;
    }
    lightring_msg.header.stamp = this->get_clock()->now();
    lightring_msg.override_system = true;

    animation_msg->lightring = lightring_msg;
    animation_msg->animation_type = animation_msg->BLINK_LIGHTS;
    animation_msg->max_runtime.sec = 5;

    led_animation_client_->send_goal(animation_msg);
  } else {
    RCLCPP_ERROR(this->get_logger(), "LED animation client NULL");
  }
}


void Turtlebot4::offload_localization_function_callback()
{
  if (offload_localization_client_ != nullptr) {
    RCLCPP_INFO(this->get_logger(), "Offload Localization");
    // Initialize Goal message
    auto goal_msg = std::make_shared<OffloadLocalization::Goal>();

    goal_msg->robot_id = robot_id_; // Grab robot id from launch parameters
    goal_msg->status = offload_status_; // Grab whether we want to offload or not from status private variable

    goal_msg->initial_pose.pose.pose = initial_pose_; // Grab stored initial pose from turtlebot4
    goal_msg->initial_pose.pose.covariance = {
                                        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 1.0
                                       };

    goal_msg->laser_scan = latest_lidar_msg_; // Grab stored laser scan data from turtlebot4

    goal_msg->deadline_ms = 100; // LiDAR publishes at 10 Hz, so 100 ms deadline

    offload_localization_client_->send_goal(goal_msg);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Offload Localization Client NULL");
  }

}

// TODO :: ruiying
void Turtlebot4::localization_goal_response_callback(GoalHandleOffloadLocalization::SharedPtr goal_handle){
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void Turtlebot4::localization_feedback_callback(GoalHandleOffloadLocalization::SharedPtr,
  const std::shared_ptr<const OffloadLocalization::Feedback> feedback) {
  // TODO: Implement feedback
}

void Turtlebot4::localization_result_callback(const GoalHandleOffloadLocalization::WrappedResult & result){
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  // Handle the result_position here!
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.pose = initial_pose_;

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = result.result->final_pose.header;
  goal_pose.pose = result.result->final_pose.pose.pose;

  sendComputePathToPose(start_pose, goal_pose); // start and end
  //sendComputePathToPose(initial_pose_, result.result->final_pose);
}

void Turtlebot4::sendComputePathToPose(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
  // Create the goal message
  auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
  goal_msg.start = start;
  goal_msg.goal = goal;

  // Send the goal asynchronously
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&Turtlebot4::pathResultCallback, this, std::placeholders::_1);

  planner_client_->async_send_goal(goal_msg, send_goal_options);
  // planner_client_->send_goal(goal_msg);
}

void Turtlebot4::pathResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult &result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Path successfully computed!");
      // Process the path
      sendFollowPath(result.result->path);
  } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to compute path!");
  }
}

void Turtlebot4::sendFollowPath(const nav_msgs::msg::Path &path) {
  // Create the goal message for FollowPath
  auto goal_msg = nav2_msgs::action::FollowPath::Goal();
  goal_msg.path = path;
  goal_msg.controller_id = "";  // Use the default controller
  goal_msg.goal_checker_id = ""; // Use the default goal checker

  // Send the goal asynchronously
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&Turtlebot4::followPathResultCallback, this, std::placeholders::_1);

  //follow_path_client_->async_send_goal(goal_msg, send_goal_options);
  controller_client_->async_send_goal(goal_msg, send_goal_options);
  // controller_client_->send_goal(goal_msg);
}

void Turtlebot4::followPathResultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult &result) {

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "FollowPath action succeeded!");
      // Process the velocity command if needed
      // The controller generates and sends velocity commands directly to the robot
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_ERROR(this->get_logger(), "FollowPath action was aborted!");
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(this->get_logger(), "FollowPath action was canceled!");
  } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown result code from FollowPath!");
  }
}

// ---------------------------------------


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
 * @brief Sends follow action goal
 */
void Turtlebot4::wall_follow_left_function_callback()
{
  if (wall_follow_client_ != nullptr && !is_docked_) {
    RCLCPP_INFO(this->get_logger(), "Wall Follow Left");
    auto goal_msg = std::make_shared<WallFollow::Goal>();
    auto runtime = builtin_interfaces::msg::Duration();
    runtime.sec = 10;
    goal_msg->follow_side = WallFollow::Goal::FOLLOW_LEFT;
    goal_msg->max_runtime = runtime;
    wall_follow_client_->send_goal(goal_msg);
  } else if (is_docked_) {
    RCLCPP_ERROR(this->get_logger(), "Undock before following wall");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Follow client NULL");
  }
}

/**
 * @brief Sends follow action goal
 */
void Turtlebot4::wall_follow_right_function_callback()
{
  if (wall_follow_client_ != nullptr && !is_docked_) {
    RCLCPP_INFO(this->get_logger(), "Wall Follow Right");
    auto goal_msg = std::make_shared<WallFollow::Goal>();
    auto runtime = builtin_interfaces::msg::Duration();
    runtime.sec = 10;
    goal_msg->follow_side = WallFollow::Goal::FOLLOW_RIGHT;
    goal_msg->max_runtime = runtime;
    wall_follow_client_->send_goal(goal_msg);
  } else if (is_docked_) {
    RCLCPP_ERROR(this->get_logger(), "Undock before following wall");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Follow client NULL");
  }
}

/**
 * @brief Sends estop service request
 */
void Turtlebot4::estop_function_callback()
{
  if (estop_client_ != nullptr) {
    // Make request
    auto request = std::make_shared<EStop::Request>();

    request->e_stop_on = wheels_enabled_;

    if (request->e_stop_on) {
      RCLCPP_INFO(this->get_logger(), "Setting EStop");
    } else {
      RCLCPP_INFO(this->get_logger(), "Clearing EStop");
    }

    estop_client_->make_request(request);
  } else {
    RCLCPP_ERROR(this->get_logger(), "EStop client NULL");
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
 * @brief Start RPLIDAR
 */
void Turtlebot4::rplidar_start_function_callback()
{
  if (rplidar_start_client_ != nullptr) {
    auto request = std::make_shared<EmptySrv::Request>();
    rplidar_start_client_->make_request(request);
    RCLCPP_INFO(this->get_logger(), "RPLIDAR started");
  } else {
    RCLCPP_ERROR(this->get_logger(), "RPLIDAR client NULL");
  }
}

/**
 * @brief Stop RPLIDAR
 */
void Turtlebot4::rplidar_stop_function_callback()
{
  if (rplidar_stop_client_ != nullptr) {
    auto request = std::make_shared<EmptySrv::Request>();
    rplidar_stop_client_->make_request(request);
    RCLCPP_INFO(this->get_logger(), "RPLIDAR stopped");
  } else {
    RCLCPP_ERROR(this->get_logger(), "RPLIDAR client NULL");
  }
}

/**
 * @brief Start OAK-D
 */
void Turtlebot4::oakd_start_function_callback()
{
  if (oakd_start_client_ != nullptr) {
    auto request = std::make_shared<TriggerSrv::Request>();
    oakd_start_client_->make_request(request);
    RCLCPP_INFO(this->get_logger(), "OAKD started");
  } else {
    RCLCPP_ERROR(this->get_logger(), "OAKD client NULL");
  }
}

/**
 * @brief Stop OAK-D
 */
void Turtlebot4::oakd_stop_function_callback()
{
  if (oakd_stop_client_ != nullptr) {
    auto request = std::make_shared<TriggerSrv::Request>();
    oakd_stop_client_->make_request(request);
    RCLCPP_INFO(this->get_logger(), "OAKD stopped");
  } else {
    RCLCPP_ERROR(this->get_logger(), "OAKD client NULL");
  }
}

void Turtlebot4::scroll_up_function_callback()
{
  if (model_ == Turtlebot4Model::STANDARD) {
    display_->scroll_up();
  }
}

void Turtlebot4::scroll_down_function_callback()
{
  if (model_ == Turtlebot4Model::STANDARD) {
    display_->scroll_down();
  }
}

void Turtlebot4::select_function_callback()
{
  if (model_ == Turtlebot4Model::STANDARD) {
    display_->select();
  }
}

void Turtlebot4::back_function_callback()
{
  if (model_ == Turtlebot4Model::STANDARD) {
    display_->back();
  }
}

void Turtlebot4::help_function_callback()
{
  if (model_ == Turtlebot4Model::STANDARD) {
    std::vector<std::string> help_message;
    help_message.push_back("Button usage:");
    help_message.push_back("1:" + turtlebot4_buttons_[Turtlebot4ButtonEnum::HMI_1].short_function_);
    help_message.push_back("2:" + turtlebot4_buttons_[Turtlebot4ButtonEnum::HMI_2].short_function_);
    help_message.push_back("3:" + turtlebot4_buttons_[Turtlebot4ButtonEnum::HMI_3].short_function_);
    help_message.push_back("4:" + turtlebot4_buttons_[Turtlebot4ButtonEnum::HMI_4].short_function_);
    display_->show_message(help_message);
  }
}

/**
 * @brief Invalid or empty function specified in ROS parameters - do nothing
 */
void Turtlebot4::unused_function_callback()
{
}

/**
 * @brief Callback for when a function call is executed
 */
void Turtlebot4::function_call_callback(std::string function_name)
{
  std_msgs::msg::String msg;
  msg.data = function_name;

  function_call_pub_->publish(msg);
}

/**
 * @brief Creates action or service clients and adds appropriate callbacks
 * for each function declared in ROS parameters
 */
void Turtlebot4::add_button_function_callbacks()
{
  for (auto & button : turtlebot4_buttons_) {
    // Short press function
    if (function_callbacks_.find(button.short_function_) != function_callbacks_.end()) {
      button.short_cb_ = function_callbacks_[button.short_function_];
    } else {
      button.short_cb_ = std::bind(&Turtlebot4::unused_function_callback, this);
    }

    // Long press function
    if (function_callbacks_.find(button.long_function_) != function_callbacks_.end()) {
      button.long_cb_ = function_callbacks_[button.long_function_];
    } else {
      button.long_cb_ = std::bind(&Turtlebot4::unused_function_callback, this);
    }

    button.function_call_cb_ = std::bind(
      &Turtlebot4::function_call_callback, this,
      std::placeholders::_1);
  }
}

/**
 * @brief Creates action or service clients and adds appropriate callbacks
 * for each function declared in ROS parameters
 */
void Turtlebot4::add_menu_function_callbacks()
{
  for (auto & entry : turtlebot4_menu_entries_) {
    if (function_callbacks_.find(entry.name_) != function_callbacks_.end()) {
      entry.cb_ = function_callbacks_[entry.name_];
    } else {
      entry.cb_ = std::bind(&Turtlebot4::unused_function_callback, this);
    }
    entry.function_call_cb_ = std::bind(
      &Turtlebot4::function_call_callback, this,
      std::placeholders::_1);
  }
}

/**
 * @brief Get IP of network interface specified in ROS parameters
 */
std::string Turtlebot4::get_ip()
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

vector<long> Turtlebot4::get_cpu_times_helper(){
  std::ifstream file("/proc/stat");
  std::string line;
  std::vector<long> data(8, 0); // to store the 8 CPU times

  if (file.is_open()) {
      std::getline(file, line);
      std::istringstream iss(line);
      std::string cpu;
      iss >> cpu;
      
      // Read the 8 CPU times
      for (int i = 0; i < 8; ++i) {
          iss >> data[i];
      }
  }
  return data;
}

double Turtlebot4::calculate_cpu_usage(){
  std::ifstream file("/proc/stat");
  std::string line;
  
  // These will hold the individual CPU times
  long user, nice, system, idle, iowait, irq, softirq, steal;

  if (file.is_open()) {
      std::getline(file, line);
      std::istringstream iss(line);
      std::string cpu;
      
      // Skip the "cpu" prefix in the first column
      iss >> cpu;
      
      // Read the CPU times from /proc/stat
      iss >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
      
      // Calculate total time and active time
      long total_time = user + nice + system + idle + iowait + irq + softirq + steal;
      long active_time = user + nice + system + irq + softirq + steal;
      
      // Calculate CPU utilization as a percentage
      cpu_load = static_cast<double>(active_time) / total_time * 100.0;
      return cpu_load;
  }

  return 0.0;  // failed to open file
}