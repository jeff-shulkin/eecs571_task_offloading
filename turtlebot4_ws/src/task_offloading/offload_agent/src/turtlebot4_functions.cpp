#include "offload_agent/offload_agent.hpp"


using namespace turtlebot4;

/**
 * @brief Creates and runs timer to update display
 * @input timeout - Sets timer period in milliseconds
 */
void OffloadAgent::display_timer(const std::chrono::milliseconds timeout)
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
void OffloadAgent::buttons_timer(const std::chrono::milliseconds timeout)
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
void OffloadAgent::leds_timer(const std::chrono::milliseconds timeout)
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
void OffloadAgent::wifi_timer(const std::chrono::milliseconds timeout)
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
void OffloadAgent::comms_timer(const std::chrono::milliseconds timeout)
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

/**
 * @brief Creates and runs timer for powering off the robot when low battery
 * @input timeout - Sets timer period in milliseconds
 */
void OffloadAgent::power_off_timer(const std::chrono::milliseconds timeout)
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
void OffloadAgent::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_msg)
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

void OffloadAgent::dock_status_callback(
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

void OffloadAgent::wheel_status_callback(
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
void OffloadAgent::low_battery_animation()
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

/**
 * @brief Sends dock action goal
 */
void OffloadAgent::dock_function_callback()
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
void OffloadAgent::undock_function_callback()
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
void OffloadAgent::wall_follow_left_function_callback()
{
  if (wall_follow_client_ != nullptr) {
    RCLCPP_INFO(this->get_logger(), "Wall Follow Left");
    auto goal_msg = std::make_shared<WallFollow::Goal>();
    auto runtime = builtin_interfaces::msg::Duration();
    runtime.sec = 10;
    goal_msg->follow_side = WallFollow::Goal::FOLLOW_LEFT;
    goal_msg->max_runtime = runtime;
    wall_follow_client_->send_goal(goal_msg);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Follow client NULL");
  }
}

/**
 * @brief Sends follow action goal
 */
void OffloadAgent::wall_follow_right_function_callback()
{
  if (wall_follow_client_ != nullptr) {
    RCLCPP_INFO(this->get_logger(), "Wall Follow Right");
    auto goal_msg = std::make_shared<WallFollow::Goal>();
    auto runtime = builtin_interfaces::msg::Duration();
    runtime.sec = 10;
    goal_msg->follow_side = WallFollow::Goal::FOLLOW_RIGHT;
    goal_msg->max_runtime = runtime;
    wall_follow_client_->send_goal(goal_msg);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Follow client NULL");
  }
}

/**
 * @brief Sends estop service request
 */
void OffloadAgent::estop_function_callback()
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
void OffloadAgent::power_function_callback()
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
void OffloadAgent::rplidar_start_function_callback()
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
void OffloadAgent::rplidar_stop_function_callback()
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
void OffloadAgent::oakd_start_function_callback()
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
void OffloadAgent::oakd_stop_function_callback()
{
  if (oakd_stop_client_ != nullptr) {
    auto request = std::make_shared<TriggerSrv::Request>();
    oakd_stop_client_->make_request(request);
    RCLCPP_INFO(this->get_logger(), "OAKD stopped");
  } else {
    RCLCPP_ERROR(this->get_logger(), "OAKD client NULL");
  }
}

void OffloadAgent::scroll_up_function_callback()
{
  if (model_ == Turtlebot4Model::STANDARD) {
    display_->scroll_up();
  }
}

void OffloadAgent::scroll_down_function_callback()
{
  if (model_ == Turtlebot4Model::STANDARD) {
    display_->scroll_down();
  }
}

void OffloadAgent::select_function_callback()
{
  if (model_ == Turtlebot4Model::STANDARD) {
    display_->select();
  }
}

void OffloadAgent::back_function_callback()
{
  if (model_ == Turtlebot4Model::STANDARD) {
    display_->back();
  }
}

void OffloadAgent::help_function_callback()
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
void OffloadAgent::unused_function_callback()
{
}

/**
 * @brief Callback for when a function call is executed
 */
void OffloadAgent::function_call_callback(std::string function_name)
{
  std_msgs::msg::String msg;
  msg.data = function_name;

  function_call_pub_->publish(msg);
}

/**
 * @brief Creates action or service clients and adds appropriate callbacks
 * for each function declared in ROS parameters
 */
void OffloadAgent::add_button_function_callbacks()
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
void OffloadAgent::add_menu_function_callbacks()
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
std::string OffloadAgent::get_ip()
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
