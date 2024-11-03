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

#include <turtlebot4_node/turtlebot4.hpp>
#include <turtlebot4_node/action.hpp>
#include <turtlebot4_node/leds.hpp>
#include <turtlebot4_node/display.hpp>
#include <turtlebot4_node/service.hpp>
#include <turtlebot4_node/utils.hpp>
#include <task_action_interfaces/action/offloadamcl.hpp>

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

class OffloadAgent : public rclcpp::Node
{
public:
  // Type alias for actions and services
  using AMCL = task_action_interfaces::action::Offloadamcl;
  //using Costmap = irobot_create_msgs::action::OffloadCostmap;
  using Dock = irobot_create_msgs::action::Dock;
  using Undock = irobot_create_msgs::action::Undock;
  using WallFollow = irobot_create_msgs::action::WallFollow;
  using LedAnimation = irobot_create_msgs::action::LedAnimation;
  using EStop = irobot_create_msgs::srv::EStop;
  using Power = irobot_create_msgs::srv::RobotPower;
  using EmptySrv = std_srvs::srv::Empty;
  using TriggerSrv = std_srvs::srv::Trigger;

  // Constructor and Destructor
  OffloadAgent();
  virtual ~OffloadAgent() {}

private:
  void run();

 // Subscription callbacks
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_msg);
  void dock_status_callback(
    const irobot_create_msgs::msg::DockStatus::SharedPtr dock_status_msg);
  void wheel_status_callback(
    const irobot_create_msgs::msg::WheelStatus::SharedPtr wheel_status_msg);
  void joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr joy_msg);

  // Function callbacks
  void offload_amcl_function_callback();

  void dock_function_callback();
  void undock_function_callback();
  void wall_follow_left_function_callback();
  void wall_follow_right_function_callback();
  void estop_function_callback();
  void power_function_callback();
  void rplidar_start_function_callback();
  void rplidar_stop_function_callback();
  void oakd_start_function_callback();
  void oakd_stop_function_callback();
  void scroll_up_function_callback();
  void scroll_down_function_callback();
  void select_function_callback();
  void back_function_callback();
  void help_function_callback();
  void unused_function_callback();
  void function_call_callback(std::string function_name);

  void add_button_function_callbacks();
  void add_menu_function_callbacks();

  void low_battery_animation();

  // Run display timer
  void display_timer(const std::chrono::milliseconds timeout);

  // Run buttons timer
  void buttons_timer(const std::chrono::milliseconds timeout);

  // Run leds timer
  void leds_timer(const std::chrono::milliseconds timeout);

  // Run wifi timer
  void wifi_timer(const std::chrono::milliseconds timeout);

  // Run comms timer
  void comms_timer(const std::chrono::milliseconds timeout);

  // Run power off timer
  void power_off_timer(const std::chrono::milliseconds timeout);

  // IP
  std::string get_ip();
  std::string wifi_interface_;

  // Node
  rclcpp::Node::SharedPtr node_handle_;

  // Turtlebot4 Functions
  std::vector<turtlebot4::Turtlebot4Button> turtlebot4_buttons_;
  std::vector<turtlebot4::Turtlebot4MenuEntry> turtlebot4_menu_entries_;
  std::map<std::string, turtlebot4::turtlebot4_function_callback_t> function_callbacks_;
  std::map<turtlebot4::Turtlebot4ButtonEnum, std::string> button_parameters_;

  // Display
  std::unique_ptr<turtlebot4::Display> display_;

  // Buttons
  std::unique_ptr<turtlebot4::Buttons> buttons_;

  // Leds
  std::unique_ptr<turtlebot4::Leds> leds_;

  // Actions
  std::unique_ptr<turtlebot4::Turtlebot4Action<AMCL>> offload_amcl_client_;
  //std::unique_ptr<turtlebot4::Turtlebot4Action<Costmap>> offload_costmap_client_;
  std::unique_ptr<turtlebot4::Turtlebot4Action<Dock>> dock_client_;
  std::unique_ptr<turtlebot4::Turtlebot4Action<Undock>> undock_client_;
  std::unique_ptr<turtlebot4::Turtlebot4Action<WallFollow>> wall_follow_client_;
  std::unique_ptr<turtlebot4::Turtlebot4Action<LedAnimation>> led_animation_client_;

  // Services
  std::unique_ptr<turtlebot4::Turtlebot4Service<EStop>> estop_client_;
  std::unique_ptr<turtlebot4::Turtlebot4Service<Power>> power_client_;
  std::unique_ptr<turtlebot4::Turtlebot4EmptyService<EmptySrv>> rplidar_start_client_;
  std::unique_ptr<turtlebot4::Turtlebot4EmptyService<EmptySrv>> rplidar_stop_client_;
  std::unique_ptr<turtlebot4::Turtlebot4Service<TriggerSrv>> oakd_start_client_;
  std::unique_ptr<turtlebot4::Turtlebot4Service<TriggerSrv>> oakd_stop_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr display_timer_;
  rclcpp::TimerBase::SharedPtr buttons_timer_;
  rclcpp::TimerBase::SharedPtr leds_timer_;
  rclcpp::TimerBase::SharedPtr wifi_timer_;
  rclcpp::TimerBase::SharedPtr comms_timer_;
  rclcpp::TimerBase::SharedPtr power_off_timer_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr dock_status_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::WheelStatus>::SharedPtr wheel_status_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ip_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr function_call_pub_;

  // Store current wheels state
  bool wheels_enabled_;

  // Store current dock state
  bool is_docked_;

  // Store power saver mode
  bool power_saver_;

  // Turtlebot4 Model
  turtlebot4::Turtlebot4Model model_;

};

}  // namespace offload_agent

#endif  // OFFLOAD_AGENT_NODE_HPP_
