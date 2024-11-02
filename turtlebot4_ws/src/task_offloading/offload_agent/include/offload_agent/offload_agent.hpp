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
#include "task_action_interfaces/action/offloadamcl.hpp"

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

class OffloadAgent : private turtlebot4::Turtlebot4
{
public:
  // Type alias for actions and services
  using AMCL = task_action_interfaces::action::Offloadamcl;
  //using Costmap = irobot_create_msgs::action::OffloadCostmap;

  // Constructor and Destructor
  OffloadAgent();
  virtual ~OffloadAgent() {}

private:
  void run();

  // Subscription callbacks

  // Actions
  std::unique_ptr<turtlebot4::Turtlebot4Action<AMCL>> offload_amcl_client_;
  //std::unique_ptr<Turtlebot4Action<Costmap>> offload_costmap_client_;

};

}  // namespace offload_agent

#endif  // OFFLOAD_AGENT_NODE_HPP_
