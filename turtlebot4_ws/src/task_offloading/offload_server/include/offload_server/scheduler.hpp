/* Author: Jeffrey Shulkin
   Purpose: Interface for the ROS2 task scheduler. This file includes the
   job definition, and task submission and reception interface to the
   ROS2 action server/cient.
*/

#ifndef TASK_OFFLOAD_SCHEDULER_HPP_
#define TASK_OFFLOAD_SCHEDULER_HPP_

#include <string>
#include <queue>
#include <vector>
#include <functional>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "offload_server/offload_server.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


namespace offload_server { class OffloadServer; }

struct ROS2Job {
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<task_action_interfaces::action::Offloadlocalization>> goal_handle;
        std::string agent_id;
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

class JobScheduler {

	public:
                JobScheduler(offload_server::OffloadServer *server);
		void add_job(ROS2Job j);
		void remove_task(ROS2Job j);
		void execute();
		void print_all_jobs();

	private:
                // hold reference to the offload server
                offload_server::OffloadServer server_;
                rclcpp::TimerBase::SharedPtr amcl_timer_;
                rclcpp::TimerBase::SharedPtr costmap_timer_;
                rclcpp::TimerBase::SharedPtr wifi_timer_;
                // All scheduler data structures
		// std::priority_queue<ROS2Job, std::deque<ROS2Job>, ROS2JobCompare> fifo_sched_;
		std::queue<ROS2Job> fifo_sched_;

}; // JobScheduler

#endif // TASK_OFFLOAD_SCHEDULER_HPP_

