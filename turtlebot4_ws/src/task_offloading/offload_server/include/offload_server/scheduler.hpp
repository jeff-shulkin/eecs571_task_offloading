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
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

enum class TaskType {
        DEBUG = 0,
        OFFLOAD_AMCL = 1,
        OFFLOAD_COSTMAP = 2
};

struct ROS2Job {
        std::string agent_id;
        TaskType task_id;
	std::function<void (void)> callback;
        std::chrono::milliseconds deadline;
};

class JobScheduler {

	public:
		void add_job();
		void remove_task(ROS2Job j);
		void execute();
		void print_all_jobs();

	private:
                rclcpp::TimerBase::SharedPtr amcl_timer_;
                rclcpp::TimerBase::SharedPtr costmap_timer_;
                rclcpp::TimerBase::SharedPtr wifi_timer_;
                // All scheduler data structures
		std::deque<ROS2Job> fifo_sched_;

}; // JobScheduler

#endif // TASK_OFFLOAD_SCHEDULER_HPP_

