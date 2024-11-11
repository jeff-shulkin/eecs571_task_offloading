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

// TODO: This looks out of date???
enum class TaskType {
        DEBUG = 0,
        OFFLOAD_AMCL = 1,
        OFFLOAD_COSTMAP = 2
};

// TODO: Figure out wtf is function<void (void)> callback
struct ROS2Job {
        std::string agent_id;
	std::function<void (void)> callback;
        std::chrono::milliseconds deadline;
};

// Custom priority queue job comparator
// struct ROS2JobCompare {
//     bool operator()(ROS2Job& a, ROS2Job& b){
//             return a->callback. > b;
//     }
// };

class JobScheduler {

	public:
		void add_job(ROS2Job j);
		void remove_task(ROS2Job j);
		void execute();
		void print_all_jobs();

	private:
                rclcpp::TimerBase::SharedPtr amcl_timer_;
                rclcpp::TimerBase::SharedPtr costmap_timer_;
                rclcpp::TimerBase::SharedPtr wifi_timer_;
  bool 
                // All scheduler data structures
		// std::priority_queue<ROS2Job, std::deque<ROS2Job>, ROS2JobCompare> fifo_sched_;
		std::queue<ROS2Job> fifo_sched_;

}; // JobScheduler

#endif // TASK_OFFLOAD_SCHEDULER_HPP_

