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
#include <function>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

enum class availableJobs {
	ROS2_DEBUG = 0,
	NAV2_AMCL = 1,
	NAV2_COSTMAP2D = 2
} // availableJobs

class ROS2Job {

	public:
		ROS2Job(int job_num, availableJobs curr_job, float curr_deadline);
		void run();

	private:
		uint_32 num = 0xFFFFFFFF; // Which job am I? (debugging)
		availableJobs job_type = ROS2_DEBUG; // ROS2/Nav2 job type
		std::function<void()> func = std::function<void()>(); // Exact member function being executed
		float deadline_ms = -1.0; // Job deadline in milliseconds

}; // ROS2Job

class taskScheduler {

	public:
		void add_job(ROS2Job j);
		void remove_job(ROS2Job j);
		void execute();
		void print_all_jobs();

	private:
		std::queue<ROS2Job> job_queue;

}; // TaskScheduler

#endif // TASK_OFFLOAD_SCHEDULER_HPP_

