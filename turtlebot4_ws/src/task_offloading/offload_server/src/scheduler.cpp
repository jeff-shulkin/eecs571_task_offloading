/* Author: Jeffrey Shulkin
   Purpose: The purpose of this code is to taken in ROS2 actions from
   the client, submit them to a queue, and using one of several scheduling
   algorithms or heuristics to select and compute, and submit the results 
   back to the action server to send to the client.
*/

#ifndef TASK_OFFLOAD_SCHEDULER_CPP_
#define TASK_OFFLOAD_SCHEDULER_CPP_

#include "offload_server/scheduler.hpp" // Includes C++ boost and ROS2 libraries
#include <chrono> // for high_resolution_timer
#include <thread>
#include <functional>
#include <cstdint>
#include <iostream>

void JobScheduler::add_job(ROS2Job j) {
	fifo_sched_.push_back(j);
}

void JobScheduler::remove_job(ROS2Job j) {
	fifo_sched_.erase(std::remove_if(fifo_sched_.begin(), fifo_sched_.end(), [j](ROS2Job job) {return job.agent_id == j.agent_id && job.task_id == j.task_id;}));
}

void JobScheduler::execute() {
	ROS2Job curr_job = fifo_sched_.front();
        fifo_sched_.pop_front();
	// TODO: implement Localization processing; relay Goal to nav2 nodes
}

#endif
