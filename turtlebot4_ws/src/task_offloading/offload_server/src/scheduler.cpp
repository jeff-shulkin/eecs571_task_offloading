/* Author: Jeffrey Shulkin
   Purpose: The purpose of this code is to taken in ROS2 actions from
   the client, submit them to a queue, and using one of several scheduling
   algorithms or heuristics to select and compute, and submit the results 
   back to the action server to send to the client.
*/

#ifndef TASK_OFFLOAD_SCHEDULER_CPP_
#define TASK_OFFLOAD_SCHEDULER_CPP_

#include "offload_server/scheduler.hpp" // Includes C++ boost and ROS2 libraries
#include "offload_server/offloader_server.hpp" //Access Nav2 flags
#include <chrono> // for high_resolution_timer
#include <thread>
#include <functional>
#include <cstdint>
#include <iostream>

void JobScheduler::add_job(ROS2Job j) {
	fifo_sched_.push(j);
}

//void JobScheduler::remove_job(ROS2Job j) {
//	fifo_sched_.erase(std::remove_if(fifo_sched_.begin(), fifo_sched_.end(), [j](ROS2Job job) {return job.agent_id == j.agent_id && job.task_id == j.task_id;}));
//}

void JobScheduler::execute() {
  if(!fifo_sched_.empty()) {
	ROS2Job curr_job = fifo_sched_.front();
	nav2_ipose_pub_->publish(curr_job.ipose);
	nav2_laser_scan_pub_->publish(curr_job.laserscan);
	while(!FPOSE_READY) { continue; } // CAUTION: busy looping is bad
	FPOSE_READY = false;
	auto result = std::make_shared<OffloadServer:Offloadlocalization::Result>();
	result->success = true;
	result->final_pose = offload_amcl_rpose;
	result->exec_time = 0.0f;
	curr_job.goal_handle->succeed(result);
        fifo_sched_.pop();
  }
}

#endif
