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

ROS2Job::ROS2Job(int job_num, availableJobs curr_job, float curr_deadline):
                num(job_num), job(curr_job), deadline_ms(curr_deadline) {
		switch(curr_job) : {
			case ROS2_DEBUG: {
				func = std::function<void()> debug([] { cout << "Debug Message." << "\n";});
			}

			case NAV2_AMCL: {
				func = std::function<void()> debug([] { cout << "Debug Message." << "\n";});
			}

			case NAV2_COSTMAP_2D: {
				 func = std::function<void()> debug([] { cout << "Debug Message." << "\n";});
			}
		}
}

void taskScheduler::schedule(ROS2Job j) {
	job_queue.push(j);
}

void taskScheduler::execute() {
	ROS2Job curr_job = job_queue.pop();
	
}
void taskScheduler::print_all_jobs() {
	for (int i = 0; i < job_queue.size(); ++i) {
		ROS2Job curr_job = job_queue.pop(); // retrieve current job
		std::cout << "Job number: " << curr_job.num << "\n";
		std::cout << "Job Type: " << curr_job.job << "\n";
		std::cout << "Job Deadline: " << curr_job.deadline << "\n";
	}
}


void sched_callback(ROS2Job j) {
	std::cout << "Printing job number " << j.job_num << "\n";
	std::cout << "EECS 571 has made me an alcoholic.\n";
}

int main() {
	// Initialize task scheduler
	taskScheduler sched;
	for (int i = 0; i < 100; ++i) {
		float deadline = 50.0; // 50 ms arbitrary deadline
		Ros2Job debug = Ros2Job(i, availableJobs.ROS_DEBUG, deadline);
		taskScheduler.job_queue.push_back(debug);
		taskScheduler.execute();
	}
}
#endif  // NAV2_COSTMAP_2D__COSTMAP_2D_ROS_CPP_