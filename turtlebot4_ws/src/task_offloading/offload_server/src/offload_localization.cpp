#ifndef OFFLOAD_LOCALIZATION_ACTION_SERVER_HPP_
#define OFFLOAD_LOCALIZATION_ACTION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "offload_server/offload_server.hpp"
#include "task_action_interfaces/action/offloadlocalization.hpp"

using namespace offload_server;

rclcpp_action::GoalResponse OffloadServer::handle_offload_localization_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const OffloadServer::OffloadLocalization::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request for robot %s", goal->robot_id.c_str());
    (void)uuid;
    if (goal->status == false) {
    // TODO: CANCEL JOBS ON SCHEDULER FROM THIS ROBOT ID
    return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OffloadServer::handle_offload_localization_cancel(
    const std::shared_ptr<GoalHandleOffloadLocalization> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OffloadServer::handle_offload_localization_accepted(const std::shared_ptr<GoalHandleOffloadLocalization> goal_handle)
{
    using namespace std::placeholders;

    // create a new job based on the received goal
    const auto goal = goal_handle->get_goal();
    ROS2Job new_job_entry(goal->robot_id, goal->deadline_ms, goal->laser_scan, goal->initial_pose);

    // add new job to the scheduler;
    sched_.add_job(new_job_entry);
    
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // std::thread{std::bind(&OffloadServer::offload_localization_execute, this, _1), goal_handle}.detach();
    
}

void OffloadServer::offload_localization_execute(const std::shared_ptr<OffloadServer::GoalHandleOffloadLocalization> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<OffloadServer::OffloadLocalization::Feedback>();
    auto result = std::make_shared<OffloadServer::OffloadLocalization::Result>();

    for (int i = 1; rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }

    
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "TODO: Publish feedback");

    loop_rate.sleep();
    }

    // Check if goal is result
    if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

#endif
