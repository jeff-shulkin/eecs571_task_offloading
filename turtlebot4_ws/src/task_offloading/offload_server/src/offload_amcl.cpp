#ifndef OFFLOAD_AMCL_ACTION_SERVER_HPP_
#define OFFLOAD_AMCL_ACTION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "offload_server/offload_server.hpp"
#include "task_action_interfaces/action/offloadamcl.hpp"

using namespace offload_server;
using AMCL = task_action_interfaces::action::Offloadamcl;
using GoalHandleOffloadAMCL = rclcpp_action::ServerGoalHandle<AMCL>;

rclcpp_action::GoalResponse OffloadServer::handle_offload_amcl_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AMCL::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->request);
    (void)uuid;
    if (goal-> > 100) {
    return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OffloadServer::handle_offload_amcl_cancel(
    const std::shared_ptr<GoalHandleOffloadAMCL> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OffloadServer::handle_offload_amcl_accepted(const std::shared_ptr<GoalHandleOffloadAMCL> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&offload_server::OffloadServer::offload_amcl_execute, this, _1), goal_handle}.detach();
}

void OffloadServer::offload_amcl_execute(const std::shared_ptr<GoalHandleOffloadAMCL> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<AMCL::Feedback>();
    auto result = std::make_shared<AMCL::Result>();
    printf("Offloading AMCL Hello World!\n");

    for (int i = 1; (i <= goal->request) && rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
        result->result = -1;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }

    // Update sequence
    //feedback->feedback = i;
    // Publish feedback
    //goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "TODO: Publish feedback");

    loop_rate.sleep();
    }

    // Check if goal is result
    if (rclcpp::ok()) {
    result->result = 1;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

#endif
