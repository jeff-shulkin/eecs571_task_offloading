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
      RCLCPP_INFO(this->get_logger(), "Goal rejected");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
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
    RCLCPP_INFO(this->get_logger(), "Goal accepted, adding job to scheduler");

    // create a new job based on the received goal
    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(this->get_logger(), "Server received goal: [%f, %f]", goal->initial_pose.pose.pose.position.x, goal->initial_pose.pose.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Server received laser frame id: %s", goal->laser_scan.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Server received odom frame id: %s", goal->odom.header.frame_id.c_str());

    // create new job entry based on the goal
    ROS2Job new_job_entry = {goal_handle, goal->robot_id, goal->job_id, std::chrono::milliseconds(goal->deadline_ms), goal->odom, goal->laser_scan, goal->initial_pose, goal->map};

    // add new job to the scheduler;
    add_job(new_job_entry);

    offload_status_ = goal->status;
    uint8_t nav2_cmd;
    if(!offload_status_) {
        // No Offload, stop server nav2
        nav2_cmd = 10; // pause
    } else {
        nav2_cmd = 11; // resume
    }
    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = nav2_cmd;
    auto future = nav2_localization_manager_client_->async_send_request(request);

}

#endif
