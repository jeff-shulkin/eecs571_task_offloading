#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "task_action_interfaces/actions/offload_amcl.hpp"

class OffloadAMCLActionServer
{
    public:
    using ActionOffloadAMCL = task_action_interfaces::actions::offload_amcl;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionOffloadAMCL>;

    explicit ActionServer(const std::shared_ptr<::rclcpp::Node> node_handle)
    {
        printf("AMCL Action Server Started!\n");

        this->action_server_ = rclcpp_action::create_server<ActionHello>(
        node_handle->get_node_base_interface(),
        node_handle->get_node_clock_interface(),
        node_handle->get_node_logging_interface(),
        node_handle->get_node_waitables_interface(),
        "offload_amcl",
        std::bind(&OffloadAMCLActionServer::handle_goal, node_handle, std::placeholders::_1, std::placeholders::_2),
        std::bind(&OffloadAMCLActionServer::handle_cancel, node_handle, std::placeholders::_1),
        std::bind(&OffloadAMCLActionServer::handle_accepted, node_handle, std::placeholders::_1));
    }

    private:
    rclcpp_action::Server<Action>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ActionHello::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->request);
        (void)uuid;
        if (goal-> > 100) {
        return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&OffloadAMCLActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ActionHello::Feedback>();
        auto result = std::make_shared<ActionHello::Result>();

        for (int i = 1; (i <= goal->request) && rclcpp::ok(); ++i) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->result = -1;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        printf("ActionHello World!\n");

        // Update sequence
        feedback->feedback = i;
        // Publish feedback
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
        }

        // Check if goal is result
        if (rclcpp::ok()) {
        result->result = 1;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};  // class ActionServer

