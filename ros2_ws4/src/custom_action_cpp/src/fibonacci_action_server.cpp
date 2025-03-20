#include <memory>
#include <thread>

#include "custom_action_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_action_cpp/visibility_control.h"

namespace custom_action_cpp {
class FibonacciActionServer : public rclcpp::Node {
public:
  using Fibonacci = custom_action_interfaces::action::Fibonacci;
  using FibonacciGoal = custom_action_interfaces::action::Fibonacci::Goal;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  CUSTOM_ACTION_CPP_PUBLIC
  explicit FibonacciActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("fibonacci_action_server", options) {
    using namespace std::placeholders;

    auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
                              std::shared_ptr<const FibonacciGoal> goal) {
      RCLCPP_INFO(this->get_logger(), "received goal request with order %d ",
                  goal->order);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel =
        [this](std::shared_ptr<GoalHandleFibonacci> goal_handle) {
          RCLCPP_INFO(this->get_logger(), "received request to cancel goal");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto handle_accepted =
        [this](std::shared_ptr<GoalHandleFibonacci> goal_handle) {
          RCLCPP_INFO(this->get_logger(), "received request to accept goal");
          auto execute_in_thread = [this, goal_handle]() {
            return this->execute(goal_handle);
          };
          std::thread{execute_in_thread}.detach();
        };
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this, "fibonacci", handle_goal, handle_cancel, handle_accepted);
  }

private:
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto &sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};
} // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::FibonacciActionServer)