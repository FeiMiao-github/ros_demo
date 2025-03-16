#include <chrono>
#include <cinttypes>

#include "my_interfaces/srv/add_three_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddThreeInts = my_interfaces::srv::AddThreeInts;

rclcpp::Node::SharedPtr g_node = nullptr;

int client_three_int() {
  auto client = g_node->create_client<AddThreeInts>("add_two_ints");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(g_node->get_logger(),
                   "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(g_node->get_logger(), "waiting for service to appear...");
  }

  auto request = std::make_shared<AddThreeInts::Request>();
  request->set__a(41);
  request->set__b(1);
  request->set__c(2);
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(g_node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(g_node->get_logger(), "service call failed :(");
    client->remove_pending_request(result_future);
    return 1;
  }

  auto result = result_future.get();
  RCLCPP_INFO(g_node->get_logger(),
              "result of %" PRId64 " + %" PRId64 " + %" PRId64 " = %" PRId64, request->a,
              request->b, request->c, result->sum);
  return 0;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  g_node = rclcpp::Node::make_shared("my_second_client");

  if (client_three_int() != 0) {
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}