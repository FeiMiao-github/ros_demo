#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
// #include "my_interfaces/srv/add_three_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
// using AddThreeInts = my_interfaces::srv::AddThreeInts;
rclcpp::Node::SharedPtr g_node = nullptr;

using RequestHeadSharedPtr = std::shared_ptr<rmw_request_id_t>;
using RequestSharedPtr = std::shared_ptr<AddTwoInts::Request>;
using ResponseSharedPtr = std::shared_ptr<AddTwoInts::Response>;

// using Request3SharedPtr = std::shared_ptr<AddThreeInts::Request>;
// using Response3SharedPtr = std::shared_ptr<AddThreeInts::Response>;

void handle_service(const RequestHeadSharedPtr request_header,
                    const RequestSharedPtr request,
                    const ResponseSharedPtr response) {
  (void)request_header;
  RCLCPP_INFO(g_node->get_logger(), "request: %" PRId64 " + %" PRId64,
              request->a, request->b);
  response->sum = request->a + request->b;
}

// void handle_three_ints_service(const RequestHeadSharedPtr request_header,
//     const Request3SharedPtr request,
//     const Response3SharedPtr response)
// {
//     (void) request_header;
//     RCLCPP_INFO(g_node->get_logger(), "request: %" PRId64 " + %" PRId64 " + %" PRId64,
//         request->a, request->b, request->c);
//     response->sum = request->a + request->b + request->c;
// }

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  g_node = rclcpp::Node::make_shared("my_first_service");
  auto server = g_node->create_service<AddTwoInts>("add_two_ints", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
