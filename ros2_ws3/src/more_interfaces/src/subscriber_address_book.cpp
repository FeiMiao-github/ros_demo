#include <iostream>
#include <rclcpp/logging.hpp>

#include "more_interfaces/msg/address_book.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using Msg = more_interfaces::msg::AddressBook;

class AddressBookSubscriber : public rclcpp::Node {
public:
  AddressBookSubscriber() : Node("address_book_publisher") {
    auto sub_cb = [this](const Msg &msg) {
      RCLCPP_INFO(this->get_logger(), "I heared contact:\n");
      RCLCPP_INFO(this->get_logger(), "First: %s Last: %s", msg.first_name.c_str(), msg.last_name.c_str());
      RCLCPP_INFO(this->get_logger(), "phone_number: %s", msg.phone_number.c_str());
      RCLCPP_INFO(this->get_logger(), "phone_type: %d", msg.phone_type);
    };
    m_address_book_subscriber_ =
        this->create_subscription<Msg>("address_book", 10, sub_cb);
  }

private:
  rclcpp::Subscription<Msg>::SharedPtr m_address_book_subscriber_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<AddressBookSubscriber>());

  rclcpp::shutdown();
  return 0;
}