#include <iostream>

#include "more_interfaces/msg/address_book.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using Msg = more_interfaces::msg::AddressBook;

class AddressBookPublisher : public rclcpp::Node {
public:
  AddressBookPublisher() : Node("address_book_publisher") {
    m_address_book_publisher_ = this->create_publisher<Msg>("address_book", 10);

    auto publish_cb = [this]() -> void {
        auto msg = Msg();
        msg.first_name = "John";
        msg.last_name = "Doe";
        msg.phone_number = "123456";
        msg.phone_type = msg.PHONE_TYPE_MOBILE;

        std::cout << "Publishing contact\nFirst: " << msg.first_name << " Last: " << msg.last_name << "\n";
        this->m_address_book_publisher_->publish(msg);
    };

    m_timer_ = this->create_wall_timer(1s, publish_cb);
  }

private:
  rclcpp::Publisher<Msg>::SharedPtr m_address_book_publisher_;
  rclcpp::TimerBase::SharedPtr m_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<AddressBookPublisher>());

    rclcpp::shutdown();
    return 0;
}