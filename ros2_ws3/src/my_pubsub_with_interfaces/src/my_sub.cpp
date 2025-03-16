#include <memory>

#include "my_interfaces/msg/num.hpp"
#include "rclcpp/rclcpp.hpp"

using Msg = my_interfaces::msg::Num;

class MySubscriber: public rclcpp::Node
{
public:
    MySubscriber()
        : Node("my_subscriber")
    {
        auto topic_cb = [this](const Msg& msg)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "I heared: '" << msg.num << "'");
        };
        m_sub_ = this->create_subscription<Msg>("topic", 10, topic_cb);
    }

private:
    rclcpp::Subscription<Msg>::SharedPtr m_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriber>());
  rclcpp::shutdown();
  return 0;
}