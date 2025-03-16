#include <chrono>
#include <memory>

#include "my_interfaces/msg/num.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

using Msg = my_interfaces::msg::Num;

class MyPublisher: public rclcpp::Node {
public:
    MyPublisher()
        : Node("my_publisher"), m_count_(0)
    {
        m_publisher_ = this->create_publisher<Msg>("topic", 10);

        auto timer_cb = [this]() {
            auto msg = Msg();
            msg.num = this->m_count_++;
            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing '" << msg.num << "'");
            m_publisher_->publish(msg);
        };

        m_timer_ = this->create_wall_timer(1ms, timer_cb);     
    }

private:
    rclcpp::TimerBase::SharedPtr m_timer_;
    rclcpp::Publisher<my_interfaces::msg::Num>::SharedPtr m_publisher_;
    size_t m_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();
  return 0;
}