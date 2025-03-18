#include <chrono>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MinimalParam: public rclcpp::Node
{
public:
    MinimalParam():
        rclcpp::Node("minimal_parameters_node")
    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.set__description("My first parameter.");

        auto timer_cb = [this]() {
            std::string my_param = get_parameter("my_parameter").as_string();
            RCLCPP_INFO(get_logger(), "Hello %s !", my_param.c_str());
            std::vector<rclcpp::Parameter> all_new_parameters { rclcpp::Parameter("my_parameter", "world")};
            this->set_parameters(all_new_parameters);
        };

        this->declare_parameter("my_parameter", "world");
        m_timer_ = this->create_wall_timer(1s, timer_cb);
    }

private:
    rclcpp::TimerBase::SharedPtr m_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalParam>());
    rclcpp::shutdown();
    return 0;
}