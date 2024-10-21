#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class BaseNode : public rclcpp::Node
{
  public:
    BaseNode()
    : Node("base_node")
    {
      subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "topic_in", 10, [this](const std_msgs::msg::Float64 &msg) { 
            float_msg.data = msg.data * 2.; 
        });
      publisher_ = this->create_publisher<std_msgs::msg::Float64>("topic_out", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&BaseNode::timer_callback, this));
    }

  private:
    std_msgs::msg::Float64 float_msg;

    void timer_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", float_msg.data);
      publisher_->publish(float_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseNode>());
  rclcpp::shutdown();
  return 0;
}

