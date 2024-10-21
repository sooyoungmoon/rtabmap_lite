#include "rtabmap_lite/RTABMapLiteNode.hpp"

using namespace std::chrono_literals;

RTABMapLiteNode::RTABMapLiteNode() : Node("rtabmap_lite_node"), count_(0)
{
  RCLCPP_INFO(this->get_logger(), "RTABMapLiteNode::RTABMapLiteNode() was called");
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&RTABMapLiteNode::timer_callback, this));
}

void RTABMapLiteNode::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "RTABMapLiteNode::timer_callback() was called");
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  //publisher_->publish(message);
}

/*
class RTABMapLiteNode : public rclcpp::Node
{
  public:
    RTABMapLiteNode()
    : Node("RTABMapLiteNode"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&RTABMapLiteNode::timer_callback, this));
    }

    private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

};
*/

