#include "rtabmap_odom_lite/RTABMapOdomLiteNode.hpp"

using namespace std::chrono_literals;

RTABMapOdomLiteNode::RTABMapOdomLiteNode() : Node("rtabmap_odom_lite_node"), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&RTABMapOdomLiteNode::timer_callback, this));
}

void RTABMapOdomLiteNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}
