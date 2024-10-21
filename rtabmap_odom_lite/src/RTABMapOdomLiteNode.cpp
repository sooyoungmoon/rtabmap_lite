#include "rtabmap_odom_lite/RTABMapOdomLiteNode.hpp"

using namespace std::chrono_literals;

using namespace std::chrono_literals;


RTABMapOdomLiteNode::RTABMapOdomLiteNode() : Node("rtabmap_odom_lite_node"), count_(0), topicQueueSize_(10), syncQueueSize_(2),
qos_(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT)
{
  RCLCPP_INFO(this->get_logger(), "RTABMapOdomLiteNode::RTABMapOdomLiteNode() was called");
  dataCallbackGroup_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&RTABMapOdomLiteNode::timer_callback, this));
  init(false, true, false);

}

void RTABMapOdomLiteNode::init(bool stereoParams, bool visParams, bool icpParams)
{
  RCLCPP_INFO(this->get_logger(), "RTABMapOdomLiteNode::init() was called");
  onOdomInit();
}

void RTABMapOdomLiteNode::onOdomInit()
{
  RCLCPP_INFO(this->get_logger(), "RTABMapOdomLiteNode::onOdomInit() was called");
  rclcpp::SubscriptionOptions options;
	options.callback_group = dataCallbackGroup_;

  int qosCamInfo = this->declare_parameter("qos_camera_info", (int)qos());

  image_transport::TransportHints hints(this);
  image_mono_sub_.subscribe(this, "/camera/color/image_raw", hints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability((rmw_qos_reliability_policy_t)qos()).get_rmw_qos_profile(), options);
  image_depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw", hints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability((rmw_qos_reliability_policy_t)qos()).get_rmw_qos_profile(), options);
  info_sub_.subscribe(this, "/camera/color/camera_info", rclcpp::QoS(topicQueueSize_).reliability((rmw_qos_reliability_policy_t)qosCamInfo).get_rmw_qos_profile(), options);

  bool approxSync = true;
	approxSync = this->declare_parameter("approx_sync", approxSync);
  double approxSyncMaxInterval = 0.0;
	approxSyncMaxInterval = this->declare_parameter("approx_sync_max_interval", approxSyncMaxInterval);
  
  if(approxSync)
  {
    approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_);
    if(approxSyncMaxInterval > 0.0)
      approxSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
    approxSync_->registerCallback(std::bind(&RTABMapOdomLiteNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }
  else
  {
    exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_);
    exactSync_->registerCallback(std::bind(&RTABMapOdomLiteNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }
}

void RTABMapOdomLiteNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

void RTABMapOdomLiteNode::callback(
		const sensor_msgs::msg::Image::ConstSharedPtr image,
		const sensor_msgs::msg::Image::ConstSharedPtr depth,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
  RCLCPP_INFO(this->get_logger(), "Received image, depth and camera info");
}

void RTABMapOdomLiteNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  RCLCPP_INFO(this->get_logger(), "Received image");
}

void RTABMapOdomLiteNode::depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  RCLCPP_INFO(this->get_logger(), "Received depth image");
}