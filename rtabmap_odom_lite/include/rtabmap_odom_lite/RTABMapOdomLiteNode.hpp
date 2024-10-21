#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// header files to pub/sub image data
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <image_transport/subscriber_filter.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/logging.hpp"

// header files to use message_filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class RTABMapOdomLiteNode : public rclcpp::Node
{
    public:
     RTABMapOdomLiteNode();     
     
    private:
        void timer_callback();    
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;

        image_transport::SubscriberFilter image_mono_sub_;
	    image_transport::SubscriberFilter image_depth_sub_;
	    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
	    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MyApproxSyncPolicy;
	    message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	    typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MyExactSyncPolicy;
	    message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

        void callback(
			const sensor_msgs::msg::Image::ConstSharedPtr image,
			const sensor_msgs::msg::Image::ConstSharedPtr depth,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo);

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
        void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

        void init(bool stereoParams, bool visParams, bool icpParams);
        void onOdomInit();
        rmw_qos_reliability_policy_t qos() const {return qos_;}
        int topicQueueSize_;
        int syncQueueSize_;

        rclcpp::CallbackGroup::SharedPtr dataCallbackGroup_;

        // Parameters
        rmw_qos_reliability_policy_t qos_;

};