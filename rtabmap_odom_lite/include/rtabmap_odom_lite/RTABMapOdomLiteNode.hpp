#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// header files to pub/sub image data
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
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

};