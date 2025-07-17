#ifndef UMRT_ENCODER_MANAGER_NODE_HPP
#define UMRT_ENCODER_MANAGER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp> 
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/subscription_options.hpp>

#include <map>
#include <string>
#include <memory>
#include <vector>

namespace umrt_serial_cam_ros
{

struct EncoderConfig
{
    std::string namespace_name;
    std::string in_raw_topic;
    std::string out_ffmpeg_topic;
    std::string encoding;
    std::string qmax;
    std::string preset;
    std::string tune;
    std::string bit_rate;
    std::string gop_size;
};

class EncoderManager : public rclcpp::Node
{

public: 
    //  Constructor
    EncoderManager();
    //  Destructor
    ~EncoderManager();

private:
    void encoderControlCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response,
        const std::string &camera_namespace);

    void rawImageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr &msg,
        const std::string &camera_namespace);

    bool startEncoder(const std::string &camera_namespace);
    bool stopEncoder(const std::string &camera_namespace);

    // Map to store configuration for each camera
    std::map<std::string, EncoderConfig> encoder_configs_;

    // Map to store image_transport publishers (the "encoders")
    std::map<std::string, image_transport::Publisher> encoder_publishers_;

    // Map to store image_transport subscribers (listening to raw images)
    std::map<std::string, image_transport::Subscriber> raw_image_subscribers_;

    // Map to store image_transport instances (one per camera/namespace)
    std::map<std::string, std::shared_ptr<image_transport::ImageTransport>> image_transports_;

    // Map to store service servers for each camera
    std::map<std::string, rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> control_services_;


};   //  class EncoderManager


}   //  namespace umrt_serial_cam_ros

#endif //   UMRT_ENCODER_MANAGER_NODE_HPP

