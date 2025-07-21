#include "umrt-serial-cam-ros/encoder_manager_node.hpp"
#include <functional> // For std::bind

namespace umrt_serial_cam_ros
{

/*
 * Public Functions 
 */

/**
 * Constructor
 */
EncoderManager::EncoderManager():Node("encoder_manager")
{  
    //  Info
    RCLCPP_INFO(this->get_logger(), "Encoder Manager Node Started");

    // Initialize encoder configurations
    encoder_configs_["rover_cam0"] = {
        "rover",
        "/rover/image_raw",
        "/rover",
        "h264_nvenc",
        "31",
        "p1",
        "ll",
        "2000000",
        "15"};

    encoder_configs_["arm_cam0"] = {
        "arm0",
        "/arm0/image_raw",
        "/arm0",
        "h264_nvenc",
        "31",
        "p1",
        "ll",
        "2000000",
        "15"};

    encoder_configs_["arm_cam1"] = {
        "arm1",
        "/arm1/image_raw",
        "/arm1",
        "h264_nvenc",
        "31",
        "p1",
        "ll",
        "2000000",
        "15"};

    //  Create services for each camera 
    for (const auto &camera : encoder_configs_)
    {
        const std::string &cam_name = camera.first;
        std::string service_name = "/" + cam_name + "/bool";

        encoder_status_[cam_name] = false;

        //  Use std::bind to pass the camera_namespace
        control_services_[cam_name] = this->create_service<std_srvs::srv::SetBool>(
            service_name, 
            std::bind(&EncoderManager::encoderControlCallback, this,
                      std::placeholders::_1, std::placeholders::_2, cam_name)
        );

         RCLCPP_INFO(this->get_logger(), "Created service: %s for %s", service_name.c_str(), cam_name.c_str());
    }

}   //  Constructor

/**
 * Destructor
 */
EncoderManager::~EncoderManager()
{
    // Stop any running encoders on shutdown
    for (const auto &encoder : encoder_status_)
    {
        const std::string &cam_name = encoder.first;
        if (encoder.second)
        {
            RCLCPP_INFO(this->get_logger(), "Stopping encoder for %s during shutdown.", cam_name.c_str());
            stopEncoder(cam_name);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Encoder Manager Node Shutting Down.");
}   //  Destructor

/*
 * Private Functions 
 */

 /**
 * encoderControlCallback - Backbone essentially, will get the bool service and check to start or stop the encoders.
 */
 void EncoderManager::encoderControlCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response,
    const std::string &camera_namespace)
{

    //  Get the SetBool Service request and then find out if it is start or stop
    bool command_bool = request->data;
    std::string command = command_bool ? "start" : "stop"; // Maybe we can set this to an int like 0 or 1 

    RCLCPP_INFO(this->get_logger(), "Received command: %s for %s", command.c_str(), camera_namespace.c_str());

    //  Get current encoder status
    bool current_encoder_status = encoder_status_[camera_namespace];

    //  Check if command is valid
    //  Start Command - To start encoder
    if (command == "start")
    {
        //  Check if encoder for that namespace is already running 
        //  If it is running, throw a warning
        if (current_encoder_status)
        {
            response->success = false;
            response->message = "Encoder " + camera_namespace + " is already running.";
            RCLCPP_WARN(this->get_logger(), response->message.c_str()); 
        }
        //  Else if its not running, attempt to start 
        else 
        {   
            //  Successful start
            if (startEncoder(camera_namespace))
            {
                response->success = true;
                response->message = "Encoder " + camera_namespace + " started.";
            }
            //  Failed start
            else
            {
                response->success = false;
                response->message = "Failed to start encoder " + camera_namespace + ".";
            }
        }
    }
    //  Stop Command - To stop an encoder
    else if (command == "stop")
    {
        //  Check if encoder for that namespace has stopped
        //  If it is stopped, thrown a warning
        if (!current_encoder_status)
        {
            response->success = false;
            response->message = "Encoder " + camera_namespace + " is already stopped.";
            RCLCPP_WARN(this->get_logger(), response->message.c_str());
        }
        //  Else if it is not stopped, attempt to stop
        else 
        {
            if (stopEncoder(camera_namespace))
            {
                response->success = true;
                response->message = "Encoder " + camera_namespace + " stopped.";
            }
            else
            {
                response->success = false;
                response->message = "Failed to stop encoder " + camera_namespace + ".";
            }
        }
    }
    //  Invalid Command
    else 
    {
        response->success = false;
        response->message = "Unknown command: " + command + ". Use 'start' or 'stop'";
        RCLCPP_ERROR(this->get_logger(), response->message.c_str());
    }

}   //  encoderControlCallback

/**
 * rawImageCallback - Function is to forward the raw images to the image_transport publisher
 *                      Hopefully the ffmpeg_image_transport will the the encoding
 */
void EncoderManager::rawImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg,
    const std::string &camera_namespace)
{
    if (encoder_publishers_.count(camera_namespace) > 0)
    {
        encoder_publishers_[camera_namespace].publish(msg);
    }
}   //  rawImageCallback

/**
 * startEncoder Function - This will start an encoder which is essentially a republish
 */
bool EncoderManager::startEncoder(const std::string &camera_namespace)
{
    //  Try to start encoder
    try
    {

        // Get config for a given camera
        const EncoderConfig &config = encoder_configs_.at(camera_namespace);

        // Create a new ImageTransport instance for this camera
        image_transports_[camera_namespace] = std::make_shared<image_transport::ImageTransport>(shared_from_this());

        // The second argument specifies the transport plugin. "ffmpeg" is what we want.
        image_transport::ImageTransport& it = *image_transports_[camera_namespace];

        // Create the image_transport publisher configured for ffmpeg encoding
        // Publisher
        // QoS
        rclcpp::QoS publisher_qos(1);
        publisher_qos.reliable();
        publisher_qos.durability_volatile();
        rclcpp::PublisherOptions pub_options;

        // Transport Hints - for the ffmpeg image transport plugin
        // image_transport::TransportHints pub_transport_hints(shared_from_this().get(), "ffmpeg");

        encoder_publishers_[camera_namespace] = it.advertise(
            config.out_ffmpeg_topic,
            publisher_qos.get_rmw_qos_profile()
        ); // Publisher

        RCLCPP_INFO(this->get_logger(), "Advertised encoder topic: %s using encoder: %s",
            config.out_ffmpeg_topic.c_str(), config.encoding.c_str());

        // Create the image_transport subscriber to listen to the raw image topic
        // Subscriber
        // QoS
        rclcpp::QoS subscriber_qos(1);
        subscriber_qos.reliable();  //  I CANT MAKE THE DECODER BEST EFFORT AAAAAAAA
        subscriber_qos.durability_volatile();
        rclcpp::SubscriptionOptions sub_options;

        // Transport Hints - for the ffmpeg image transport plugin
        image_transport::TransportHints sub_transport_hints(shared_from_this().get(), "raw");

        raw_image_subscribers_[camera_namespace] = it.subscribe(
            config.in_raw_topic,
            subscriber_qos.get_rmw_qos_profile(),
            [this, camera_namespace](const sensor_msgs::msg::Image::ConstSharedPtr &msg) 
            {
                this->rawImageCallback(msg, camera_namespace);
            },
            shared_from_this(),
            &sub_transport_hints,
            sub_options
        ); // Subscriber
                
        // ROS INFO
        RCLCPP_INFO(this->get_logger(), "Encoder for %s started. Subscribing to %s, Publishing to %s (ffmpeg).",
                    camera_namespace.c_str(), config.in_raw_topic.c_str(), config.out_ffmpeg_topic.c_str());

        //  Set the encoder status to true
        encoder_status_[camera_namespace] = true;
                
        //  Successful start
        return true;
    }
    //  Catch exception if try fails
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start encoder %s: %s", camera_namespace.c_str(), e.what());
        encoder_status_[camera_namespace] = false;
        return false;
    }
}   //  startEncoder

/**
 * stopEncoder Function - Stop the Encoder given the camera namespace
 */
bool EncoderManager::stopEncoder(const std::string &camera_namespace)
{   
    //  Check if it is running, if so erase
    if (encoder_publishers_.count(camera_namespace) > 0)
    {
        //  Must erase, as erasing only one leaves the other things hanging 
        raw_image_subscribers_.erase(camera_namespace); //  Erase Subscribers
        encoder_publishers_.erase(camera_namespace);    //  Erase Publishers
        image_transports_.erase(camera_namespace);  //  Erase Image Transport Instances

        encoder_status_[camera_namespace] = false;

        RCLCPP_INFO(this->get_logger(), "Encoder for %s stopped.", camera_namespace.c_str());
        return true;
    }
    //  If it is 0, means it isnt running or already stopped
    else
    {
        RCLCPP_WARN(this->get_logger(), "Encoder %s is not running or already stopped.", camera_namespace.c_str());
        return false;
    }

}   //  stopEncoder

}   //  namespace umrt_serial_cam_ros