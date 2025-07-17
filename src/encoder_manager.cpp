#include <rclcpp/rclcpp.hpp>
#include "umrt-serial-cam-ros/encoder_manager_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Create a MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Create an instance of your EncoderManager node
    auto encoder_manager_node = std::make_shared<umrt_serial_cam_ros::EncoderManager>();

    // Add your node to the executor
    executor.add_node(encoder_manager_node);

    // Spin the executor, which will handle callbacks in multiple threads
    // This allows service calls to be processed concurrently.
    executor.spin();

    rclcpp::shutdown();
    return 0;
} 