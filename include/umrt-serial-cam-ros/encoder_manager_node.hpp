#ifndef ENCODER_MANAGER_NODE_HPP
#define ENCODER_MANAGER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/SetBool.hpp>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <csignal> // For signal handling (SIGINT, SIGTERM)
#include <sys/types.h> // For pid_t
#include <sys/wait.h>  // For waitpid


// Is the namespace necessary 
namespace EncoderManager
{

// Struct to hold process information
struct EncoderProcessInfo {
    pid_t pid;
    // You could add other metadata here if needed, e.g., start_time, status_flags
};

class EncoderManager: public rclcpp::Node
{
public:
    // Constructor
    EncoderManager();

    // Destructor to ensure all child processes are cleaned up
    ~EncoderManager();

private:

    // Service callback function
    void encoderControlCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response,
        const std::string& camera_namespace);

    // Helper function to start an encoder process
    bool startEncoder(const std::string& camera_namespace, std::string& message);

    // Helper function to stop an encoder process
    bool stopEncoder(const std::string& camera_namespace, std::string& message);

    // Map to store active encoder processes (process ID of the `ros2 launch` command)
    std::map<std::string, EncoderProcessInfo> encoder_processes_;

    // Map to store launch arguments for each camera
    std::map<std::string, std::map<std::string, std::string>> encoder_launch_args_;

    // Vector to hold shared pointers to service servers
    // This is necessary to keep them alive
    std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> services_;

    // Static function for error handling child process
    // static void handleChildSignal(int signum);
};

} //  namespace EncoderManager

#endif //ENCODER_MANAGER_NODE_HPP
