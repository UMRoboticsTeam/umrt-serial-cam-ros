#include "umrt-serial-cam-ros/encoder_manager_node.hpp"
#include <iostream>
#include <memory> // For std::make_shared
#include <unistd.h> // For fork, execve, getpid, setpgid
#include <array>    // For std::array
#include <sstream>  // For std::stringstream

namespace EncoderManager
{

// std::map<pid_t, std::string> global_child_pids_to_name; 

/*
 * Public Functions 
 */

/**
 * Constructor
 */
EncoderManager::EncoderManager():Node("encoder_manager")
{
    RCLCPP_INFO(this->get_logger(), "Encoder Manager Node Started");

    //  Initialize the processes
    encoder_processes_ = 
    {
        {"rover_cam0", {0}},
        {"arm_cam0", {0}},
        {"arm_cam1", {0}}
    };


    //  Initialize the arguments 
    //  ADD LAUNCH ARGUMENTS HERE
    encoder_launch_args_ = {
        {"rover_cam0", 
            {
                {"namespace", "rover"},
                {"in_raw", "/rover/image_raw"},
                {"out_ffmpeg", "/rover/ffmpeg"},
                {"encoding", "h264_nvenc"},
                {"qmax", "31"},
                {"preset", "p1"},
                {"tune", "ll"},
                {"bit_rate", "2000000"},
                {"gop_size", "15"}
            }
        },
        {"arm_cam0", 
            {
                {"namespace", "arm0"},
                {"in_raw", "/arm0/image_raw"},
                {"out_ffmpeg", "/arm0/ffmpeg"},
                {"encoding", "h264_nvenc"},
                {"qmax", "31"},
                {"preset", "p1"},
                {"tune", "ll"},
                {"bit_rate", "2000000"},
                {"gop_size", "15"}
            }
        },
        {"arm_cam1", 
            {
                {"namespace", "arm1"},
                {"in_raw", "/arm1/image_raw"},
                {"out_ffmpeg", "/arm1/ffmpeg"},
                {"encoding", "h264_nvenc"},
                {"qmax", "31"},
                {"preset", "p1"},
                {"tune", "ll"},
                {"bit_rate", "2000000"},
                {"gop_size", "15"}
            }
        }
    };

    // Create a service for each camera 
    for (const auto& process : encoder_processes_) 
    {
        const std::string& cam_name = process.first;
        std::string service_name = "/" + cam_name + "bool";
    
        //  Create service
        services_.push_back(
            this->create_service<std_srvs::srv::SetBool>(
                service_name,
                std::bind(&EncoderManager::encoderControlCallback, this, std::placeholders::_1, std::placeholders::_2, cam_name)
            )
        );

        RCLCPP_INFO(this->get_logger(), "Created service: %s for %s", service_name.c_str(), cam_name.c_str() );
    }

}

/**
 * Destructor
 */
EncoderManager::~EncoderManager()
{
    RCLCPP_INFO(this->get_logger(), "Encoder Manager Node Shutting Down. Terminating child processes...");
    for (auto const& [cam_name, process_info] : encoder_processes_) {
        if (process_info.pid > 0 && kill(process_info.pid, 0) == 0) { // Check if process still exists
            std::string msg;
            stopEncoder(cam_name, msg); // Attempt to stop it gracefully
            RCLCPP_INFO(this->get_logger(), "Cleanup: %s", msg.c_str());
        }
    }
}


/*
 * Private Functions 
 */

/**
 * encoderControlCallback Function
 */
void EncoderManager::encoderControlCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response,
    const std::string& camera_namespace)
{

    // Get the request data
    bool command_bool = request->data;
    std::string command = command_bool ? "start":"stop";

    // Successful Camera Namespace
    RCLCPP_INFO(this->get_logger(), "Received command: %s for %s", command.c_str(), camera_namespace.c_str());

    // Check if command is valid 
    // Start Command - To Start Encoder 
    if (command == "start")
    {
        // Check if encoder for that namespace is already running
        // if so
        if (encoder_processes_[camera_namespace].pid != 0 && kill(encoder_processes_[camera_namespace].pid, 0) == 0 )
        {
            response->success = false;
            response->message = "Encoder: " + camera_namespace + " is already running.\n";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        }
        // otherwise run it
        else
        {
            encoder_processes_[camera_namespace].pid = 0;
            response->success = startEncoder(camera_namespace, response->message);
        }
    }

    // Stop Command - To Stop Encoder
    else if (command == "stop")
    {
        // Check if encoder for that namespace is already stopped
        // if so
        if (encoder_processes_[camera_namespace].pid == 0 || kill(encoder_processes_[camera_namespace].pid, 0) != 0)
        {
            response->success = false;
            response->message = "Encoder: " + camera_namespace + " is already stopped.\n";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        }
        // otherwise stop it
        else
        {
            response->success = stopEncoder(camera_namespace, response->message);
        }
    }

    // Unknown Command
    else
    {
        response->success = false;
        response->message = "Unknown command: " + command + "Use 'start' or 'stop'.\n";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }

    return;
}   //  encoderControlCallback

/**
 * startEncoder Function
 */
bool EncoderManager::startEncoder(const std::string& camera_namespace, std::string& message) 
{
    pid_t pid = fork();

     //  Fork failed
    if (pid == -1)
    {
        message = "Failed to fork process for " + camera_namespace + ": " + std::string(strerror(errno));
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
    else if (pid == 0)
    {
        setpgid(0, 0);
        
        //  Setup the command to launch
        std::vector<const char*> cmd = { "ros2", "launch", "umrt-serial-cam-ros", "encode.launch.py"};
        //  Add Launch Argument
        for (const auto& launch_arg : encoder_launch_args_[camera_namespace]) 
        {
            std::string launch_arg = launch_arg.first + ":=" + launch_arg.second;
            cmd.push_back(strdup(launch_arg.c_str())); // strdup to allocate memory for const char*
        }
        //  Need this to pass to execve
        cmd.push_back(nullptr);

        /*
            I HAVE NO IDEA WHAT THIS PART DOES - the python way is much simpler if there is a simpler to do this please let me know
        */
        // Prepare environment variables (CRITICAL for ros2 to work)
        // Copy parent's environment and then override/add specific ROS 2 variables.
        extern char **environ;
        std::vector<const char*> envp_vec;
        std::map<std::string, std::string> child_env_map;

        // Copy existing environment
        for (char **env = environ; *env != 0; env++) 
        {
            std::string env_pair(*env);
            size_t eq_pos = env_pair.find('=');
            if (eq_pos != std::string::npos) {
                child_env_map[env_pair.substr(0, eq_pos)] = env_pair.substr(eq_pos + 1);
            }
        }

        // Override/Add specific ROS 2 environment variables
        child_env_map["ROS_DOMAIN_ID"] = std::to_string(this->get_node_options().domain_id()); // Use node's domain ID
        child_env_map["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp";
        child_env_map["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4";

        // Convert map to vector of const char* (using strdup for persistence)
        std::vector<std::string> env_strings_for_exec;
        for (const auto& pair : child_env_map) 
        {
            env_strings_for_exec.push_back(pair.first + "=" + pair.second);
        }
        for (const auto& s : env_strings_for_exec)
        {
            envp_vec.push_back(strdup(s.c_str()));
        }
        envp_vec.push_back(nullptr); // Null-terminate envp

        RCLCPP_INFO(this->get_logger(), "Child process attempting to exec: %s %s %s %s ...", cmd[0], cmd[1], cmd[2], cmd[3]);

        // Execute the command (replaces the child process image)
        execve("/opt/ros/humble/bin/ros2", const_cast<char* const*>(cmd.data()), const_cast<char* const*>(envp_vec.data()));

        // If execve returns, it failed
        std::cerr << "Child process: execve failed for " << camera_namespace << ": " << strerror(errno) << std::endl;

        // Clean up duplicated strings if execve fails
        for (const char* ptr : cmd) 
        {
            if (ptr != nullptr && ptr != cmd[0] && ptr != cmd[1] && ptr != cmd[2] && ptr != cmd[3]) 
            {
                free(const_cast<char*>(ptr));
            }
        }
        for (const char* ptr : envp_vec) 
        {
            if (ptr != nullptr) 
            {
                free(const_cast<char*>(ptr));
            }
        }
        _exit(1); // Exit child process immediately on failure
    } 
    else 
    {
        // Parent process
        encoder_processes_[camera_namespace].pid = pid;
        message = "Encoder " + camera_namespace + " started (PID: " + std::to_string(pid) + ")";
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        return true;
    }

}   //  startEncoder


/**
 *  stopEncoder Function
 */
bool EncoderManager::stopEncoder(const std::string& camera_namespace, std::string& message)
{

    // Get Process ID for camera 
    pid_t pid = encoder_processes_[camera_namespace].pid;

    if (pid == 0) 
    {
        message = "Encoder " + camera_namespace + " is not running.";
        RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Attempting to stop encoder %s (PID: %d)", camera_namespace.c_str(), pid);


    //  Try SIGINT basically CTRL+C
    if (kill(-pid, SIGINT) == 0) { // Send to process group
        RCLCPP_INFO(this->get_logger(), "Sent SIGINT to encoder %s (PID: %d)", camera_namespace.c_str(), pid);
    }
    //  If SIGINT doesn't work 
    else 
    {
        if (errno == ESRCH) 
        {
            message = "Encoder " + camera_namespace + " process not found.";
            RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
            encoder_processes_[camera_namespace].pid = 0; // Clear PID
            return true; // Consider it stopped
        }
        message = "Failed to send SIGINT to encoder " + camera_namespace + ": " + std::string(strerror(errno));
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false; // Failed to send signal
    }

    //  Wait for process to terminate
    int status;
    pid_t result = waitpid(pid, &status, 0); // Wait for specific PID, 0 for blocking

    // The process is terminated
    if (result == pid)
    {
        encoder_processes_[camera_namespace].pid = 0; // Clear PID
        // global_child_pids_to_name.erase(pid); // Remove from global tracker
        message = "Encoder " + camera_namespace + " stopped.";
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        return true;
    }
    // there is a waitpid error 
    else if (result == -1)
    {
        message = "Error waiting for encoder " + camera_namespace + ": " + std::string(strerror(errno));
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
    // another error with pid  
    else 
    {
        message = "Unexpected waitpid result for encoder " + camera_namespace;
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }

} // stopEncoder

/**
 * handleChildSignal Function
 */
// static void EncoderManager::handleChildSignal(int signum)
// {

// }


}   //  namespace EncoderManager