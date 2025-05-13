#ifndef PROJECT_NAME_EXAMPLE_NODE_HPP
#define PROJECT_NAME_EXAMPLE_NODE_HPP

#include <rclcpp/rclcpp.hpp>

/**
 * This is an example node provided to demonstrate file structure and CMake configuration.
 */
class ExampleNode : public rclcpp::Node {
public:
    /**
     * Constructs an ExampleNode.
     */
    ExampleNode();

    /**
     * Prints a string to stdout.
     */
    void example();
};


#endif //PROJECT_NAME_EXAMPLE_NODE_HPP
