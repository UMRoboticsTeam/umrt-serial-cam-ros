#include "project-name/example_node.hpp"

#include <iostream>

ExampleNode::ExampleNode() : Node("example") {}

void ExampleNode::example() {
    std::cout << "Hello World" << std::endl;
}
