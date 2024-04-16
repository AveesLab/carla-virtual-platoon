#include "ros2node.hpp"

Ros2Node::Ros2Node()
  : rclcpp::Node("Command_PC", rclcpp::NodeOptions() 
                   .allow_undeclared_parameters(true)
                               .automatically_declare_parameters_from_overrides(true)) 
{

}

