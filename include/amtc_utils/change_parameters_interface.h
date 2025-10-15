#pragma once
#include <amtc_utils/params_helper.h>

#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include <rcl_interfaces/srv/detail/describe_parameters__struct.hpp>
#include <rcl_interfaces/srv/detail/list_parameters__struct.hpp>
#include <rcl_interfaces/srv/detail/set_parameters_atomically__struct.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <string_view>
#include <vector>

namespace amtc {

// class to set the parameters of a node from a list of parameter sets on the global parameters
std::vector<rcl_interfaces::msg::ParameterDescriptor>
copy_if_prefix(const std::vector<rcl_interfaces::msg::ParameterDescriptor> &in, const std::string_view prefix);

class ChangeParametersInterface {

public:
  ChangeParametersInterface(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
                            rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
                            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface,
                            rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface,
                            rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
                            std::string node_name, std::vector<std::string> basenames);

  ChangeParametersInterface(rclcpp::Node &node, std::string node_name, std::vector<std::string> basenames);
  void configure();
  void activate();

  bool switch_to_parameters(const std::string &basename);
  bool switch_to_parameters(const std::vector<rclcpp::Parameter> &parameters);

private:
  void get_parameter_descriptions();
  void get_parameter_set();

  std::string node_name_;
  std::vector<std::string> parameter_namespaces_;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_interface_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;

  rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::map<std::string, std::vector<rclcpp::Parameter>> parameter_set_;
  std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descriptions_;
};

} // namespace amtc
