#include <algorithm>
#include <amtc_utils/params_helper.h>
#include <ranges>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

namespace amtc {

std::vector<rclcpp::Parameter>
declare_parameters(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface,
                   std::vector<rclcpp::Parameter> default_values, std::string base_name) {

  std::vector<rclcpp::Parameter> retval;
  retval.reserve(default_values.size());

  for (auto default_value : default_values) {
    rclcpp::Parameter renamed_param;
    if (!base_name.empty()) {
      renamed_param =
          rclcpp::Parameter(base_name + "." + default_value.get_name(), default_value.get_parameter_value());
    } else {
      renamed_param = default_value;
    }

    try {

      retval.emplace_back(renamed_param.get_name(), parameter_interface->declare_parameter(
                                                        renamed_param.get_name(), renamed_param.get_parameter_value()));
    } catch (rclcpp::exceptions::UninitializedStaticallyTypedParameterException &e) {
      continue;
    }
  }
  return retval;
  auto msg_view = std::views::transform(default_values, [](auto p) {
    return p.to_parameter_msg();
  });
  std::vector<rcl_interfaces::msg::Parameter> values_msg(msg_view.begin(), msg_view.end());
  return declare_parameters(parameter_interface, values_msg, base_name);
};

std::vector<rclcpp::Parameter>
declare_parameters(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface,
                   std::vector<rcl_interfaces::msg::Parameter> default_values, std::string base_name) {

  std::vector<rclcpp::Parameter> defaults;
  defaults.reserve(default_values.size());
  std::ranges::transform(default_values, std::back_inserter(defaults), [](auto msg) {
    return rclcpp::Parameter::from_parameter_msg(msg);
  });

  return declare_parameters(parameter_interface, defaults, base_name);
};

std::vector<rclcpp::Parameter>
declare_parameters(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface,
                   std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors, std::string base_name) {
  std::vector<rclcpp::Parameter> retval;
  retval.reserve(descriptors.size());
  for (auto desc : descriptors) {

    if (!base_name.empty()) {
      desc.name = base_name + "." + desc.name;
    }

    retval.emplace_back(desc.name,
                        parameter_interface->declare_parameter(desc.name, rclcpp::ParameterType(desc.type), desc));
    if (retval.back().get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      throw rclcpp::exceptions::UninitializedStaticallyTypedParameterException(desc.name);
    }
  }
  return retval;
};
} // namespace amtc
